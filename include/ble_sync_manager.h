#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <FS.h>
#include <LittleFS.h>
#include <Update.h>
#include <memory>
#include <string>
#include <vector>
#include "storage_manager.h"
#include "config_manager.h"
#include "screensaver.h"

#define BLE_SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define BLE_CHAR_CTRL_UUID      "19b10001-e8f2-537e-4f6c-d104768a1214"
#define BLE_CHAR_DATA_UUID      "19b10002-e8f2-537e-4f6c-d104768a1214"

#define BLE_TIMEOUT_SECONDS     60

enum BleSyncState {
    BLE_SYNC_INACTIVE = 0,
    BLE_SYNC_ADVERTISING,
    BLE_SYNC_CONNECTED,
    BLE_SYNC_RECEIVING,
    BLE_SYNC_OTA_RECEIVING,
    BLE_SYNC_SUCCESS,
    BLE_SYNC_FAILED,
    BLE_SYNC_TIMEOUT_EXPIRED
};

enum BleCtrlCmd {
    CMD_START_TRANSFER  = 0x01,
    CMD_FINISH_TRANSFER = 0x02,
    CMD_ABORT           = 0x03,
    CMD_LIST_BOOKS      = 0x04,
    CMD_DELETE_BOOK     = 0x05,
    CMD_RENAME_BOOK     = 0x06,
    CMD_SELECT_BOOK     = 0x07,
    CMD_WIFI_SET        = 0x20,  // [slot][ssid_len][ssid][pass...]  (empty ssid clears the slot)
    CMD_SS_CONFIG       = 0x21,  // [enabled][url...]
    CMD_START_OTA       = 0x10,
    CMD_FINISH_OTA      = 0x11,
    CMD_ABORT_OTA       = 0x12
};

enum BleCtrlStatus {
    STATUS_IDLE         = 0x00,
    STATUS_READY        = 0x01,
    STATUS_SUCCESS      = 0x02,
    STATUS_ERROR        = 0x03,
    STATUS_ABORTED      = 0x04,
    STATUS_BOOK_LIST    = 0x05
};

class BleSyncManager : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
private:
    NimBLEServer* pServer = nullptr;
    NimBLECharacteristic* pCtrlChar = nullptr;
    NimBLECharacteristic* pDataChar = nullptr;

    BleSyncState current_state = BLE_SYNC_INACTIVE;
    uint32_t start_time = 0;
    uint32_t total_expected_bytes = 0;
    uint32_t bytes_received = 0;
    char incoming_filename[48] = {0};
    char target_file_path[64] = {0};
    File upload_file;
    bool client_connected = false;
    bool new_book_ready = false;
    bool is_ota_mode = false;
    bool ota_reboot_pending = false;
    uint32_t ota_reboot_time = 0;

    // Queued operations for safe execution on the main loop task (avoids nimble_host stack overflow)
    volatile bool pending_list_books = false;
    volatile bool pending_delete_book = false;
    std::string pending_delete_path;
    volatile bool pending_rename_book = false;
    std::string pending_rename_old;
    std::string pending_rename_new;
    volatile bool pending_select_book = false;
    std::string pending_select_path;
    volatile bool pending_wifi_set = false;
    uint8_t pending_wifi_slot = 0;
    std::string pending_wifi_ssid;
    std::string pending_wifi_pass;
    volatile bool pending_ss_config = false;
    bool pending_ss_enabled = true;
    std::string pending_ss_url;
    volatile bool pending_progress_reset = false;
    char pending_progress_reset_path[64] = {0};

    // Appends `src` to `out` as JSON string content (escapes quotes/backslashes, drops control chars)
    static void json_escape_append(String& out, const char* src) {
        for (const char* p = src; *p; p++) {
            unsigned char c = (unsigned char)*p;
            if (c == '"' || c == '\\') { out += '\\'; out += (char)c; }
            else if (c >= 0x20) { out += (char)c; }
        }
    }

    void onConnect(NimBLEServer* pServer) override {
        client_connected = true;
        if (current_state == BLE_SYNC_ADVERTISING) {
            current_state = BLE_SYNC_CONNECTED;
        }
        Serial.println("[BLE] Client connected!");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        client_connected = false;
        Serial.println("[BLE] Client disconnected.");
        if (current_state == BLE_SYNC_RECEIVING) {
            // Unexpected disconnect during transfer
            abort_transfer();
            current_state = BLE_SYNC_FAILED;
        } else if (current_state == BLE_SYNC_OTA_RECEIVING) {
            if (Update.isRunning()) {
                Update.abort();
            }
            is_ota_mode = false;
            current_state = BLE_SYNC_FAILED;
        } else if (current_state != BLE_SYNC_SUCCESS) {
            current_state = BLE_SYNC_ADVERTISING;
            NimBLEDevice::startAdvertising();
        }
    }

    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        if (pCharacteristic == pCtrlChar) {
            std::string val = pCharacteristic->getValue();
            if (val.length() < 1) return;
            uint8_t cmd = (uint8_t)val[0];

            if (cmd == CMD_START_TRANSFER && val.length() >= 6) {
                // Byte 0: CMD_START_TRANSFER
                // Bytes 1..4: uint32_t total filesize (little-endian)
                total_expected_bytes = (uint8_t)val[1] | 
                                      ((uint8_t)val[2] << 8) | 
                                      ((uint8_t)val[3] << 16) | 
                                      ((uint8_t)val[4] << 24);

                // Bytes 5..N: null-terminated filename
                size_t name_len = val.length() - 5;
                if (name_len > sizeof(incoming_filename) - 1) {
                    name_len = sizeof(incoming_filename) - 1;
                }
                memcpy(incoming_filename, val.data() + 5, name_len);
                incoming_filename[name_len] = '\0';

                // Ensure path starts with /
                if (incoming_filename[0] != '/') {
                    snprintf(target_file_path, sizeof(target_file_path), "/%s", incoming_filename);
                } else {
                    strncpy(target_file_path, incoming_filename, sizeof(target_file_path) - 1);
                }

                Serial.printf("[BLE] Start Transfer -> File: '%s', Size: %u bytes\n", 
                              target_file_path, total_expected_bytes);

                // Close any existing open file before opening
                if (upload_file) {
                    upload_file.close();
                }

                // Open LittleFS file for writing
                upload_file = LittleFS.open(target_file_path, "w");
                if (!upload_file) {
                    Serial.println("[BLE] Failed to open LittleFS file for writing!");
                    send_status(STATUS_ERROR);
                    current_state = BLE_SYNC_FAILED;
                    return;
                }

                bytes_received = 0;
                current_state = BLE_SYNC_RECEIVING;
                send_status(STATUS_READY);

            } else if (cmd == CMD_FINISH_TRANSFER) {
                Serial.printf("[BLE] Finish Transfer command received. Total: %u / %u bytes\n",
                              bytes_received, total_expected_bytes);
                if (upload_file) {
                    upload_file.flush();
                    upload_file.close();
                }

                if (total_expected_bytes > 0 && bytes_received < total_expected_bytes) {
                    Serial.printf("[BLE] Error: Received fewer bytes than expected! (Got %u, expected %u, missing %u)\n",
                                  bytes_received, total_expected_bytes, total_expected_bytes - bytes_received);
                    send_status(STATUS_ERROR);
                    current_state = BLE_SYNC_FAILED;
                } else {
                    Serial.printf("[BLE] Book transfer completed successfully! (%u bytes saved to %s)\n",
                                  bytes_received, target_file_path);
                    send_status(STATUS_SUCCESS);
                    current_state = BLE_SYNC_SUCCESS;
                    new_book_ready = true;
                    // Same filename may carry stale bookmark offsets from a previous copy of the book
                    strncpy(pending_progress_reset_path, target_file_path, sizeof(pending_progress_reset_path) - 1);
                    pending_progress_reset_path[sizeof(pending_progress_reset_path) - 1] = '\0';
                    pending_progress_reset = true;
                }

            } else if (cmd == CMD_ABORT) {
                Serial.println("[BLE] Abort command received.");
                abort_transfer();
                send_status(STATUS_ABORTED);
                current_state = BLE_SYNC_FAILED;

            } else if (cmd == CMD_LIST_BOOKS) {
                start_time = millis(); // Refresh timeout
                Serial.println("[BLE] List books requested by web client (queued).");
                pending_list_books = true;

            } else if (cmd == CMD_DELETE_BOOK && val.length() >= 2) {
                start_time = millis(); // Refresh timeout
                std::string fname = val.substr(1);
                if (fname[0] != '/') fname = "/" + fname;
                pending_delete_path = fname;
                pending_delete_book = true;

            } else if (cmd == CMD_RENAME_BOOK && val.length() >= 4) {
                start_time = millis(); // Refresh timeout
                uint8_t old_len = (uint8_t)val[1];
                if (val.length() > 2 + old_len) {
                    std::string old_name = val.substr(2, old_len);
                    std::string new_name = val.substr(2 + old_len);
                    if (old_name[0] != '/') old_name = "/" + old_name;
                    if (new_name[0] != '/') new_name = "/" + new_name;
                    pending_rename_old = old_name;
                    pending_rename_new = new_name;
                    pending_rename_book = true;
                }

            } else if (cmd == CMD_SELECT_BOOK && val.length() >= 2) {
                start_time = millis(); // Refresh timeout
                std::string target = val.substr(1);
                if (target[0] != '/') target = "/" + target;
                pending_select_path = target;
                pending_select_book = true;

            } else if (cmd == CMD_WIFI_SET && val.length() >= 3) {
                start_time = millis(); // Refresh timeout
                uint8_t ssid_len = (uint8_t)val[2];
                if (val.length() >= (size_t)(3 + ssid_len) && ssid_len <= 32) {
                    pending_wifi_slot = (uint8_t)val[1];
                    pending_wifi_ssid = val.substr(3, ssid_len);
                    pending_wifi_pass = val.substr(3 + ssid_len);
                    pending_wifi_set = true;
                }

            } else if (cmd == CMD_SS_CONFIG && val.length() >= 2) {
                start_time = millis(); // Refresh timeout
                pending_ss_enabled = (val[1] != 0);
                pending_ss_url = val.substr(2);
                pending_ss_config = true;

            } else if (cmd == CMD_START_OTA && val.length() >= 5) {
                // Byte 0: CMD_START_OTA (0x10)
                // Bytes 1..4: uint32_t total binary size (little-endian)
                // Bytes 5..36 (optional): MD5 ASCII string (32 hex characters)
                start_time = millis(); // Refresh timeout
                total_expected_bytes = (uint8_t)val[1] | 
                                      ((uint8_t)val[2] << 8) | 
                                      ((uint8_t)val[3] << 16) | 
                                      ((uint8_t)val[4] << 24);

                Serial.printf("[BLE OTA] Start OTA -> Expected Size: %u bytes\n", total_expected_bytes);

                if (upload_file) {
                    upload_file.close();
                }
                if (Update.isRunning()) {
                    Update.abort();
                }

                if (total_expected_bytes == 0 || total_expected_bytes > 0x1C0000) {
                    Serial.printf("[BLE OTA] Invalid OTA binary size: %u (Max 1.75 MB)\n", total_expected_bytes);
                    send_status(STATUS_ERROR);
                    current_state = BLE_SYNC_FAILED;
                    return;
                }

                if (val.length() >= 5 + 32) {
                    std::string md5_hex = val.substr(5, 32);
                    Update.setMD5(md5_hex.c_str());
                    Serial.printf("[BLE OTA] MD5 verification set: %s\n", md5_hex.c_str());
                }

                if (!Update.begin(total_expected_bytes, U_FLASH)) {
                    Serial.printf("[BLE OTA] Update.begin failed! Error: %s\n", Update.errorString());
                    send_status(STATUS_ERROR);
                    current_state = BLE_SYNC_FAILED;
                    return;
                }

                bytes_received = 0;
                is_ota_mode = true;
                current_state = BLE_SYNC_OTA_RECEIVING;
                send_status(STATUS_READY);

            } else if (cmd == CMD_FINISH_OTA) {
                Serial.printf("[BLE OTA] Finish OTA received. Written: %u / %u bytes\n",
                              bytes_received, total_expected_bytes);
                start_time = millis();

                if (Update.end(true)) {
                    if (Update.isFinished()) {
                        Serial.printf("[BLE OTA] Firmware verified and written successfully! (%u bytes)\n", bytes_received);
                        send_status(STATUS_SUCCESS);
                        current_state = BLE_SYNC_SUCCESS;
                        ota_reboot_pending = true;
                        ota_reboot_time = millis() + 1500;
                    } else {
                        Serial.println("[BLE OTA] Update.end() returned true but update is not finished!");
                        send_status(STATUS_ERROR);
                        current_state = BLE_SYNC_FAILED;
                    }
                } else {
                    Serial.printf("[BLE OTA] Update.end() failed! Error: %s\n", Update.errorString());
                    send_status(STATUS_ERROR);
                    current_state = BLE_SYNC_FAILED;
                }

            } else if (cmd == CMD_ABORT_OTA) {
                Serial.println("[BLE OTA] Abort OTA requested.");
                if (Update.isRunning()) {
                    Update.abort();
                }
                is_ota_mode = false;
                send_status(STATUS_ABORTED);
                current_state = BLE_SYNC_FAILED;
            }

        } else if (pCharacteristic == pDataChar) {
            std::string val = pCharacteristic->getValue();
            if (val.length() > 0) {
                if (current_state == BLE_SYNC_OTA_RECEIVING) {
                    start_time = millis(); // Refresh timeout during OTA
                    size_t written = Update.write((uint8_t*)val.data(), val.length());
                    if (written != val.length()) {
                        Serial.printf("[BLE OTA] Write short! Expected %u, wrote %u\n", val.length(), written);
                    }
                    bytes_received += written;
                } else if (current_state == BLE_SYNC_RECEIVING && upload_file) {
                    start_time = millis(); // Refresh timeout during book transfer
                    size_t written = upload_file.write((const uint8_t*)val.data(), val.length());
                    if (written != val.length()) {
                        Serial.printf("[BLE] Disk write short! Expected %u, wrote %u\n", val.length(), written);
                    }
                    bytes_received += written;
                }
            }
        }
    }

    void send_status(uint8_t status) {
        if (pCtrlChar) {
            pCtrlChar->setValue(&status, 1);
            pCtrlChar->notify();
        }
    }

    void abort_transfer() {
        if (upload_file) {
            upload_file.close();
        }
        if (strlen(target_file_path) > 0 && LittleFS.exists(target_file_path)) {
            LittleFS.remove(target_file_path);
        }
    }

    void process_list_books() {
        Serial.println("[BLE] Processing list books request in main loop context...");
        std::unique_ptr<StoredBookInfo[]> books(new (std::nothrow) StoredBookInfo[MAX_STORED_BOOKS]);
        if (!books) {
            Serial.println("[BLE] Out of memory allocating book list!");
            return;
        }

        int count = StorageManager::get_stored_books(books.get(), MAX_STORED_BOOKS);

        String json = "{\"total\":";
        json += String((unsigned int)LittleFS.totalBytes());
        json += ",\"used\":";
        json += String((unsigned int)LittleFS.usedBytes());
        json += ",\"active\":\"";
        json += String(ConfigManager::get_current_book_path());
        json += "\",\"books\":[";

        for (int i = 0; i < count; i++) {
            if (i > 0) json += ",";
            int32_t saved_frame = ConfigManager::get_book_progress(books[i].filename);
            bool is_v2 = (books[i].format_version == EBK_FORMAT_V2_TEXT);
            int cur_unit = saved_frame + 1; // v1: page, v2: chapter

            json += "{\"file\":\"";
            json_escape_append(json, books[i].filename);
            json += "\",\"title\":\"";
            json_escape_append(json, books[i].title);
            json += "\",\"size\":";
            json += String((unsigned int)books[i].file_size);

            if (is_v2) {
                // Chapter-based progress: chapter N of M, and percent through that chapter
                uint16_t ch = 0;
                uint32_t off = 0;
                ConfigManager::get_book_progress(books[i].filename, &ch, &off);
                if (ch >= books[i].total_chapters) { ch = 0; off = 0; }
                int pct = 0;
                char ctitle[33] = {0};
                Ebk2ChapterEntry ce;
                if (StorageManager::read_v2_chapter_entry(books[i].filename, ch, &ce)) {
                    pct = StorageManager::v2_chapter_percent(ce, off);
                    memcpy(ctitle, ce.title, sizeof(ce.title));
                }
                cur_unit = ch + 1;
                json += ",\"fmt\":2,\"ch\":";
                json += String((unsigned int)(ch + 1));
                json += ",\"chs\":";
                json += String((unsigned int)books[i].total_chapters);
                json += ",\"pct\":";
                json += String(pct);
                json += ",\"ctitle\":\"";
                json_escape_append(json, ctitle);
                json += "\"";
            }

            json += ",\"cur\":";
            json += String(cur_unit);
            json += ",\"tot\":";
            json += String(books[i].total_frames);
            json += "}";
        }
        json += "]}";

        const size_t MAX_CHUNK = 400;
        size_t offset = 0;
        std::vector<uint8_t> packet(MAX_CHUNK + 2);

        while (offset < json.length()) {
            size_t len = json.length() - offset;
            if (len > MAX_CHUNK) len = MAX_CHUNK;
            uint8_t has_more = (offset + len < json.length()) ? 1 : 0;

            packet[0] = STATUS_BOOK_LIST;
            packet[1] = has_more;
            memcpy(packet.data() + 2, json.c_str() + offset, len);

            if (pCtrlChar) {
                pCtrlChar->setValue(packet.data(), len + 2);
                pCtrlChar->notify();
            }
            offset += len;
            if (has_more) delay(15);
        }
        Serial.printf("[BLE] Sent book list JSON (%u bytes) to client.\n", json.length());
    }

    void process_delete_book(const std::string& fname) {
        Serial.printf("[BLE] Delete book requested: '%s'\n", fname.c_str());
        if (LittleFS.exists(fname.c_str())) {
            LittleFS.remove(fname.c_str());
            Serial.println("[BLE] Book deleted successfully.");

            // If active book was deleted, switch to another book or sample
            if (strcmp(ConfigManager::get_current_book_path(), fname.c_str()) == 0) {
                std::unique_ptr<StoredBookInfo[]> remaining(new (std::nothrow) StoredBookInfo[MAX_STORED_BOOKS]);
                if (remaining) {
                    int count = StorageManager::get_stored_books(remaining.get(), MAX_STORED_BOOKS);
                    if (count > 0) {
                        ConfigManager::switch_book(remaining[0].filename);
                    } else {
                        ConfigManager::switch_book("/sample_book.ebk");
                    }
                }
            }
            send_status(STATUS_SUCCESS);
        } else {
            Serial.println("[BLE] Error: Book not found on storage.");
            send_status(STATUS_ERROR);
        }
    }

    void process_rename_book(const std::string& old_name, const std::string& new_name) {
        Serial.printf("[BLE] Rename book: '%s' -> '%s'\n", old_name.c_str(), new_name.c_str());
        if (LittleFS.exists(old_name.c_str())) {
            if (LittleFS.rename(old_name.c_str(), new_name.c_str())) {
                if (strcmp(ConfigManager::get_current_book_path(), old_name.c_str()) == 0) {
                    int32_t prog = ConfigManager::get_book_progress(old_name.c_str());
                    ConfigManager::set_book_progress(new_name.c_str(), prog);
                    ConfigManager::switch_book(new_name.c_str());
                }
                Serial.println("[BLE] Book renamed successfully.");
                send_status(STATUS_SUCCESS);
            } else {
                Serial.println("[BLE] Failed to rename book.");
                send_status(STATUS_ERROR);
            }
        } else {
            Serial.println("[BLE] Old book file not found.");
            send_status(STATUS_ERROR);
        }
    }

    void process_select_book(const std::string& target) {
        Serial.printf("[BLE] Select active book: '%s'\n", target.c_str());
        if (LittleFS.exists(target.c_str())) {
            ConfigManager::switch_book(target.c_str());
            new_book_ready = true;
            send_status(STATUS_SUCCESS);
        } else {
            send_status(STATUS_ERROR);
        }
    }

public:
    static BleSyncManager& get() {
        static BleSyncManager instance;
        return instance;
    }

    bool begin() {
        if (current_state != BLE_SYNC_INACTIVE) {
            stop();
        }

        Serial.println("\n[BLE] Initializing Bluetooth Low Energy (NimBLE)...");
        NimBLEDevice::init("ESP32-Ereader");
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);
        NimBLEDevice::setMTU(517);

        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(this);

        NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);

        pCtrlChar = pService->createCharacteristic(
            BLE_CHAR_CTRL_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
        );
        pCtrlChar->setCallbacks(this);

        pDataChar = pService->createCharacteristic(
            BLE_CHAR_DATA_UUID,
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
        );
        pDataChar->setCallbacks(this);

        pService->start();

        NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->start();

        current_state = BLE_SYNC_ADVERTISING;
        start_time = millis();
        bytes_received = 0;
        total_expected_bytes = 0;
        incoming_filename[0] = '\0';
        target_file_path[0] = '\0';
        new_book_ready = false;
        is_ota_mode = false;
        ota_reboot_pending = false;
        ota_reboot_time = 0;

        Serial.println("[BLE] Advertising started as 'ESP32-Ereader'. 60s timeout active.");
        return true;
    }

    void stop() {
        if (current_state == BLE_SYNC_INACTIVE) return;

        Serial.println("[BLE] Deinitializing BLE stack & shutting down 2.4 GHz radio...");
        if (upload_file) {
            upload_file.close();
        }
        if (Update.isRunning()) {
            Update.abort();
        }

        NimBLEDevice::deinit(true);
        pServer = nullptr;
        pCtrlChar = nullptr;
        pDataChar = nullptr;
        current_state = BLE_SYNC_INACTIVE;
        client_connected = false;
        is_ota_mode = false;
        Serial.println("[BLE] Radio successfully turned off.");
    }

    BleSyncState update() {
        if (current_state == BLE_SYNC_INACTIVE) {
            return BLE_SYNC_INACTIVE;
        }

        // Clear stale bookmark of a freshly uploaded book (same filename may have old offsets)
        if (pending_progress_reset) {
            pending_progress_reset = false;
            ConfigManager::set_book_progress(pending_progress_reset_path, 0, 0, 0);
        }

        // Handle queued BLE operations in the main loop context (safe stack size)
        if (pending_list_books) {
            pending_list_books = false;
            process_list_books();
        }
        if (pending_delete_book) {
            pending_delete_book = false;
            process_delete_book(pending_delete_path);
        }
        if (pending_rename_book) {
            pending_rename_book = false;
            process_rename_book(pending_rename_old, pending_rename_new);
        }
        if (pending_select_book) {
            pending_select_book = false;
            process_select_book(pending_select_path);
        }
        if (pending_wifi_set) {
            pending_wifi_set = false;
            ScreensaverConfig::set_wifi(pending_wifi_slot, String(pending_wifi_ssid.c_str()), String(pending_wifi_pass.c_str()));
            Serial.printf("[BLE] WiFi slot %u %s\n", pending_wifi_slot, pending_wifi_ssid.empty() ? "cleared" : "saved");
            send_status(STATUS_SUCCESS);
        }
        if (pending_ss_config) {
            pending_ss_config = false;
            ScreensaverConfig::set_enabled(pending_ss_enabled);
            ScreensaverConfig::set_url(String(pending_ss_url.c_str()));
            Serial.println("[BLE] Screensaver config saved.");
            send_status(STATUS_SUCCESS);
        }

        // Trigger reboot into newly flashed OTA partition
        if (ota_reboot_pending && millis() > ota_reboot_time) {
            Serial.println("[BLE OTA] Rebooting into new firmware now...");
            delay(100);
            esp_restart();
        }

        // When not actively receiving data, enforce the 60s timeout
        if (current_state == BLE_SYNC_ADVERTISING || current_state == BLE_SYNC_CONNECTED) {
            if (millis() - start_time >= (BLE_TIMEOUT_SECONDS * 1000UL)) {
                Serial.println("[BLE] 60-second connection timeout reached!");
                current_state = BLE_SYNC_TIMEOUT_EXPIRED;
            }
        }

        return current_state;
    }

    int get_remaining_seconds() const {
        if (current_state != BLE_SYNC_ADVERTISING && current_state != BLE_SYNC_CONNECTED) {
            return 0;
        }
        uint32_t elapsed = (millis() - start_time) / 1000;
        if (elapsed >= BLE_TIMEOUT_SECONDS) return 0;
        return BLE_TIMEOUT_SECONDS - elapsed;
    }

    BleSyncState get_state() const { return current_state; }
    uint32_t get_bytes_received() const { return bytes_received; }
    uint32_t get_total_expected_bytes() const { return total_expected_bytes; }
    const char* get_target_file_path() const { return target_file_path; }
    bool is_new_book_ready() const { return new_book_ready; }
    bool is_client_connected() const { return client_connected; }
    bool is_ota_in_progress() const { return is_ota_mode; }
};
