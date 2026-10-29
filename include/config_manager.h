#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <stdint.h>
#include <string.h>
#include "hardware_config.h"

#define MAX_BOOK_PATH_LEN 64
#define CONFIG_SCHEMA_VERSION 2

/**
 * In-memory mirror of persistent reader configuration.
 */
struct ReaderConfig {
    uint16_t schema_version;
    char     current_book_path[MAX_BOOK_PATH_LEN];
    int32_t  current_frame;
    uint8_t  rotation;            // 0: Landscape, 1: Portrait 90 deg CW
    uint8_t  refresh_interval;    // Full anti-ghosting refresh interval (pages, default 8)
    uint16_t auto_sleep_min;      // Inactivity timeout in minutes before Stage 2 sleep (default 5)
    uint32_t boot_count;          // Total boot/wake cycles (survives power loss)
    uint32_t total_page_turns;    // Cumulative lifetime page turns
    uint32_t last_bat_mv;         // Last measured battery millivolts
    
    // Future-proof expansion slots
    uint8_t  font_size_idx;       // 0: Default, 1: Medium, 2: Large
    uint8_t  fast_refresh_mode;   // 1: Enabled (fast 200ms B/W LUT), 0: Always full
    uint8_t  inverted_colors;     // 0: Normal (black on white), 1: Inverted (white on black)
    uint8_t  reserved[5];         // Reserved padding for future settings
};

class ConfigManager {
private:
    static Preferences prefs;
    static Preferences progress_prefs;
    static ReaderConfig config;
    static bool dirty;
    static uint32_t dirty_time;
    static const uint32_t DEBOUNCE_DELAY_MS = 1000; // 1 second settle time before flash commit

    /**
     * 32-bit FNV-1a hash to convert arbitrary length book paths/titles
     * into unique, collision-resistant 10-char NVS keys ("p_XXXXXXXX").
     */
    static uint32_t fnv1a_hash(const char* str) {
        uint32_t hash = 2166136261u;
        if (!str) return hash;
        while (*str) {
            hash ^= (uint8_t)*str++;
            hash *= 16777619u;
        }
        return hash;
    }

    static void make_progress_key(const char* book_path, char* key_buf, size_t buf_len) {
        uint32_t h = fnv1a_hash(book_path);
        snprintf(key_buf, buf_len, "p_%08x", (unsigned int)h);
    }

public:
    /**
     * Initializes the persistent storage system.
     * Restores all settings from NVS, handles schema migrations, and increments boot count.
     */
    static void begin() {
        dirty = false;
        dirty_time = 0;

        prefs.begin("ereader", false);
        uint16_t stored_ver = prefs.getUInt("cfg_ver", 0);

        if (stored_ver == 0) {
            // First boot on clean flash: initialize sensible defaults
            Serial.println("[NVS] No previous configuration found. Initializing factory defaults...");
            config.schema_version = CONFIG_SCHEMA_VERSION;
            strncpy(config.current_book_path, "/sample_book.ebk", sizeof(config.current_book_path) - 1);
            config.current_book_path[sizeof(config.current_book_path) - 1] = '\0';
            config.current_frame = 0;
            config.rotation = ORIENT_LANDSCAPE;
            config.refresh_interval = 8;
            config.auto_sleep_min = 5;
            config.boot_count = 1;
            config.total_page_turns = 0;
            config.last_bat_mv = 3800;
            config.font_size_idx = 0;
            config.fast_refresh_mode = 1;
            config.inverted_colors = 0;
            memset(config.reserved, 0, sizeof(config.reserved));

            // Write all default keys to NVS
            prefs.putUInt("cfg_ver", config.schema_version);
            prefs.putString("book_path", config.current_book_path);
            prefs.putInt("cur_frame", config.current_frame);
            prefs.putUChar("rotation", config.rotation);
            prefs.putUChar("refresh_inv", config.refresh_interval);
            prefs.putUInt("sleep_min", config.auto_sleep_min);
            prefs.putUInt("boot_count", config.boot_count);
            prefs.putUInt("page_turns", config.total_page_turns);
            prefs.putUInt("last_bat_mv", config.last_bat_mv);
            prefs.putUChar("font_idx", config.font_size_idx);
            prefs.putUChar("fast_mode", config.fast_refresh_mode);
            prefs.putUChar("invert_col", config.inverted_colors);
            prefs.putUChar("zen_mode", config.reserved[0]);
        } else {
            // Load persistent settings with default fallbacks for future-proof backward compatibility
            config.schema_version = stored_ver;
            String bpath = prefs.getString("book_path", "/sample_book.ebk");
            strncpy(config.current_book_path, bpath.c_str(), sizeof(config.current_book_path) - 1);
            config.current_book_path[sizeof(config.current_book_path) - 1] = '\0';
            config.current_frame = prefs.getInt("cur_frame", 0);
            config.rotation = prefs.getUChar("rotation", ORIENT_LANDSCAPE);
            config.refresh_interval = prefs.getUChar("refresh_inv", 8);
            config.auto_sleep_min = prefs.getUInt("sleep_min", 5);
            config.boot_count = prefs.getUInt("boot_count", 0) + 1;
            config.total_page_turns = prefs.getUInt("page_turns", 0);
            config.last_bat_mv = prefs.getUInt("last_bat_mv", 3800);
            config.font_size_idx = prefs.getUChar("font_idx", 0);
            config.fast_refresh_mode = prefs.getUChar("fast_mode", 1);
            config.inverted_colors = prefs.getUChar("invert_col", 0);
            memset(config.reserved, 0, sizeof(config.reserved));
            config.reserved[0] = prefs.getUChar("zen_mode", 0);

            // Save incremented boot count
            prefs.putUInt("boot_count", config.boot_count);

            // Future firmware migration hook
            if (stored_ver < CONFIG_SCHEMA_VERSION) {
                Serial.printf("[NVS] Migrating settings from Schema v%u to v%u...\n", stored_ver, CONFIG_SCHEMA_VERSION);
                if (stored_ver < 2) {
                    // v2: default inactivity sleep dropped from 60 to 5 minutes
                    config.auto_sleep_min = 5;
                    prefs.putUInt("sleep_min", config.auto_sleep_min);
                }
                config.schema_version = CONFIG_SCHEMA_VERSION;
                prefs.putUInt("cfg_ver", config.schema_version);
                Serial.println("[NVS] Migration complete!");
            }
        }
        prefs.end();

        // Print loaded state
        dump_state();
    }

    /**
     * Dumps all persistent settings to Serial.
     */
    static void dump_state() {
        Serial.println("\n[NVS] ================= Persistent Config Loaded =================");
        Serial.printf("[NVS] Schema Version  : %u\n", config.schema_version);
        Serial.printf("[NVS] Active Book     : %s\n", config.current_book_path);
        Serial.printf("[NVS] Page Position   : Frame %d\n", config.current_frame);
        Serial.printf("[NVS] Screen Rotation : %s (%d)\n",
                      (config.rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*", config.rotation);
        Serial.printf("[NVS] Zen Reading Mode: %s\n", (config.reserved[0] != 0) ? "ON (Hide Bars)" : "OFF");
        Serial.printf("[NVS] Refresh Rate    : Full refresh every %u pages\n", config.refresh_interval);
        Serial.printf("[NVS] Auto Sleep Time : %u minutes\n", config.auto_sleep_min);
        Serial.printf("[NVS] Total Boot Count: %u\n", config.boot_count);
        Serial.printf("[NVS] Total Page Turns: %u\n", config.total_page_turns);
        Serial.println("[NVS] ===========================================================");
    }

    // --- Active Book & Page Accessors ---

    static const char* get_current_book_path() {
        return config.current_book_path;
    }

    static void get_current_book_path(char* out_buf, size_t buf_len) {
        if (!out_buf || buf_len == 0) return;
        strncpy(out_buf, config.current_book_path, buf_len - 1);
        out_buf[buf_len - 1] = '\0';
    }

    static int32_t get_current_frame() {
        return config.current_frame;
    }

    static void set_current_frame(int32_t frame, bool immediate = false) {
        if (config.current_frame != frame) {
            config.current_frame = frame;
            config.total_page_turns++;
            dirty = true;
            dirty_time = millis();
        }
        if (immediate) {
            commit_immediate();
        }
    }

    // --- Screen Settings Accessors ---

    static uint8_t get_rotation() {
        return config.rotation;
    }

    static void set_rotation(uint8_t rot, bool immediate = true) {
        if (config.rotation != rot) {
            config.rotation = rot;
            dirty = true;
            dirty_time = millis();
        }
        if (immediate) {
            commit_immediate();
        }
    }

    static bool get_zen_mode() {
        return (config.reserved[0] != 0);
    }

    static void set_zen_mode(bool enable, bool immediate = true) {
        uint8_t val = enable ? 1 : 0;
        if (config.reserved[0] != val) {
            config.reserved[0] = val;
            dirty = true;
            dirty_time = millis();
        }
        if (immediate) {
            commit_immediate();
        }
    }

    static uint8_t get_refresh_interval() {
        return config.refresh_interval;
    }

    static void set_refresh_interval(uint8_t inv, bool immediate = false) {
        config.refresh_interval = inv;
        dirty = true;
        dirty_time = millis();
        if (immediate) commit_immediate();
    }

    static uint16_t get_auto_sleep_min() {
        return config.auto_sleep_min;
    }

    static void set_auto_sleep_min(uint16_t mins, bool immediate = false) {
        config.auto_sleep_min = mins;
        dirty = true;
        dirty_time = millis();
        if (immediate) commit_immediate();
    }

    static uint32_t get_boot_count() {
        return config.boot_count;
    }

    static uint32_t get_total_page_turns() {
        return config.total_page_turns;
    }

    static uint32_t get_last_battery_mv() {
        return config.last_bat_mv;
    }

    static void set_last_battery_mv(uint32_t mv, bool immediate = false) {
        config.last_bat_mv = mv;
        dirty = true;
        dirty_time = millis();
        if (immediate) commit_immediate();
    }

    // --- Per-Book Progress Tracking ---

    /**
     * Reads saved frame index for a specific book path, and optionally restores chapter and offset.
     * Returns 0 if book was never opened.
     */
    static int32_t get_book_progress(const char* book_path, uint16_t* out_chapter = nullptr, uint32_t* out_offset = nullptr) {
        if (!book_path || strlen(book_path) == 0) return 0;
        char key[16];
        make_progress_key(book_path, key, sizeof(key));

        progress_prefs.begin("progress", true); // read-only
        int32_t saved_frame = progress_prefs.getInt(key, 0);
        if (out_chapter) {
            char ch_key[16];
            snprintf(ch_key, sizeof(ch_key), "c_%s", key + 2);
            *out_chapter = progress_prefs.getUShort(ch_key, 0);
        }
        if (out_offset) {
            char off_key[16];
            snprintf(off_key, sizeof(off_key), "o_%s", key + 2);
            *out_offset = progress_prefs.getUInt(off_key, 0);
        }
        progress_prefs.end();
        return saved_frame;
    }

    /**
     * Saves reading progress for a specific book path (frame, chapter, and text offset).
     */
    static void set_book_progress(const char* book_path, int32_t frame, uint16_t chapter = 0, uint32_t offset = 0) {
        if (!book_path || strlen(book_path) == 0) return;
        char key[16];
        make_progress_key(book_path, key, sizeof(key));

        progress_prefs.begin("progress", false); // read-write
        progress_prefs.putInt(key, frame);
        char ch_key[16];
        snprintf(ch_key, sizeof(ch_key), "c_%s", key + 2);
        progress_prefs.putUShort(ch_key, chapter);

        char off_key[16];
        snprintf(off_key, sizeof(off_key), "o_%s", key + 2);
        progress_prefs.putUInt(off_key, offset);
        progress_prefs.end();
        Serial.printf("[NVS] Saved bookmark for '%s' -> Page %d, Ch %u, Offset %u (key: %s)\n",
                      book_path, frame + 1, chapter + 1, (unsigned int)offset, key);
    }

    /**
     * Switches the active book:
     * 1. Saves current reading progress for the outgoing book.
     * 2. Sets the new active book path.
     * 3. Restores saved reading progress for the incoming book (or 0).
     * 4. Immediately writes to flash NVS.
     */
    static int32_t switch_book(const char* new_book_path, uint16_t* out_chapter = nullptr, uint32_t* out_offset = nullptr) {
        if (!new_book_path || strlen(new_book_path) == 0) return 0;

        // Update active book
        strncpy(config.current_book_path, new_book_path, sizeof(config.current_book_path) - 1);
        config.current_book_path[sizeof(config.current_book_path) - 1] = '\0';

        // Retrieve saved bookmark for new book
        config.current_frame = get_book_progress(new_book_path, out_chapter, out_offset);
        
        dirty = true;
        commit_immediate();

        Serial.printf("[NVS] Switched active book to: '%s' (Resumed at Page %d)\n",
                      config.current_book_path, config.current_frame + 1);
        return config.current_frame;
    }

    // --- Commit & Synchronization ---

    /**
     * Checks if pending changes need to be written to NVS after settle debounce.
     * Call this inside loop().
     */
    static void commit_if_due() {
        if (dirty && (millis() - dirty_time >= DEBOUNCE_DELAY_MS)) {
            commit_immediate();
        }
    }

    /**
     * Immediately commits all pending configuration and progress to flash NVS.
     * Guaranteed safe across sudden deep sleep, power disconnect, or reboots.
     */
    static void commit_immediate() {
        if (!dirty) return;

        prefs.begin("ereader", false);
        prefs.putUInt("cfg_ver", config.schema_version);
        prefs.putString("book_path", config.current_book_path);
        prefs.putInt("cur_frame", config.current_frame);
        prefs.putUChar("rotation", config.rotation);
        prefs.putUChar("refresh_inv", config.refresh_interval);
        prefs.putUInt("sleep_min", config.auto_sleep_min);
        prefs.putUInt("boot_count", config.boot_count);
        prefs.putUInt("page_turns", config.total_page_turns);
        prefs.putUInt("last_bat_mv", config.last_bat_mv);
        prefs.putUChar("font_idx", config.font_size_idx);
        prefs.putUChar("fast_mode", config.fast_refresh_mode);
        prefs.putUChar("invert_col", config.inverted_colors);
        prefs.putUChar("zen_mode", config.reserved[0]);
        prefs.end();

        // Also update bookmark for current book in progress table
        if (strlen(config.current_book_path) > 0) {
            char key[16];
            make_progress_key(config.current_book_path, key, sizeof(key));
            progress_prefs.begin("progress", false);
            progress_prefs.putInt(key, config.current_frame);
            progress_prefs.end();
        }

        dirty = false;
        Serial.printf("[NVS] Flash committed: Book='%s', Page=%d, Rot=%s, Boots=%u, Turns=%u\n",
                      config.current_book_path, config.current_frame + 1,
                      (config.rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*",
                      config.boot_count, config.total_page_turns);
    }

    /**
     * Factory reset: Wipes all NVS preferences and restores initial defaults.
     */
    static void factory_reset() {
        Serial.println("[NVS] >>> Performing Factory Reset of Persistent Storage... <<<");
        prefs.begin("ereader", false);
        prefs.clear();
        prefs.end();

        progress_prefs.begin("progress", false);
        progress_prefs.clear();
        progress_prefs.end();

        begin();
        Serial.println("[NVS] Factory reset complete. Defaults restored.");
    }
};
