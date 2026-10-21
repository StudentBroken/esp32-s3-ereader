#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_BW.h>
#include "GxEPD2_420_Z96.h"
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <esp_sleep.h>
#include <driver/touch_pad.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "hardware_config.h"
#include "storage_manager.h"
#include "config_manager.h"
#include "ebk_format.h"
#include "ble_sync_manager.h"

// Capacitive Touch Pins & Channels
#define TOUCH_PREV_PIN    D4   // GPIO 5 (Touch Pad 5)
#define TOUCH_NEXT_PIN    D5   // GPIO 6 (Touch Pad 6)
#define TOUCH_MENU_PIN    D9   // GPIO 8 (Touch Pad 8)

#define TOUCH_PAD_PREV    TOUCH_PAD_NUM5
#define TOUCH_PAD_NEXT    TOUCH_PAD_NUM6
#define TOUCH_PAD_MENU    TOUCH_PAD_NUM8

// Asynchronous Button Events
enum ButtonEvent {
    EVENT_NONE = 0,
    EVENT_PREV,        // D4 tap: Prev / Up / Back
    EVENT_NEXT,        // D5 tap: Next / Down
    EVENT_MENU_TAP,    // D9 tap: Menu / Select
    EVENT_MENU_HOLD,   // D9 hold 2s: Power Confirm Dialog
    EVENT_TOGGLE_BARS  // D4 + D5 chord (simultaneous press & release): Toggle Zen / Hide Bars mode
};

QueueHandle_t inputQueue = NULL;
TaskHandle_t touchTaskHandle = NULL;
volatile bool touch_task_running = false;

#define TOUCH_DELTA_NAV         600
#define TOUCH_RELEASE_NAV       300
#define TOUCH_DELTA_MENU        1000
#define TOUCH_RELEASE_MENU      500
#define TOUCH_MAX_VALID_DELTA   6000  // Deltas > 6000 are electrical SPI crosstalk spikes, NOT human touch
#define LONG_PRESS_MENU_MS      2000  // 2 seconds hold on D9 opens Settings/Power Menu

// Adaptive Capacitive Touch Tracker
// - Locks onto ambient baseline, filters electrical noise, and triggers with hysteresis.
// - Proximity lock prevents baseline from adapting upwards when a finger approaches.
// - Fast downward snap ensures immediate responsiveness if release occurs after calibration drift.
// - Stuck-key watchdog auto-recovers baseline if held > 3.5s due to grip shift.
struct TouchTracker {
    uint8_t pin;
    float baseline;
    bool initialized;
    uint32_t last_raw;
    int32_t last_delta;
    bool is_touched;
    uint32_t touched_start_time;

    void init(uint8_t p) {
        pin = p;
        baseline = (pin == TOUCH_PREV_PIN) ? 26300.0f : 33500.0f;
        initialized = false;
        last_raw = (uint32_t)baseline;
        last_delta = 0;
        is_touched = false;
        touched_start_time = 0;
    }

    void calibrate(int samples = 30) {
        delay(30);
        uint32_t sum = 0;
        int valid = 0;
        for (int i = 0; i < samples; i++) {
            uint32_t r = touchRead(pin);
            if (r >= 1000 && r <= 60000) {
                sum += r;
                valid++;
            }
            delay(5);
        }
        if (valid > 0) {
            baseline = (float)(sum / valid);
            initialized = true;
        } else {
            baseline = (pin == TOUCH_PREV_PIN) ? 26300.0f : 33500.0f;
            initialized = true;
        }
        last_raw = (uint32_t)baseline;
        last_delta = 0;
        is_touched = false;
        touched_start_time = 0;
    }

    void flush() {
        uint32_t raw = touchRead(pin);
        if (raw >= 1000 && raw <= 60000) {
            last_raw = raw;
            last_delta = (int32_t)raw - (int32_t)baseline;
            if (raw < (uint32_t)(baseline - 40.0f)) {
                baseline = (float)raw;
                last_delta = 0;
            }
        }
        is_touched = false;
        touched_start_time = 0;
    }

    bool update(int32_t trigger_delta, int32_t release_delta, bool is_busy = false) {
        uint32_t raw = touchRead(pin);

        // Sanity check: valid capacitive reading on ESP32-S3 is 1,000 to 60,000.
        // Values > 60,000 or < 1,000 are uninitialized driver artifacts or electrical noise.
        if (raw < 1000 || raw > 60000) {
            return is_touched;
        }

        last_raw = raw;

        if (!initialized) {
            baseline = (float)raw;
            initialized = true;
            last_delta = 0;
            is_touched = false;
            touched_start_time = 0;
            return false;
        }

        last_delta = (int32_t)raw - (int32_t)baseline;

        // Discard electrical SPI crosstalk spikes (> 6000) without altering touch state
        if (last_delta > TOUCH_MAX_VALID_DELTA) {
            return is_touched;
        }

        // Hysteresis touch detection:
        if (!is_touched) {
            if (last_delta >= trigger_delta) {
                is_touched = true;
                touched_start_time = millis();
            }
        } else {
            if (last_delta < release_delta) {
                is_touched = false;
                touched_start_time = 0;
            }
        }

        // Bidirectional baseline tracking + Stuck-Key Protection:
        if (!is_busy) {
            if (!is_touched) {
                touched_start_time = 0;
                if (last_delta < 0) {
                    // Raw is below baseline: snap downward quickly
                    baseline += ((float)raw - baseline) * 0.10f;
                } else if (last_delta < trigger_delta) {
                    // Raw is resting above baseline: adapt upward smoothly to track thermal/grip drift
                    baseline += ((float)raw - baseline) * 0.02f;
                }
            } else {
                // If button is held/stuck continuously for > 3500ms (longer than 2s long press)
                // It means ambient baseline shifted while held, or baseline calibrated too low!
                if (touched_start_time == 0) {
                    touched_start_time = millis();
                } else if (millis() - touched_start_time >= 3500) {
                    baseline = (float)raw;
                    is_touched = false;
                    touched_start_time = 0;
                    last_delta = 0;
                    Serial.printf("\n[TOUCH] Pad D%d auto-recalibrated baseline to %.0f (stuck-key timeout)\n",
                                  (pin == TOUCH_PREV_PIN ? 4 : (pin == TOUCH_NEXT_PIN ? 5 : 9)), baseline);
                }
            }
        }

        return is_touched;
    }
};

TouchTracker tracker_prev;
TouchTracker tracker_next;
TouchTracker tracker_menu;

// Completely tears down and re-initializes ESP32-S3 touch hardware and FSM
void touch_hardware_reinit() {
    touch_pad_deinit();
    vTaskDelay(pdMS_TO_TICKS(20));
    touch_pad_sleep_channel_enable(TOUCH_PAD_MENU, false);
    touch_pad_sleep_channel_enable(TOUCH_PAD_PREV, false);
    touch_pad_sleep_channel_enable(TOUCH_PAD_NEXT, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    touchRead(TOUCH_PREV_PIN);
    touchRead(TOUCH_NEXT_PIN);
    touchRead(TOUCH_MENU_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Dedicated Touch Scanner Task running continuously on Core 0 at ~66Hz
// Never blocks on e-ink SPI or BUSY pin refreshes!
void touch_task(void* pvParameters) {
    bool d4_is_held = false;
    uint32_t d4_press_start = 0;
    bool d5_is_held = false;
    uint32_t d5_press_start = 0;
    bool both_held = false;
    bool chord_fired = false;

    bool d9_is_held = false;
    uint32_t d9_press_start = 0;
    bool d9_long_handled = false;
    int fault_counter = 0;
    uint32_t last_busy_end = 0;

    while (touch_task_running) {
        // Skip sampling during high-speed SPI bus data bursts (avoids hardware pin crosstalk)
        if (epd_spi_active) {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        bool is_busy = (digitalRead(EPD_BUSY_PIN) == HIGH);
        if (is_busy) {
            last_busy_end = millis();
            d4_press_start = 0;
            d4_is_held = false;
            d5_press_start = 0;
            d5_is_held = false;
            both_held = false;
            chord_fired = false;
            d9_press_start = 0;
            d9_is_held = false;
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        // Post-refresh blanking window (350ms):
        // The e-ink +/-15V charge pumps discharge into the ground plane for ~300ms after BUSY drops.
        // We completely blank touch processing during this cooldown to eliminate electrical ghost touches.
        if (millis() - last_busy_end < 350) {
            d4_press_start = 0;
            d4_is_held = false;
            d5_press_start = 0;
            d5_is_held = false;
            both_held = false;
            chord_fired = false;
            d9_press_start = 0;
            d9_is_held = false;
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        // Peripheral health check (only evaluated when display is idle and settled)
        uint32_t r_d4 = tracker_prev.last_raw;
        uint32_t r_d5 = tracker_next.last_raw;
        uint32_t r_d9 = tracker_menu.last_raw;

        if ((r_d4 < 1000 || r_d4 > 60000) ||
            (r_d5 < 1000 || r_d5 > 60000) ||
            (r_d9 < 1000 || r_d9 > 60000)) {
            fault_counter++;
            if (fault_counter > 20) { // 20 * 15ms = 300ms continuous lockup
                touch_hardware_reinit();
                tracker_prev.calibrate(20);
                tracker_next.calibrate(20);
                tracker_menu.calibrate(20);
                fault_counter = 0;
            }
        } else {
            fault_counter = 0;
        }

        bool prev_touched = tracker_prev.update(TOUCH_DELTA_NAV, TOUCH_RELEASE_NAV, false);
        bool next_touched = tracker_next.update(TOUCH_DELTA_NAV, TOUCH_RELEASE_NAV, false);
        bool menu_touched = tracker_menu.update(TOUCH_DELTA_MENU, TOUCH_RELEASE_MENU, false);

        // Common-Mode Noise Filter:
        // A human finger only touches ONE pad at a time, UNLESS pressing Prev + Next together.
        // If Menu triggers with any other pad (or all 3 pads trigger), it is common-mode electrical noise / full board grip.
        if (menu_touched && (prev_touched || next_touched)) {
            prev_touched = false;
            next_touched = false;
            menu_touched = false;
            d4_press_start = 0;
            d4_is_held = false;
            d5_press_start = 0;
            d5_is_held = false;
            d9_press_start = 0;
            d9_is_held = false;
            both_held = false;
            chord_fired = false;
        }

        // 1. D4 (PREV) Tracking
        if (prev_touched) {
            if (!d4_is_held) {
                d4_is_held = true;
                d4_press_start = millis();
            }
        }

        // 2. D5 (NEXT) Tracking
        if (next_touched) {
            if (!d5_is_held) {
                d5_is_held = true;
                d5_press_start = millis();
            }
        }

        // 3. Simultaneous chord: Both D4 and D5 held at the same time
        if (d4_is_held && d5_is_held) {
            both_held = true;
        }

        // 4. D4 Release
        if (!prev_touched && d4_is_held) {
            uint32_t dur = millis() - d4_press_start;
            d4_is_held = false;
            if (both_held) {
                if (!chord_fired) {
                    chord_fired = true;
                    if (inputQueue != NULL) {
                        ButtonEvent evt = EVENT_TOGGLE_BARS;
                        xQueueSend(inputQueue, &evt, 0);
                    }
                }
            } else if (dur >= 40) {
                if (inputQueue != NULL) {
                    ButtonEvent evt = EVENT_PREV;
                    xQueueSend(inputQueue, &evt, 0);
                }
            }
        }

        // 5. D5 Release
        if (!next_touched && d5_is_held) {
            uint32_t dur = millis() - d5_press_start;
            d5_is_held = false;
            if (both_held) {
                if (!chord_fired) {
                    chord_fired = true;
                    if (inputQueue != NULL) {
                        ButtonEvent evt = EVENT_TOGGLE_BARS;
                        xQueueSend(inputQueue, &evt, 0);
                    }
                }
            } else if (dur >= 40) {
                if (inputQueue != NULL) {
                    ButtonEvent evt = EVENT_NEXT;
                    xQueueSend(inputQueue, &evt, 0);
                }
            }
        }

        // 6. When both fingers are completely released, reset chord state for next gesture
        if (!prev_touched && !next_touched) {
            both_held = false;
            chord_fired = false;
        }

        // Debounced D9 (MENU TAP / HOLD)
        if (menu_touched) {
            if (!d9_is_held) {
                d9_is_held = true;
                d9_press_start = millis();
                d9_long_handled = false;
            } else if (!d9_long_handled && (millis() - d9_press_start >= LONG_PRESS_MENU_MS)) {
                d9_long_handled = true;
                if (inputQueue != NULL) {
                    ButtonEvent evt = EVENT_MENU_HOLD;
                    xQueueSend(inputQueue, &evt, 0);
                }
            }
        } else {
            if (d9_is_held) {
                uint32_t dur = millis() - d9_press_start;
                d9_is_held = false;
                if (!d9_long_handled && dur >= 40) {
                    if (inputQueue != NULL) {
                        ButtonEvent evt = EVENT_MENU_TAP;
                        xQueueSend(inputQueue, &evt, 0);
                    }
                }
                d9_long_handled = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    vTaskDelete(NULL);
}

// Inactivity Timeouts: 1 hour awake reading time before auto-entering Stage 2 (Zzz Screensaver)
#define AWAKE_TIMEOUT_MS  (60ULL * 60 * 1000ULL)
#define MENU_TIMEOUT_MS   15000  // 15 seconds of inactivity auto-closes menus

// Sleep Stages
enum SleepStage {
    STAGE_READING = 1,  // Stage 1: Reading standby. Current page visible.
    STAGE_DEEP_ZZZ = 2  // Stage 2: Deeper sleep. Randomized "Zzz" screensaver. ONLY Power (D9) wakes.
};

// UI Interaction State Machine
enum UiState {
    UI_STATE_READING = 0,
    UI_STATE_MENU,
    UI_STATE_LIBRARY,
    UI_STATE_JUMP,
    UI_STATE_POWER_CONFIRM,
    UI_STATE_BLE_SYNC
};

// Menu Item Identifiers
enum MenuItem {
    MENU_RESUME = 0,
    MENU_LIBRARY,
    MENU_BLE_SYNC,
    MENU_ORIENTATION,
    MENU_JUMP,
    MENU_CLEAN_REFRESH,
    MENU_SLEEP,
    MENU_COUNT
};

// Sleep state across Deep Sleep in RTC SRAM
RTC_DATA_ATTR SleepStage current_stage = STAGE_READING;

// Canonical Reader State (backed by NVS flash and synced to ConfigManager)
uint8_t current_rotation = ORIENT_LANDSCAPE;
int current_frame = 0;
int total_frames = 4;
int boot_count = 0;
uint32_t last_bat_mv = 3800;
char current_book_path[MAX_BOOK_PATH_LEN] = "/sample_book.ebk";
bool zen_mode = false; // When true, hides top and bottom bars (pure text + page number)

// Dynamic v2 Text Navigation & State
#define NAV_STACK_SIZE 64
uint32_t page_offset_stack[NAV_STACK_SIZE];
int page_stack_depth = 0;
uint32_t current_text_file_offset = 0;
uint32_t next_text_file_offset = 0;
uint16_t current_chapter_idx = 0;

// Runtime UI State
UiState current_ui_state = UI_STATE_READING;
int selected_menu_idx = 0;
int jump_target_frame = 0;
uint32_t menu_active_timer = 0;

// Library Browser State
StoredBookInfo cached_books[MAX_STORED_BOOKS];
int selected_library_idx = 0;

// Fast EPD Driver for HINK-E042A13 / Z96 (SSD1619) with custom 200ms partial LUT & 1.7s full LUT
SPIClass epdSPI(FSPI);
GxEPD2_BW<GxEPD2_420_Z96, GxEPD2_420_Z96::HEIGHT> display(
    GxEPD2_420_Z96(EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN)
);

// High-definition Unicode UTF-8 font renderer for multilingual reading (French accents, quotes, etc.)
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

// Frame buffer (15,000 bytes for 400x300 1-bit monochrome canvas)
uint8_t frame_buffer[EBK_FRAME_BUFFER_SIZE];

/**
 * Clean battery gauge: 22x11px bordered shell + fill bar + percentage text.
 * Red fill exclusively when battery is critical (<= 15%).
 */
void draw_battery_indicator(int16_t x, int16_t y, uint8_t pct, bool dark_mode = true) {
    uint16_t border = dark_mode ? GxEPD_WHITE : GxEPD_BLACK;
    uint16_t fill   = dark_mode ? GxEPD_WHITE : GxEPD_BLACK;

    display.drawRect(x, y, 22, 11, border);
    display.fillRect(x + 22, y + 3, 2, 5, border);

    int fill_w = (pct * 18) / 100;
    if (fill_w > 0) {
        display.fillRect(x + 2, y + 2, fill_w, 7, fill);
    }

    display.setFont(NULL);
    display.setTextColor(border);
    display.setCursor(x - 30, y + 2);
    display.printf("%u%%", pct);
}

/**
 * Word-wrapped paragraph renderer with automatic boundary checking.
 * Adapts to any width dynamically across Landscape (400px) and Portrait (300px).
 */
void draw_formatted_paragraph(const char* text, int start_x, int start_y, int max_w, int line_h) {
    char word[128];
    char line[256] = "";
    int cur_y = start_y;
    const char* p = text;

    u8g2Fonts.setFont(u8g2_font_helvR12_tf);
    u8g2Fonts.setFontMode(1);
    u8g2Fonts.setForegroundColor(GxEPD_BLACK);
    u8g2Fonts.setBackgroundColor(GxEPD_WHITE);

    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        int wlen = 0;
        while (*p && *p != ' ' && *p != '\n' && wlen < 120) {
            word[wlen++] = *p++;
        }
        word[wlen] = '\0';

        char test_line[256];
        if (line[0] == '\0') {
            strncpy(test_line, word, sizeof(test_line) - 1);
            test_line[sizeof(test_line) - 1] = '\0';
        } else {
            snprintf(test_line, sizeof(test_line), "%s %s", line, word);
        }

        int lw = u8g2Fonts.getUTF8Width(test_line);
        if (lw > max_w && line[0] != '\0') {
            u8g2Fonts.setCursor(start_x, cur_y);
            u8g2Fonts.print(line);
            cur_y += line_h;
            strncpy(line, word, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
        } else {
            strncpy(line, test_line, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
        }

        if (*p == '\n') {
            u8g2Fonts.setCursor(start_x, cur_y);
            u8g2Fonts.print(line);
            cur_y += line_h + 6;
            line[0] = '\0';
            p++;
        }
    }
    if (line[0] != '\0') {
        u8g2Fonts.setCursor(start_x, cur_y);
        u8g2Fonts.print(line);
    }
}

/**
 * Renders the built-in demo book pages dynamically in either Landscape or Portrait.
 */
void render_fallback_page(int page_idx, uint32_t bat_mv, uint8_t bat_pct) {
    int w = display.width();
    int h = display.height();

    display.fillScreen(GxEPD_WHITE);

    display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

    // Header Bar
    if (!zen_mode) {
        display.fillRect(4, 4, w - 8, 30, GxEPD_BLACK);
        u8g2Fonts.setFont(u8g2_font_helvB10_tf);
        u8g2Fonts.setFontMode(1);
        u8g2Fonts.setForegroundColor(GxEPD_WHITE);
        u8g2Fonts.setBackgroundColor(GxEPD_BLACK);
        u8g2Fonts.setCursor(16, 22);
        u8g2Fonts.print("THE TIME MACHINE");

        draw_battery_indicator(w - 36, 15, bat_pct, true);
    }

    // Reading Body
    int start_y = zen_mode ? 28 : 66;
    int margin_x = (w > 350) ? 20 : 14;
    int content_w = w - (margin_x * 2);

    u8g2Fonts.setFont(u8g2_font_helvB12_tf);
    u8g2Fonts.setFontMode(1);
    u8g2Fonts.setForegroundColor(GxEPD_BLACK);
    u8g2Fonts.setBackgroundColor(GxEPD_WHITE);

    if (page_idx == 0) {
        u8g2Fonts.setCursor(margin_x, start_y);
        u8g2Fonts.print("Chapter 1: The Time Traveller");
        draw_formatted_paragraph(
            "The Time Traveller was expounding a recondite matter to us. "
            "His grey eyes shone and twinkled, and his usually pale face was flushed and animated.\n"
            "The fire burned brightly, and the soft radiance of the incandescent lights caught the "
            "bubbles that passed in our glasses.",
            margin_x, start_y + 26, content_w, 24);
    } else if (page_idx == 1) {
        u8g2Fonts.setCursor(margin_x, start_y);
        u8g2Fonts.print("Chapter 1 (Continued)");
        draw_formatted_paragraph(
            "Our chairs, being his patents, embraced and caressed us rather than submitted to be sat upon, "
            "and there was that luxurious after-dinner atmosphere when thought roams gracefully free of precision.\n"
            "\"You must follow me carefully,\" he said.",
            margin_x, start_y + 26, content_w, 24);
    } else {
        u8g2Fonts.setCursor(margin_x, start_y);
        u8g2Fonts.print("Chapter 1: The 4th Dimension");
        draw_formatted_paragraph(
            "\"I shall have to controvert one or two ideas that are almost universally accepted. "
            "The geometry, for instance, they taught you at school is founded on a misconception.\"",
            margin_x, start_y + 26, content_w, 24);
    }

    // Footer Bar or Zen Page Number
    if (!zen_mode) {
        display.fillRect(6, h - 32, w - 12, 26, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(NULL);

        if (w > 350) {
            display.setCursor(16, h - 23);
            display.print("Prev");
            display.setCursor(w / 2 - 45, h - 23);
            display.printf("Page %d of %d", page_idx + 1, total_frames);
            display.setCursor(w / 2 + 45, h - 23);
            display.print("Menu");
            display.setCursor(w - 48, h - 23);
            display.print("Next");
        } else {
            display.setCursor(14, h - 23);
            display.print("Prev");
            display.setCursor(w / 2 - 38, h - 23);
            display.printf("P.%d/%d", page_idx + 1, total_frames);
            display.setCursor(w / 2 + 18, h - 23);
            display.print("Menu");
            display.setCursor(w - 48, h - 23);
            display.print("Next");
        }
    } else {
        // Zen Mode: discreet page number at bottom, no black bar!
        display.setFont(NULL);
        display.setTextColor(GxEPD_BLACK);
        char pbuf[32];
        snprintf(pbuf, sizeof(pbuf), "Page %d of %d", page_idx + 1, total_frames);
        display.setCursor((w - (strlen(pbuf) * 6)) / 2, h - 14);
        display.print(pbuf);
    }
}

/**
 * Dynamic on-device text renderer for EBK v2 structured text books.
 * Measures, word-wraps, and paginates raw UTF-8 text on the fly.
 * Automatically respects current_rotation (Landscape 400x300, Portrait 300x400).
 */
bool render_v2_text_page(const char* book_path, uint16_t chap_idx, uint32_t start_file_offset,
                         uint32_t* out_next_offset, uint8_t bat_pct) {
    if (!book_path || !LittleFS.exists(book_path)) return false;

    Ebk2Header h2;
    if (!StorageManager::get_v2_header(book_path, &h2)) return false;

    Ebk2ChapterEntry chap;
    if (!StorageManager::read_v2_chapter_entry(book_path, chap_idx, &chap)) return false;

    int w = display.width();
    int h = display.height();

    display.fillScreen(GxEPD_WHITE);
    display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

    // 1. Header Bar
    if (!zen_mode) {
        display.fillRect(4, 4, w - 8, 30, GxEPD_BLACK);
        u8g2Fonts.setFont(u8g2_font_helvB10_tf);
        u8g2Fonts.setFontMode(1);
        u8g2Fonts.setForegroundColor(GxEPD_WHITE);
        u8g2Fonts.setBackgroundColor(GxEPD_BLACK);
        u8g2Fonts.setCursor(16, 22);

        // Display Chapter Title or Book Title (preserves UTF-8 accents)
        char header_title[64];
        if (chap.title[0] != '\0') {
            strncpy(header_title, chap.title, sizeof(header_title) - 1);
        } else {
            strncpy(header_title, h2.title, sizeof(header_title) - 1);
        }
        header_title[sizeof(header_title) - 1] = '\0';
        u8g2Fonts.print(header_title);

        draw_battery_indicator(w - 36, 15, bat_pct, true);
    }

    // 2. Body Text Stream (Full UTF-8 multilingual rendering)
    u8g2Fonts.setFont(u8g2_font_helvR12_tf);
    u8g2Fonts.setFontMode(1);
    u8g2Fonts.setForegroundColor(GxEPD_BLACK);
    u8g2Fonts.setBackgroundColor(GxEPD_WHITE);

    int margin_x = (w > 350) ? 20 : 14;
    int content_w = w - (margin_x * 2);
    int line_h = 24;
    int cur_y = zen_mode ? 28 : 64;
    int max_y = zen_mode ? (h - 22) : (h - 36);

    // Read up to 2048 bytes of chapter text starting at start_file_offset
    char text_buf[2048];
    uint32_t read_start = (start_file_offset >= chap.offset) ? start_file_offset : chap.offset;
    uint32_t chap_end = chap.offset + chap.length;

    if (read_start >= chap_end) {
        u8g2Fonts.setCursor(margin_x, cur_y);
        u8g2Fonts.print("[End of Chapter]");
        if (out_next_offset) *out_next_offset = chap_end;
        return true;
    }

    size_t max_to_read = min((size_t)(sizeof(text_buf) - 1), (size_t)(chap_end - read_start));
    size_t bytes_read = StorageManager::read_v2_text_chunk(book_path, read_start, text_buf, max_to_read + 1);
    text_buf[bytes_read] = '\0';

    // Guard against splitting a multi-byte UTF-8 character at the buffer boundary
    if (bytes_read > 0) {
        size_t b = bytes_read;
        while (b > 0 && (text_buf[b - 1] & 0xC0) == 0x80) {
            b--;
        }
        if (b > 0 && (text_buf[b - 1] & 0x80)) {
            uint8_t lead = (uint8_t)text_buf[b - 1];
            int expected = 1;
            if ((lead & 0xE0) == 0xC0) expected = 2;
            else if ((lead & 0xF0) == 0xE0) expected = 3;
            else if ((lead & 0xF8) == 0xF0) expected = 4;
            if (bytes_read - (b - 1) < (size_t)expected) {
                bytes_read = b - 1;
                text_buf[bytes_read] = '\0';
            }
        }
    }

    const char* p = text_buf;
    char word[128];
    char line[256] = "";
    int consumed_bytes = 0;
    bool page_full = false;

    while (*p && !page_full) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (!*p) break;

        if (*p == '\n') {
            if (line[0] != '\0') {
                u8g2Fonts.setCursor(margin_x, cur_y);
                u8g2Fonts.print(line);
                cur_y += line_h;
                line[0] = '\0';
            }
            cur_y += 6;
            p++;
            while (*p == '\n') p++;

            if (cur_y + line_h > max_y) {
                consumed_bytes = (p - text_buf);
                page_full = true;
                break;
            }
            continue;
        }

        int wlen = 0;
        const char* word_start = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\n' && wlen < 120) {
            word[wlen++] = *p++;
        }
        word[wlen] = '\0';

        char test_line[256];
        if (line[0] == '\0') {
            strncpy(test_line, word, sizeof(test_line) - 1);
            test_line[sizeof(test_line) - 1] = '\0';
        } else {
            snprintf(test_line, sizeof(test_line), "%s %s", line, word);
        }

        int lw = u8g2Fonts.getUTF8Width(test_line);

        if (lw > content_w && line[0] != '\0') {
            if (cur_y + (line_h * 2) > max_y) {
                u8g2Fonts.setCursor(margin_x, cur_y);
                u8g2Fonts.print(line);
                line[0] = '\0';
                consumed_bytes = (word_start - text_buf);
                page_full = true;
                break;
            }
            u8g2Fonts.setCursor(margin_x, cur_y);
            u8g2Fonts.print(line);
            cur_y += line_h;
            strncpy(line, word, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
        } else {
            strncpy(line, test_line, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
        }
    }

    if (!page_full && line[0] != '\0') {
        u8g2Fonts.setCursor(margin_x, cur_y);
        u8g2Fonts.print(line);
        consumed_bytes = (p - text_buf);
    }

    if (out_next_offset) {
        *out_next_offset = read_start + consumed_bytes;
    }

    // 3. Footer Bar or Zen Page Number
    if (!zen_mode) {
        display.fillRect(6, h - 30, w - 12, 24, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(NULL);

        display.setCursor(16, h - 21);
        display.print("Prev");

        display.setCursor(w / 2 - 40, h - 21);
        display.printf("Ch.%d/%d", chap_idx + 1, h2.total_chapters);

        display.setCursor(w - 56, h - 21);
        display.print("Next");
    } else {
        // Zen Mode: discreet chapter indicator at bottom, no black bar!
        display.setFont(NULL);
        display.setTextColor(GxEPD_BLACK);
        char pbuf[32];
        snprintf(pbuf, sizeof(pbuf), "Ch.%d/%d", chap_idx + 1, h2.total_chapters);
        display.setCursor((w - (strlen(pbuf) * 6)) / 2, h - 14);
        display.print(pbuf);
    }

    return true;
}

/**
 * 1-bit bitmap renderer with support for 90° CW rotation.
 */
void render_ebk_bitmap(const uint8_t* src, uint8_t rotation) {
    if (rotation == ORIENT_LANDSCAPE) {
        display.drawBitmap(0, 0, src, EBK_FRAME_WIDTH, EBK_FRAME_HEIGHT, GxEPD_WHITE, GxEPD_BLACK);
    } else if (rotation == ORIENT_PORTRAIT) {
        for (int old_y = 0; old_y < EBK_FRAME_HEIGHT; old_y++) {
            for (int old_x = 0; old_x < EBK_FRAME_WIDTH; old_x++) {
                int byte_idx = (old_y * EBK_FRAME_WIDTH + old_x) / 8;
                int bit_idx = 7 - (old_x % 8);
                bool is_white = (src[byte_idx] >> bit_idx) & 1;
                int new_x = old_y;
                int new_y = (EBK_FRAME_WIDTH - 1) - old_x;
                display.drawPixel(new_x, new_y, is_white ? GxEPD_WHITE : GxEPD_BLACK);
            }
        }
    }
}

/**
 * Full page reader refresh. Supports fast 1.0s B/W differential mode and periodic full refresh.
 */
void render_current_reading_page(uint32_t bat_mv, uint8_t bat_pct, bool fast_bw = true) {
    static int fast_turn_count = 0;
    bool do_fast = fast_bw && (fast_turn_count < 8);
    if (do_fast) {
        fast_turn_count++;
    } else {
        fast_turn_count = 0;
    }

    EbkFormatVersion ver = StorageManager::get_book_version(current_book_path);

    Serial.printf("\n[EPD] Rendering Page %d (%s Refresh, Orient: %s, Format: %s)...\n",
                  current_frame + 1,
                  do_fast ? "FAST B/W ~1.0s" : "FULL CLEAN ANTI-GHOST",
                  (current_rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*",
                  (ver == EBK_FORMAT_V2_TEXT) ? "v2 Text" : (ver == EBK_FORMAT_V1_BITMAP ? "v1 Bitmap" : "Demo"));
    uint32_t t0 = millis();

    bool has_v1_frame = false;
    EbkFrameHeader frame_hdr;
    if (ver == EBK_FORMAT_V1_BITMAP) {
        has_v1_frame = StorageManager::read_frame(current_book_path, current_frame, &frame_hdr, frame_buffer, sizeof(frame_buffer));
    }

    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    auto render_page_content = [&]() {
        if (ver == EBK_FORMAT_V2_TEXT) {
            render_v2_text_page(current_book_path, current_chapter_idx, current_text_file_offset, &next_text_file_offset, bat_pct);
        } else if (ver == EBK_FORMAT_V1_BITMAP && has_v1_frame && current_rotation == ORIENT_LANDSCAPE) {
            render_ebk_bitmap(frame_buffer, current_rotation);
            draw_battery_indicator(w - 36, 12, bat_pct, false);
        } else {
            render_fallback_page(current_frame, bat_mv, bat_pct);
        }
    };

    if (do_fast) {
        display.setPartialWindow(0, 0, w, h);
        display.firstPage();
        do {
            render_page_content();
        } while (display.nextPage());
    } else {
        display.setFullWindow();
        display.firstPage();
        do {
            render_page_content();
        } while (display.nextPage());
    }

    Serial.printf("[EPD] Page render completed in %lu ms!\n", millis() - t0);
}

/**
 * Settings & Quick Menu Modal Screen.
 */
void render_menu_screen(uint8_t bat_pct) {
    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    int card_w = (w > 350) ? 350 : 280;
    int card_h = (h > 350) ? 330 : 250;
    int card_x = (w - card_w) / 2;
    int card_y = (h - card_h) / 2;

    display.setPartialWindow(0, 0, w, h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

        // Header
        display.fillRect(4, 4, w - 8, 26, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(14, 13);
        display.print("SETTINGS & POWER MENU");
        draw_battery_indicator(w - 36, 11, bat_pct, true);

        // Modal Card with Drop Shadow
        display.fillRect(card_x + 4, card_y + 4, card_w, card_h, GxEPD_BLACK);
        display.fillRect(card_x, card_y, card_w, card_h, GxEPD_WHITE);
        display.drawRect(card_x, card_y, card_w, card_h, GxEPD_BLACK);
        display.drawRect(card_x + 2, card_y + 2, card_w - 4, card_h - 4, GxEPD_BLACK);

        // Title inside Card
        display.fillRect(card_x + 4, card_y + 4, card_w - 8, 22, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + 11);
        display.print("QUICK OPTIONS");

        // Menu items
        const char* menu_labels[MENU_COUNT];
        char orient_buf[36];
        snprintf(orient_buf, sizeof(orient_buf), "Orientation: %s",
                 (current_rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*");

        menu_labels[MENU_RESUME]        = "Resume Reading";
        menu_labels[MENU_LIBRARY]       = "Book Library / Switch";
        menu_labels[MENU_BLE_SYNC]      = "Wireless BLE Upload (Sync)";
        menu_labels[MENU_ORIENTATION]   = orient_buf;
        menu_labels[MENU_JUMP]          = "Jump Page Scrubber";
        menu_labels[MENU_CLEAN_REFRESH] = "Full Anti-Ghost Refresh";
        menu_labels[MENU_SLEEP]         = "Power Off / Sleep...";

        int start_y = card_y + 32;
        int row_h = (card_h - 58) / MENU_COUNT;

        for (int i = 0; i < MENU_COUNT; i++) {
            int item_y = start_y + (i * row_h);
            if (i == selected_menu_idx) {
                display.fillRect(card_x + 6, item_y, card_w - 12, row_h - 3, GxEPD_BLACK);
                display.setTextColor(GxEPD_WHITE);
                display.setCursor(card_x + 14, item_y + (row_h / 2) - 4);
                display.printf("> %s", menu_labels[i]);
            } else {
                display.setTextColor(GxEPD_BLACK);
                display.setCursor(card_x + 14, item_y + (row_h / 2) - 4);
                display.printf("  %s", menu_labels[i]);
            }
        }

        // Subtext / Footer instruction inside card
        display.fillRect(card_x + 4, card_y + card_h - 22, card_w - 8, 18, GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 16, card_y + card_h - 14);
        display.print("Up          Down          Select");

        // Screen bottom footer
        display.fillRect(4, h - 26, w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(12, h - 18);
        display.print("Select 'Resume' to return | Auto-close 15s");

    } while (display.nextPage());
}

/**
 * Power Off / Deep Sleep Confirmation Screen.
 * Prompts user before entering Stage 2. D4 cancels/goes back, D9 confirms sleep.
 */
void render_power_confirm_screen(uint8_t bat_pct) {
    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    int card_w = (w > 350) ? 340 : 270;
    int card_h = 190;
    int card_x = (w - card_w) / 2;
    int card_y = (h - card_h) / 2;

    display.setPartialWindow(0, 0, w, h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

        // Header
        display.fillRect(4, 4, w - 8, 26, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(14, 13);
        display.print("POWER & SLEEP");
        draw_battery_indicator(w - 36, 11, bat_pct, true);

        // Modal Card with Drop Shadow
        display.fillRect(card_x + 4, card_y + 4, card_w, card_h, GxEPD_BLACK);
        display.fillRect(card_x, card_y, card_w, card_h, GxEPD_WHITE);
        display.drawRect(card_x, card_y, card_w, card_h, GxEPD_BLACK);
        display.drawRect(card_x + 2, card_y + 2, card_w - 4, card_h - 4, GxEPD_BLACK);

        // Header inside Card (Monochrome Black)
        display.fillRect(card_x + 4, card_y + 4, card_w - 8, 24, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + 12);
        display.print("POWER OFF / DEEP SLEEP?");

        // Confirmation Message
        display.setFont(&FreeSans9pt7b);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 16, card_y + 58);
        display.print("Put device to sleep?");

        display.setFont(NULL);
        display.setCursor(card_x + 16, card_y + 78);
        display.print("Enters Stage 2 ('Zzz' screensaver).");
        display.setCursor(card_x + 16, card_y + 92);
        display.print("Touch Menu pad to wake.");

        // Two Action Buttons: Back Button and Confirm Sleep
        int btn_w = (card_w - 36) / 2;
        int btn_y = card_y + 120;

        // BACK Button
        display.drawRect(card_x + 12, btn_y, btn_w, 40, GxEPD_BLACK);
        display.drawRect(card_x + 13, btn_y + 1, btn_w - 2, 38, GxEPD_BLACK);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 18, btn_y + 12);
        display.print("< BACK");
        display.setCursor(card_x + 18, btn_y + 25);
        display.print("Cancel");

        // CONFIRM SLEEP Button (Inverted solid black)
        display.fillRect(card_x + 24 + btn_w, btn_y, btn_w, 40, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 30 + btn_w, btn_y + 12);
        display.print("SLEEP >");
        display.setCursor(card_x + 30 + btn_w, btn_y + 25);
        display.print("Confirm");

        // Screen bottom footer
        display.fillRect(4, h - 26, w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(12, h - 18);
        display.print("Select Back to cancel | Select Sleep to confirm");

    } while (display.nextPage());
}

void render_ble_sync_screen(uint8_t bat_pct);

/**
 * Library Browser Screen.
 * Allows browsing stored .ebk files in LittleFS with saved reading percentages,
 * plus an option to enter 60s BLE Sync mode to receive books wirelessly from a web browser.
 */
void render_library_screen(int selected_idx, uint8_t bat_pct) {
    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    int card_w = (w > 350) ? 360 : 280;
    int card_h = (h > 350) ? 320 : 240;
    int card_x = (w - card_w) / 2;
    int card_y = (h - card_h) / 2;

    int book_count = StorageManager::get_stored_books(cached_books, MAX_STORED_BOOKS);
    int total_items = book_count + 1; // Existing books + "[+] Add More Books (BLE)"

    display.setPartialWindow(0, 0, w, h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

        // Header
        display.fillRect(4, 4, w - 8, 26, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(14, 13);
        display.printf("LIBRARY BROWSER (%d Books)", book_count);
        draw_battery_indicator(w - 36, 11, bat_pct, true);

        // Modal Card with Drop Shadow
        display.fillRect(card_x + 4, card_y + 4, card_w, card_h, GxEPD_BLACK);
        display.fillRect(card_x, card_y, card_w, card_h, GxEPD_WHITE);
        display.drawRect(card_x, card_y, card_w, card_h, GxEPD_BLACK);
        display.drawRect(card_x + 2, card_y + 2, card_w - 4, card_h - 4, GxEPD_BLACK);

        // Title inside Card
        display.fillRect(card_x + 4, card_y + 4, card_w - 8, 22, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + 11);
        display.print("SELECT BOOK OR ADD NEW");

        int max_display = (card_h > 260) ? 6 : 4;
        int display_count = min(total_items, max_display);
        int offset = 0;
        if (total_items > max_display) {
            offset = selected_idx - (max_display / 2);
            if (offset < 0) offset = 0;
            if (offset + max_display > total_items) offset = total_items - max_display;
        }

        int row_h = (card_h - 60) / display_count;
        int start_y = card_y + 30;

        for (int r = 0; r < display_count; r++) {
            int i = offset + r;
            int item_y = start_y + (r * row_h);
            bool is_selected = (i == selected_idx);

            if (i < book_count) {
                // Stored Book Entry
                bool is_active = (strcmp(cached_books[i].filename, current_book_path) == 0);
                int32_t saved_frame = is_active ? current_frame : ConfigManager::get_book_progress(cached_books[i].filename);
                int pct = (cached_books[i].total_frames > 0) ? ((saved_frame + 1) * 100) / cached_books[i].total_frames : 0;

                char item_buf[64];
                snprintf(item_buf, sizeof(item_buf), "%s%d. %s",
                         is_active ? "[*] " : "[ ] ",
                         i + 1, cached_books[i].title);

                char prog_buf[32];
                snprintf(prog_buf, sizeof(prog_buf), "P.%d/%d (%d%%)",
                         saved_frame + 1, cached_books[i].total_frames, pct);

                if (is_selected) {
                    display.fillRect(card_x + 6, item_y, card_w - 12, row_h - 3, GxEPD_BLACK);
                    u8g2Fonts.setForegroundColor(GxEPD_WHITE);
                    u8g2Fonts.setBackgroundColor(GxEPD_BLACK);
                } else {
                    u8g2Fonts.setForegroundColor(GxEPD_BLACK);
                    u8g2Fonts.setBackgroundColor(GxEPD_WHITE);
                }
                u8g2Fonts.setFont(u8g2_font_helvR10_tf);
                u8g2Fonts.setFontMode(1);
                u8g2Fonts.setCursor(card_x + 12, item_y + (row_h / 2) + 3);
                u8g2Fonts.print(item_buf);

                display.setFont(NULL);
                display.setTextColor(is_selected ? GxEPD_WHITE : GxEPD_BLACK);
                display.setCursor(card_x + card_w - 95, item_y + (row_h / 2) - 5);
                display.print(prog_buf);

            } else {
                // Action Entry: Add More Books via BLE
                if (is_selected) {
                    display.fillRect(card_x + 6, item_y, card_w - 12, row_h - 3, GxEPD_BLACK);
                    display.setTextColor(GxEPD_WHITE);
                } else {
                    display.setTextColor(GxEPD_BLACK);
                }
                display.setCursor(card_x + 12, item_y + (row_h / 2) - 5);
                display.print("[+] Add More Books (BLE)");
                display.setCursor(card_x + card_w - 95, item_y + (row_h / 2) - 5);
                display.print("Sync (60s)");
            }
        }

        // Subtext / Footer instruction inside card
        display.fillRect(card_x + 4, card_y + card_h - 22, card_w - 8, 18, GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 16, card_y + card_h - 14);
        display.print("Up          Down          Select");

        // Screen bottom footer
        display.fillRect(4, h - 26, w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(12, h - 18);
        display.print("Auto-close 15s | Press D9 to select");

    } while (display.nextPage());
}

/**
 * Dedicated BLE Book Sync Screen.
 * Renders status, 60s countdown, Web upload instructions, and live progress bar.
 */
void render_ble_sync_screen(uint8_t bat_pct) {
    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    int card_w = (w > 350) ? 360 : 280;
    int card_h = (h > 350) ? 320 : 240;
    int card_x = (w - card_w) / 2;
    int card_y = (h - card_h) / 2;

    BleSyncState st = BleSyncManager::get().get_state();
    int rem_sec = BleSyncManager::get().get_remaining_seconds();
    uint32_t rx = BleSyncManager::get().get_bytes_received();
    uint32_t total = BleSyncManager::get().get_total_expected_bytes();

    display.setPartialWindow(0, 0, w, h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

        // Top Header
        display.fillRect(4, 4, w - 8, 26, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(14, 13);
        display.print("BLE BOOK SYNC");
        draw_battery_indicator(w - 36, 11, bat_pct, true);

        // Modal Card with Drop Shadow
        display.fillRect(card_x + 4, card_y + 4, card_w, card_h, GxEPD_BLACK);
        display.fillRect(card_x, card_y, card_w, card_h, GxEPD_WHITE);
        display.drawRect(card_x, card_y, card_w, card_h, GxEPD_BLACK);
        display.drawRect(card_x + 2, card_y + 2, card_w - 4, card_h - 4, GxEPD_BLACK);

        // Title inside Card
        display.fillRect(card_x + 4, card_y + 4, card_w - 8, 22, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + 11);
        display.print("WIRELESS SYNC & FIRMWARE OTA");

        // Info lines
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(card_x + 14, card_y + 48);
        display.print("Device: ESP32-Ereader");

        display.setFont(NULL);
        display.setCursor(card_x + 14, card_y + 68);
        display.print("1. Open Web Tool in Chrome or Edge");
        display.setCursor(card_x + 14, card_y + 82);
        display.print("2. Connect BLE & choose .ebk book or firmware .bin");

        // Status box
        display.drawRect(card_x + 10, card_y + 100, card_w - 20, 72, GxEPD_BLACK);
        if (st == BLE_SYNC_ADVERTISING) {
            display.setCursor(card_x + 18, card_y + 118);
            display.printf("Status: Advertising... (%ds remaining)", rem_sec);
            display.setCursor(card_x + 18, card_y + 138);
            display.print("Waiting for browser connection...");
        } else if (st == BLE_SYNC_CONNECTED) {
            display.setCursor(card_x + 18, card_y + 118);
            display.print("Status: Web Browser Connected!");
            display.setCursor(card_x + 18, card_y + 138);
            display.print("Send book or firmware from browser...");
        } else if (st == BLE_SYNC_RECEIVING || st == BLE_SYNC_OTA_RECEIVING) {
            int pct = (total > 0) ? (int)((rx * 100ULL) / total) : 0;
            display.setCursor(card_x + 18, card_y + 114);
            if (st == BLE_SYNC_OTA_RECEIVING) {
                display.printf("Flashing OTA: %u / %u KB (%d%%)", (unsigned int)(rx / 1024), (unsigned int)(total / 1024), pct);
            } else {
                display.printf("Receiving: %u / %u KB (%d%%)", (unsigned int)(rx / 1024), (unsigned int)(total / 1024), pct);
            }
            
            // Progress bar
            display.drawRect(card_x + 18, card_y + 130, card_w - 36, 14, GxEPD_BLACK);
            int fill = (pct * (card_w - 40)) / 100;
            if (fill > 0) {
                display.fillRect(card_x + 20, card_y + 132, fill, 10, GxEPD_BLACK);
            }
        } else if (st == BLE_SYNC_SUCCESS) {
            display.setCursor(card_x + 18, card_y + 125);
            if (BleSyncManager::get().is_ota_in_progress()) {
                display.print("Firmware Updated! Rebooting reader...");
            } else {
                display.print("Transfer Complete! Opening book...");
            }
        } else if (st == BLE_SYNC_FAILED) {
            display.setCursor(card_x + 18, card_y + 125);
            display.print("Transfer Failed or Aborted.");
        } else if (st == BLE_SYNC_TIMEOUT_EXPIRED) {
            display.setCursor(card_x + 18, card_y + 125);
            display.print("60s Timeout Expired. Returning...");
        }

        // Subtext / Footer inside Card
        display.fillRect(card_x + 4, card_y + card_h - 22, card_w - 8, 18, GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 16, card_y + card_h - 14);
        display.print("[D4] Cancel & Return to Library");

        // Screen bottom footer
        display.fillRect(4, h - 26, w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(12, h - 18);
        display.print("Press D4 (Prev) to cancel | 60s Auto-Timeout");

    } while (display.nextPage());
}

/**
 * Interactive Page Jump scrubber screen.
 */
void render_jump_screen(int target, int total) {
    display.setRotation(current_rotation);
    int w = display.width();
    int h = display.height();

    int card_w = (w > 350) ? 320 : 260;
    int card_h = 170;
    int card_x = (w - card_w) / 2;
    int card_y = (h - card_h) / 2;

    display.setPartialWindow(0, 0, w, h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.drawRect(2, 2, w - 4, h - 4, GxEPD_BLACK);

        // Header
        display.fillRect(4, 4, w - 8, 26, GxEPD_BLACK);
        display.setFont(NULL);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(14, 13);
        display.print("PAGE NAVIGATION");

        // Card
        display.fillRect(card_x + 4, card_y + 4, card_w, card_h, GxEPD_BLACK);
        display.fillRect(card_x, card_y, card_w, card_h, GxEPD_WHITE);
        display.drawRect(card_x, card_y, card_w, card_h, GxEPD_BLACK);

        display.fillRect(card_x + 4, card_y + 4, card_w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + 11);
        display.print("SCRUB TO TARGET PAGE");

        // Target page display
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(card_x + 20, card_y + 55);
        display.printf("Page %d of %d", target + 1, total);

        // Progress bar
        display.drawRect(card_x + 20, card_y + 70, card_w - 40, 14, GxEPD_BLACK);
        int bar_fill = ((target + 1) * (card_w - 44)) / total;
        if (bar_fill > 0) {
            display.fillRect(card_x + 22, card_y + 72, bar_fill, 10, GxEPD_BLACK);
        }

        // Instructions
        display.setFont(NULL);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(card_x + 16, card_y + 105);
        display.print("-1 Page                  +1 Page");

        display.fillRect(card_x + 4, card_y + card_h - 26, card_w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(card_x + 12, card_y + card_h - 16);
        display.print("Confirm Jump");

        display.fillRect(4, h - 26, w - 8, 22, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(12, h - 18);
        display.print("Confirm Jump | Auto-closes in 15s");

    } while (display.nextPage());
}

// Stage 2: Clean "Zzz" Screensaver with non-overlapping randomized placements
struct ZzzItem {
    int16_t x, y, w, h;
    uint8_t size;
    uint16_t color;
    char text[8];
};

void render_stage2_zzz_screen(uint32_t bat_mv, uint8_t bat_pct) {
    Serial.println("\n[EPD] Entering STAGE 2: Rendering clean non-overlapping 'Zzz' sleep screen...");
    uint32_t t0 = millis();

    epdSPI.begin(EPD_SCLK_PIN, -1, EPD_MOSI_PIN, -1);
    display.epd2.selectSPI(epdSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    display.init(115200, /*initial=*/true, /*reset_duration=*/20, /*pulldown_rst_mode=*/false);
    u8g2Fonts.begin(display);
    display.setRotation(current_rotation);

    int disp_w = display.width();
    int disp_h = display.height();

    // Pre-calculate non-overlapping Zzz placements
    const int MAX_ZZZ = 35;
    ZzzItem items[MAX_ZZZ];
    int count = 0;

    const char* words[] = {"Zzz", "zzz", "ZZZ", "Zz", "zz"};
    const int num_words = sizeof(words) / sizeof(words[0]);

    srand(boot_count * 7919 + bat_mv + millis());

    for (int attempt = 0; attempt < 600 && count < 26; attempt++) {
        const char* word = words[rand() % num_words];
        int len = strlen(word);
        int size = 1 + (rand() % 4); // sizes 1 to 4
        int w = len * 6 * size + 14; // width with margin
        int h = 8 * size + 14;       // height with margin

        if (disp_w - w - 20 <= 0 || disp_h - h - 20 <= 0) continue;

        int rx = 10 + (rand() % (disp_w - w - 20));
        int ry = 10 + (rand() % (disp_h - h - 20));

        // Check AABB collision against already placed items
        bool overlap = false;
        for (int j = 0; j < count; j++) {
            if (rx < items[j].x + items[j].w &&
                rx + w > items[j].x &&
                ry < items[j].y + items[j].h &&
                ry + h > items[j].y) {
                overlap = true;
                break;
            }
        }

        if (!overlap) {
            items[count].x = rx;
            items[count].y = ry;
            items[count].w = w;
            items[count].h = h;
            items[count].size = size;
            items[count].color = GxEPD_BLACK;
            strncpy(items[count].text, word, sizeof(items[count].text) - 1);
            items[count].text[sizeof(items[count].text) - 1] = '\0';
            count++;
        }
    }

    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setFont(NULL);

        for (int i = 0; i < count; i++) {
            display.setTextSize(items[i].size);
            display.setTextColor(items[i].color);
            display.setCursor(items[i].x + 7, items[i].y + 7);
            display.print(items[i].text);
        }
    } while (display.nextPage());

    display.powerOff();
    display.hibernate();

    Serial.printf("[EPD] Clean Zzz screensaver rendered (%d non-overlapping items) in %lu ms!\n", count, millis() - t0);
}

// Stage 2 Deep Sleep: "Zzz" screen displayed. ONLY Power (D9) wakes up!
void enter_stage2_deep_sleep(uint32_t bat_mv, uint8_t bat_pct) {
    current_stage = STAGE_DEEP_ZZZ;

    // Immediately commit all reading progress and settings to NVS flash
    ConfigManager::commit_immediate();

    // Terminate Core 0 touch task before entering deep sleep
    touch_task_running = false;
    if (touchTaskHandle != NULL) {
        vTaskDelete(touchTaskHandle);
        touchTaskHandle = NULL;
    }
    delay(50);

    render_stage2_zzz_screen(bat_mv, bat_pct);

    Serial.println("\n[PWR] Entering STAGE 2 (Deeper Sleep with Zzz screen)...");
    Serial.println("[PWR] Waiting for finger release on Power pad (D9)...");

    uint32_t release_start = millis();
    while (millis() - release_start < 2500) {
        uint32_t r = touchRead(TOUCH_MENU_PIN);
        if (tracker_menu.initialized && r <= (tracker_menu.baseline + 120)) {
            break;
        }
        delay(20);
    }
    delay(200);

    Serial.println("[PWR] Arming Power Pad (D9 / GPIO 8) as deep sleep wake source (threshold: +250 delta)...");
    touchSleepWakeUpEnable(TOUCH_MENU_PIN, 250);

    // Reset sleep channel benchmark so current untouched state is captured as reference
    touch_pad_sleep_channel_reset_benchmark();
    delay(250); // Allow hardware filter to stabilize

    uint32_t benchmark = 0;
    touch_pad_sleep_channel_read_benchmark(TOUCH_PAD_MENU, &benchmark);
    Serial.printf("[PWR] Benchmark captured: %u | Trigger threshold: >= %u\n", benchmark, benchmark + 250);

    Serial.println("[PWR] Stage 2 Armed: ONLY Power (D9) can wake the device (~20 uA). Goodnight!");
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    boot_count++;
    Serial.begin(115200);

    // Completely reset touch peripheral from any previous deep sleep wake state
    touch_pad_deinit();
    delay(20);
    touch_pad_sleep_channel_enable(TOUCH_PAD_MENU, false);
    touch_pad_sleep_channel_enable(TOUCH_PAD_PREV, false);
    touch_pad_sleep_channel_enable(TOUCH_PAD_NEXT, false);
    delay(10);
    touchRead(TOUCH_PREV_PIN);
    touchRead(TOUCH_NEXT_PIN);
    touchRead(TOUCH_MENU_PIN);

    tracker_prev.init(TOUCH_PREV_PIN);
    tracker_next.init(TOUCH_NEXT_PIN);
    tracker_menu.init(TOUCH_MENU_PIN);

    uint32_t w = millis();
    while (!Serial && millis() - w < 1200) delay(10);

    Serial.println("\n=============================================");
    Serial.printf("  ESP32-S3 E-Reader 2-Stage System (Boot #%d) \n", boot_count);
    Serial.printf("  Current Orientation: %s (setRotation=%d)\n",
                  (current_rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*", current_rotation);
    Serial.println("=============================================");

    StorageManager::begin();
    ConfigManager::begin();

    // Restore persistent config from flash NVS
    current_rotation = ConfigManager::get_rotation();
    zen_mode = ConfigManager::get_zen_mode();
    ConfigManager::get_current_book_path(current_book_path, sizeof(current_book_path));
    current_frame = ConfigManager::get_current_frame();
    boot_count = ConfigManager::get_boot_count();

    // Sample Battery Voltage
    uint32_t bat_mv = read_battery_millivolts();
    uint8_t bat_pct = calculate_battery_percentage(bat_mv);
    last_bat_mv = bat_mv;
    ConfigManager::set_last_battery_mv(bat_mv);
    Serial.printf("[BAT] Measured: %u mV (%u%%)\n", (unsigned int)bat_mv, bat_pct);

    // Verify active book exists on LittleFS or auto-select from storage
    if (!LittleFS.exists(current_book_path)) {
        Serial.printf("[BOOK] Current book path '%s' not found on LittleFS. Scanning storage...\n", current_book_path);
        int count = StorageManager::get_stored_books(cached_books, MAX_STORED_BOOKS);
        if (count > 0) {
            current_frame = ConfigManager::switch_book(cached_books[0].filename);
            strncpy(current_book_path, cached_books[0].filename, sizeof(current_book_path) - 1);
            current_book_path[sizeof(current_book_path) - 1] = '\0';
            total_frames = cached_books[0].total_frames;
            Serial.printf("[BOOK] Auto-selected available book: '%s' (Title: '%s', %d pages)\n",
                          current_book_path, cached_books[0].title, total_frames);
        } else {
            Serial.println("[BOOK] No .ebk files found in LittleFS. Using built-in demo pages.");
            total_frames = 4;
        }
    } else {
        EbkFormatVersion ver = StorageManager::get_book_version(current_book_path);
        if (ver == EBK_FORMAT_V2_TEXT) {
            Ebk2Header h2;
            if (StorageManager::get_v2_header(current_book_path, &h2)) {
                total_frames = h2.total_chapters;
                uint16_t saved_chap = 0;
                uint32_t saved_offset = 0;
                ConfigManager::get_book_progress(current_book_path, &saved_chap, &saved_offset);
                current_chapter_idx = (saved_chap < h2.total_chapters) ? saved_chap : 0;
                current_text_file_offset = saved_offset;
                if (current_text_file_offset == 0) {
                    Ebk2ChapterEntry ch_entry;
                    if (StorageManager::read_v2_chapter_entry(current_book_path, current_chapter_idx, &ch_entry)) {
                        current_text_file_offset = ch_entry.offset;
                    }
                }
                Serial.printf("[BOOK] Active v2 book: '%s' (Title: '%s', Author: '%s', Chapters: %d, Resumed at Ch %d, Offset %u)\n",
                              current_book_path, h2.title, h2.author, h2.total_chapters, current_chapter_idx + 1, (unsigned int)current_text_file_offset);
            }
        } else if (ver == EBK_FORMAT_V1_BITMAP) {
            EbkHeader hdr;
            if (StorageManager::get_book_header(current_book_path, &hdr)) {
                total_frames = hdr.total_frames;
                Serial.printf("[BOOK] Active v1 book: '%s' (Title: '%s', Total Frames: %d)\n",
                              current_book_path, hdr.title, total_frames);
            } else {
                total_frames = 4;
            }
        } else {
            total_frames = 4;
        }
    }

    if (total_frames <= 0) total_frames = 4;
    if (current_frame >= total_frames) {
        current_frame = max(0, total_frames - 1);
        ConfigManager::set_current_frame(current_frame, true);
    }

    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    Serial.printf("[WAKE] Wakeup cause code: %d\n", wakeup_cause);

    if (wakeup_cause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
        touch_pad_t touch_pad = esp_sleep_get_touchpad_wakeup_status();
        Serial.printf("[WAKE] Woken up by touch pad #%d!\n", touch_pad);

        // User woke up device by pressing D9. Wait for release so it doesn't trigger an immediate UI tap
        delay(100);
        uint32_t val_wake = touchRead(TOUCH_MENU_PIN);
        uint32_t rel_start = millis();
        while (millis() - rel_start < 1500) {
            uint32_t val = touchRead(TOUCH_MENU_PIN);
            if (val + 350 < val_wake || val < 34500) {
                // Dropped by 350 counts or returned to baseline -> finger released!
                break;
            }
            delay(20);
        }
        delay(100);
    }

    // Initialize SPI bus and EPD Hardware
    Serial.println("[EPD] Initializing SPI bus and display controller...");
    epdSPI.begin(EPD_SCLK_PIN, -1, EPD_MOSI_PIN, -1);
    display.epd2.selectSPI(epdSPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    display.init(115200, /*initial=*/true, /*reset_duration=*/20, /*pulldown_rst_mode=*/false);
    u8g2Fonts.begin(display);

    // Waking up from STAGE 2 (Zzz Screensaver) via Power Button
    if (current_stage == STAGE_DEEP_ZZZ) {
        Serial.println("[WAKE] >>> Woke up from STAGE 2 ('Zzz' Screen) via Power Button! <<<");
        current_stage = STAGE_READING;
        render_current_reading_page(bat_mv, bat_pct, /*fast_bw=*/false);
        Serial.printf("[APP] Resumed reading '%s' at Page %d/%d!\n", current_book_path, current_frame + 1, total_frames);
    } else {
        // Cold boot / Reset / USB Power On / Firmware Flashing
        Serial.println("[WAKE] Cold boot / USB Power On / Reset");
        current_stage = STAGE_READING;
        render_current_reading_page(bat_mv, bat_pct, /*fast_bw=*/false);
        Serial.printf("[APP] Restored reading '%s' at Page %d/%d (Rot: %s)!\n",
                      current_book_path, current_frame + 1, total_frames,
                      (current_rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*");
    }

    // Calibrate touch baselines AFTER display is initialized and settled in active reading mode
    Serial.println("[CALIB] Locking ambient touch baselines in active display state...");
    delay(400); // 400ms power rail and SPI line stabilization
    tracker_prev.calibrate(40);
    tracker_next.calibrate(40);
    tracker_menu.calibrate(40);
    Serial.printf("[CALIB] Baselines locked: D4=%.0f, D5=%.0f, D9=%.0f (Triggers: D4/D5/D9 >= +280)\n",
                  tracker_prev.baseline, tracker_next.baseline, tracker_menu.baseline);

    // Create FreeRTOS queue and launch dedicated touch scanning task on Core 0
    inputQueue = xQueueCreate(32, sizeof(ButtonEvent));
    touch_task_running = true;
    xTaskCreatePinnedToCore(
        touch_task,
        "touch_task",
        4096,
        NULL,
        2,
        &touchTaskHandle,
        0   // Core 0
    );
    Serial.println("[RTOS] Dedicated Touch Task active on Core 0! Uninterrupted scanning during e-ink refreshes.");

    Serial.printf("[APP] Active Reading Mode. [D4]=Prev, [D5]=Next, [D9]=Menu. HOLD D9 2s for Settings/Power. Auto-sleep: %llu mins.\n", AWAKE_TIMEOUT_MS / 60000ULL);
}

void clear_input_queue() {
    if (inputQueue != NULL) {
        ButtonEvent dummy;
        while (xQueueReceive(inputQueue, &dummy, 0) == pdTRUE) {}
    }
}

void open_settings_menu() {
    Serial.println("\n[APP] >>> Opening Settings / Power Menu! <<<");
    clear_input_queue();
    current_ui_state = UI_STATE_MENU;
    selected_menu_idx = 0;
    menu_active_timer = millis();
    uint32_t mv = read_battery_millivolts();
    render_menu_screen(calculate_battery_percentage(mv));
}

void open_power_confirm() {
    Serial.println("\n[APP] >>> Opening Power Off Confirmation Dialog! <<<");
    clear_input_queue();
    current_ui_state = UI_STATE_POWER_CONFIRM;
    menu_active_timer = millis();
    uint32_t mv = read_battery_millivolts();
    render_power_confirm_screen(calculate_battery_percentage(mv));
}

void open_library_menu() {
    Serial.println("\n[APP] >>> Opening Library Browser! <<<");
    clear_input_queue();
    current_ui_state = UI_STATE_LIBRARY;
    menu_active_timer = millis();

    int book_count = StorageManager::get_stored_books(cached_books, MAX_STORED_BOOKS);
    selected_library_idx = 0;
    for (int i = 0; i < book_count; i++) {
        if (strcmp(cached_books[i].filename, current_book_path) == 0) {
            selected_library_idx = i;
            break;
        }
    }

    uint32_t mv = read_battery_millivolts();
    render_library_screen(selected_library_idx, calculate_battery_percentage(mv));
}

void start_ble_sync_mode() {
    Serial.println("\n[APP] >>> Starting 60s BLE Book Sync Mode! <<<");
    clear_input_queue();
    current_ui_state = UI_STATE_BLE_SYNC;
    BleSyncManager::get().begin();
    uint32_t mv = read_battery_millivolts();
    render_ble_sync_screen(calculate_battery_percentage(mv));
}

void stop_ble_sync_mode(bool reload_library = true) {
    Serial.println("\n[APP] >>> Stopping BLE Book Sync Mode! <<<");
    BleSyncManager::get().stop();
    if (reload_library) {
        open_library_menu();
    }
}

void loop() {
    static uint32_t awake_start = millis();
    static uint32_t last_telemetry = 0;

    // UI render debounce flags
    static bool menu_dirty = false;
    static uint32_t menu_dirty_time = 0;
    static bool jump_dirty = false;
    static uint32_t jump_dirty_time = 0;

    // Live Telemetry (prints every 300ms so user can observe touch response in real-time)
    if (millis() - last_telemetry >= 300) {
        last_telemetry = millis();
        Serial.printf("[TOUCH] D4(PREV): %5u (b:%5.0f d:%+5d) [%c] | D5(NEXT): %5u (b:%5.0f d:%+5d) [%c] | D9(MENU): %5u (b:%5.0f d:%+5d) [%c] | UI:%d\n",
                      tracker_prev.last_raw, tracker_prev.baseline, tracker_prev.last_delta, tracker_prev.is_touched ? 'X' : ' ',
                      tracker_next.last_raw, tracker_next.baseline, tracker_next.last_delta, tracker_next.is_touched ? 'X' : ' ',
                      tracker_menu.last_raw, tracker_menu.baseline, tracker_menu.last_delta, tracker_menu.is_touched ? 'X' : ' ',
                      (int)current_ui_state);
    }

    // Serial command testing: 'n'/'d'=Next/Down, 'p'/'u'=Prev/Up, 'm'/'s'=Menu/Select, 'c'=Clean Full Refresh
    if (Serial.available()) {
        char c = Serial.read();
        ButtonEvent evt = EVENT_NONE;
        if (c == 'p' || c == 'u') evt = EVENT_PREV;
        else if (c == 'n' || c == 'd') evt = EVENT_NEXT;
        else if (c == 'm' || c == 's') evt = EVENT_MENU_TAP;
        else if (c == 'h' || c == 'P') evt = EVENT_MENU_HOLD;
        else if (c == 'b' || c == 'z' || c == 't') {
            Serial.println("\n[CMD] >>> Serial Toggle Zen Mode (Hide Bars) <<<");
            evt = EVENT_TOGGLE_BARS;
        }
        else if (c == 'c') {
            Serial.println("\n[CMD] >>> Serial 'c' -> CLEAN FULL REFRESH <<<");
            uint32_t mv = read_battery_millivolts();
            render_current_reading_page(mv, calculate_battery_percentage(mv), /*fast_bw=*/false);
        }
        else if (c == 'S' || c == 'k') {
            Serial.println("\n[CMD] >>> Serial Dump Persistent Config <<<");
            ConfigManager::dump_state();
            StorageManager::list_books();
        }
        else if (c == 'R') {
            Serial.println("\n[CMD] >>> Serial Factory Reset Config <<<");
            ConfigManager::factory_reset();
            current_frame = ConfigManager::get_current_frame();
            current_rotation = ConfigManager::get_rotation();
            zen_mode = ConfigManager::get_zen_mode();
            ConfigManager::get_current_book_path(current_book_path, sizeof(current_book_path));
            uint32_t mv = read_battery_millivolts();
            render_current_reading_page(mv, calculate_battery_percentage(mv), /*fast_bw=*/false);
        }
        if (evt != EVENT_NONE && inputQueue != NULL) {
            xQueueSend(inputQueue, &evt, 0);
        }
    }

    // =========================================================================
    // EVENT PROCESSING FROM ASYNC QUEUE
    // =========================================================================
    ButtonEvent evt;
    if (inputQueue != NULL && xQueueReceive(inputQueue, &evt, 0) == pdTRUE) {
        awake_start = millis();
        menu_active_timer = millis();

        // Global Hold 2s on D9 -> Always opens Power dialog
        if (evt == EVENT_MENU_HOLD) {
            Serial.println("\n[TOUCH] >>> D9 HELD 2s -> Opening Power / Sleep Dialog! <<<");
            menu_dirty = false;
            jump_dirty = false;
            open_power_confirm();
        } else {
            switch (current_ui_state) {
                case UI_STATE_READING: {
                    EbkFormatVersion ver = StorageManager::get_book_version(current_book_path);
                    if (evt == EVENT_TOGGLE_BARS) {
                        zen_mode = !zen_mode;
                        ConfigManager::set_zen_mode(zen_mode);
                        Serial.printf("\n[UI] >>> Prev+Next Chord: Toggled Zen Reading Mode -> %s <<<\n",
                                      zen_mode ? "ON (Hide Bars, Max Text)" : "OFF (Show Bars)");
                        uint32_t mv = read_battery_millivolts();
                        render_current_reading_page(mv, calculate_battery_percentage(mv), /*fast_bw=*/false);
                    } else if (evt == EVENT_PREV) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == EVENT_PREV) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }

                        if (ver == EBK_FORMAT_V2_TEXT) {
                            if (page_stack_depth > 0) {
                                current_text_file_offset = page_offset_stack[--page_stack_depth];
                                current_frame = max(0, current_frame - 1);
                                ConfigManager::set_current_frame(current_frame);
                                ConfigManager::set_book_progress(current_book_path, current_frame, current_chapter_idx, current_text_file_offset);
                                Serial.printf("\n[TOUCH] >>> PREV PAGE (Stack pop -> Page %d, Offset %u) <<<\n",
                                              current_frame + 1, (unsigned int)current_text_file_offset);
                                uint32_t mv = read_battery_millivolts();
                                render_current_reading_page(mv, calculate_battery_percentage(mv));
                            } else if (current_chapter_idx > 0) {
                                current_chapter_idx--;
                                Ebk2ChapterEntry prev_chap;
                                if (StorageManager::read_v2_chapter_entry(current_book_path, current_chapter_idx, &prev_chap)) {
                                    current_text_file_offset = prev_chap.offset;
                                    current_frame = max(0, current_frame - 1);
                                    ConfigManager::set_current_frame(current_frame);
                                    ConfigManager::set_book_progress(current_book_path, current_frame, current_chapter_idx, current_text_file_offset);
                                    Serial.printf("\n[TOUCH] >>> PREV CHAPTER (Ch %d -> Offset %u) <<<\n",
                                                  current_chapter_idx + 1, (unsigned int)current_text_file_offset);
                                    uint32_t mv = read_battery_millivolts();
                                    render_current_reading_page(mv, calculate_battery_percentage(mv));
                                }
                            } else {
                                Serial.println("[TOUCH] Already at First Page of Book!");
                            }
                        } else {
                            if (current_frame > 0) {
                                current_frame = max(current_frame - step, 0);
                                ConfigManager::set_current_frame(current_frame); // debounced save
                                ConfigManager::set_book_progress(current_book_path, current_frame);
                                Serial.printf("\n[TOUCH] >>> PREV PAGE (step %d -> Page %d) <<<\n", step, current_frame + 1);
                                uint32_t mv = read_battery_millivolts();
                                render_current_reading_page(mv, calculate_battery_percentage(mv));
                            } else {
                                Serial.println("[TOUCH] Already at First Page!");
                            }
                        }
                    } else if (evt == EVENT_NEXT) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == EVENT_NEXT) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }

                        if (ver == EBK_FORMAT_V2_TEXT) {
                            Ebk2ChapterEntry chap;
                            StorageManager::read_v2_chapter_entry(current_book_path, current_chapter_idx, &chap);
                            uint32_t chap_end = chap.offset + chap.length;

                            if (next_text_file_offset > current_text_file_offset && next_text_file_offset < chap_end) {
                                if (page_stack_depth < NAV_STACK_SIZE) {
                                    page_offset_stack[page_stack_depth++] = current_text_file_offset;
                                }
                                current_text_file_offset = next_text_file_offset;
                                current_frame++;
                                ConfigManager::set_current_frame(current_frame);
                                ConfigManager::set_book_progress(current_book_path, current_frame, current_chapter_idx, current_text_file_offset);
                                Serial.printf("\n[TOUCH] >>> NEXT PAGE (Offset %u -> %u) <<<\n",
                                              (unsigned int)page_offset_stack[page_stack_depth - 1], (unsigned int)current_text_file_offset);
                                uint32_t mv = read_battery_millivolts();
                                render_current_reading_page(mv, calculate_battery_percentage(mv));
                            } else {
                                // Advance to next chapter
                                Ebk2Header h2;
                                if (StorageManager::get_v2_header(current_book_path, &h2) && (current_chapter_idx + 1 < h2.total_chapters)) {
                                    current_chapter_idx++;
                                    Ebk2ChapterEntry next_chap;
                                    if (StorageManager::read_v2_chapter_entry(current_book_path, current_chapter_idx, &next_chap)) {
                                        if (page_stack_depth < NAV_STACK_SIZE) {
                                            page_offset_stack[page_stack_depth++] = current_text_file_offset;
                                        }
                                        current_text_file_offset = next_chap.offset;
                                        current_frame++;
                                        ConfigManager::set_current_frame(current_frame);
                                        ConfigManager::set_book_progress(current_book_path, current_frame, current_chapter_idx, current_text_file_offset);
                                        Serial.printf("\n[TOUCH] >>> NEXT CHAPTER (Ch %d -> Offset %u) <<<\n",
                                                      current_chapter_idx + 1, (unsigned int)current_text_file_offset);
                                        uint32_t mv = read_battery_millivolts();
                                        render_current_reading_page(mv, calculate_battery_percentage(mv));
                                    }
                                } else {
                                    Serial.println("[TOUCH] Already at Last Page of Book!");
                                }
                            }
                        } else {
                            if (current_frame < total_frames - 1) {
                                current_frame = min(current_frame + step, total_frames - 1);
                                ConfigManager::set_current_frame(current_frame); // debounced save
                                ConfigManager::set_book_progress(current_book_path, current_frame);
                                Serial.printf("\n[TOUCH] >>> NEXT PAGE (step %d -> Page %d) <<<\n", step, current_frame + 1);
                                uint32_t mv = read_battery_millivolts();
                                render_current_reading_page(mv, calculate_battery_percentage(mv));
                            } else {
                                Serial.println("[TOUCH] Already at Last Page!");
                            }
                        }
                    } else if (evt == EVENT_MENU_TAP) {
                        Serial.println("\n[TOUCH] >>> Menu TAP -> Opening Settings Menu! <<<");
                        open_settings_menu();
                    }
                    break;
                }

                case UI_STATE_MENU: {
                    ButtonEvent up_evt   = (current_rotation == ORIENT_PORTRAIT) ? EVENT_NEXT : EVENT_PREV;
                    ButtonEvent down_evt = (current_rotation == ORIENT_PORTRAIT) ? EVENT_PREV : EVENT_NEXT;

                    if (evt == up_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == up_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        selected_menu_idx = (selected_menu_idx - step) % MENU_COUNT;
                        if (selected_menu_idx < 0) selected_menu_idx += MENU_COUNT;
                        Serial.printf("\n[MENU] Up (via %s, step %d) -> Selected Item #%d\n",
                                      (up_evt == EVENT_PREV ? "D4" : "D5"), step, selected_menu_idx);
                        menu_dirty = true;
                        menu_dirty_time = millis();
                    } else if (evt == down_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == down_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        selected_menu_idx = (selected_menu_idx + step) % MENU_COUNT;
                        Serial.printf("\n[MENU] Down (via %s, step %d) -> Selected Item #%d\n",
                                      (down_evt == EVENT_PREV ? "D4" : "D5"), step, selected_menu_idx);
                        menu_dirty = true;
                        menu_dirty_time = millis();
                    } else if (evt == EVENT_MENU_TAP) {
                        Serial.printf("\n[MENU] Select -> Executing Menu Item #%d\n", selected_menu_idx);
                        menu_dirty = false;
                        uint32_t mv = read_battery_millivolts();
                        uint8_t pct = calculate_battery_percentage(mv);

                        switch (selected_menu_idx) {
                            case MENU_RESUME:
                                current_ui_state = UI_STATE_READING;
                                render_current_reading_page(mv, pct);
                                break;
                            case MENU_LIBRARY:
                                open_library_menu();
                                break;
                            case MENU_BLE_SYNC:
                                start_ble_sync_mode();
                                break;
                            case MENU_ORIENTATION:
                                current_rotation = (current_rotation == ORIENT_LANDSCAPE) ? ORIENT_PORTRAIT : ORIENT_LANDSCAPE;
                                ConfigManager::set_rotation(current_rotation, true);
                                Serial.printf("[SETTINGS] Toggled Orientation to: %s (setRotation=%d) [Saved to NVS]\n",
                                              (current_rotation == ORIENT_LANDSCAPE) ? "Landscape" : "Portrait 90*", current_rotation);
                                render_menu_screen(pct);
                                break;
                            case MENU_JUMP:
                                current_ui_state = UI_STATE_JUMP;
                                jump_target_frame = current_frame;
                                render_jump_screen(jump_target_frame, total_frames);
                                break;
                            case MENU_CLEAN_REFRESH:
                                current_ui_state = UI_STATE_READING;
                                render_current_reading_page(mv, pct, /*fast_bw=*/false);
                                break;
                            case MENU_SLEEP:
                                open_power_confirm();
                                break;
                        }
                    }
                    break;
                }

                case UI_STATE_LIBRARY: {
                    ButtonEvent up_evt   = (current_rotation == ORIENT_PORTRAIT) ? EVENT_NEXT : EVENT_PREV;
                    ButtonEvent down_evt = (current_rotation == ORIENT_PORTRAIT) ? EVENT_PREV : EVENT_NEXT;

                    int book_count = StorageManager::get_stored_books(cached_books, MAX_STORED_BOOKS);
                    int total_items = book_count + 1; // Existing books + "[+] Add More Books (BLE)"

                    if (evt == up_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == up_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        selected_library_idx = (selected_library_idx - step) % total_items;
                        if (selected_library_idx < 0) selected_library_idx += total_items;
                        if (selected_library_idx < book_count) {
                            Serial.printf("\n[LIBRARY] Up -> Selected Book #%d (\"%s\")\n",
                                          selected_library_idx + 1, cached_books[selected_library_idx].title);
                        } else {
                            Serial.println("\n[LIBRARY] Up -> Selected '[+] Add More Books (BLE Sync)'");
                        }
                        uint32_t mv = read_battery_millivolts();
                        render_library_screen(selected_library_idx, calculate_battery_percentage(mv));
                    } else if (evt == down_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == down_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        selected_library_idx = (selected_library_idx + step) % total_items;
                        if (selected_library_idx < book_count) {
                            Serial.printf("\n[LIBRARY] Down -> Selected Book #%d (\"%s\")\n",
                                          selected_library_idx + 1, cached_books[selected_library_idx].title);
                        } else {
                            Serial.println("\n[LIBRARY] Down -> Selected '[+] Add More Books (BLE Sync)'");
                        }
                        uint32_t mv = read_battery_millivolts();
                        render_library_screen(selected_library_idx, calculate_battery_percentage(mv));
                    } else if (evt == EVENT_MENU_TAP) {
                        if (selected_library_idx == book_count) {
                            // User selected "[+] Add More Books (BLE Sync)"!
                            start_ble_sync_mode();
                        } else if (selected_library_idx < book_count) {
                            // User selected an existing book! Open and resume at saved bookmark!
                            const char* target_path = cached_books[selected_library_idx].filename;
                            Serial.printf("\n[LIBRARY] Select -> Opening Book: %s (\"%s\")\n",
                                          target_path, cached_books[selected_library_idx].title);

                            if (strcmp(target_path, current_book_path) != 0) {
                                // Save outgoing book bookmark
                                ConfigManager::set_book_progress(current_book_path, current_frame, current_chapter_idx, current_text_file_offset);

                                uint16_t saved_chap = 0;
                                uint32_t saved_offset = 0;
                                current_frame = ConfigManager::switch_book(target_path, &saved_chap, &saved_offset);
                                strncpy(current_book_path, target_path, sizeof(current_book_path) - 1);
                                current_book_path[sizeof(current_book_path) - 1] = '\0';
                                total_frames = cached_books[selected_library_idx].total_frames;
                                if (total_frames <= 0) total_frames = 4;
                                if (current_frame >= total_frames) current_frame = 0;

                                page_stack_depth = 0;
                                current_chapter_idx = saved_chap;
                                current_text_file_offset = saved_offset;

                                EbkFormatVersion v = StorageManager::get_book_version(target_path);
                                if (v == EBK_FORMAT_V2_TEXT) {
                                    Ebk2Header h2;
                                    if (StorageManager::get_v2_header(target_path, &h2)) {
                                        if (current_chapter_idx >= h2.total_chapters) current_chapter_idx = 0;
                                    }
                                    if (current_text_file_offset == 0) {
                                        Ebk2ChapterEntry ch0;
                                        if (StorageManager::read_v2_chapter_entry(target_path, current_chapter_idx, &ch0)) {
                                            current_text_file_offset = ch0.offset;
                                        }
                                    }
                                }
                            }

                            current_ui_state = UI_STATE_READING;
                            uint32_t mv = read_battery_millivolts();
                            render_current_reading_page(mv, calculate_battery_percentage(mv), /*fast_bw=*/false);
                        }
                    }
                    break;
                }

                case UI_STATE_BLE_SYNC: {
                    if (evt == EVENT_PREV || evt == EVENT_MENU_TAP) {
                        Serial.println("\n[BLE] User cancelled BLE Sync mode. Returning to library.");
                        stop_ble_sync_mode(true);
                    }
                    break;
                }

                case UI_STATE_JUMP: {
                    ButtonEvent dec_evt = (current_rotation == ORIENT_PORTRAIT) ? EVENT_NEXT : EVENT_PREV;
                    ButtonEvent inc_evt = (current_rotation == ORIENT_PORTRAIT) ? EVENT_PREV : EVENT_NEXT;

                    if (evt == dec_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == dec_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        jump_target_frame = max(jump_target_frame - step, 0);
                        Serial.printf("\n[JUMP] Target (-%d via %s) -> Page %d\n",
                                      step, (dec_evt == EVENT_PREV ? "D4" : "D5"), jump_target_frame + 1);
                        jump_dirty = true;
                        jump_dirty_time = millis();
                    } else if (evt == inc_evt) {
                        int step = 1;
                        ButtonEvent peek_evt;
                        while (xQueuePeek(inputQueue, &peek_evt, 0) == pdTRUE && peek_evt == inc_evt) {
                            xQueueReceive(inputQueue, &peek_evt, 0);
                            step++;
                        }
                        jump_target_frame = min(jump_target_frame + step, total_frames - 1);
                        Serial.printf("\n[JUMP] Target (+%d via %s) -> Page %d\n",
                                      step, (inc_evt == EVENT_PREV ? "D4" : "D5"), jump_target_frame + 1);
                        jump_dirty = true;
                        jump_dirty_time = millis();
                    } else if (evt == EVENT_MENU_TAP) {
                        Serial.printf("\n[JUMP] Confirmed Jump to Page %d\n", jump_target_frame + 1);
                        jump_dirty = false;
                        current_frame = jump_target_frame;
                        ConfigManager::set_current_frame(current_frame, true); // immediate save
                        current_ui_state = UI_STATE_READING;
                        uint32_t mv = read_battery_millivolts();
                        render_current_reading_page(mv, calculate_battery_percentage(mv));
                    }
                    break;
                }

                case UI_STATE_POWER_CONFIRM: {
                    ButtonEvent cancel_evt = (current_rotation == ORIENT_PORTRAIT) ? EVENT_NEXT : EVENT_PREV;
                    if (evt == cancel_evt) {
                        Serial.printf("\n[POWER] %s pressed -> Cancel and return to reading\n",
                                      (cancel_evt == EVENT_PREV ? "D4" : "D5"));
                        current_ui_state = UI_STATE_READING;
                        uint32_t mv = read_battery_millivolts();
                        render_current_reading_page(mv, calculate_battery_percentage(mv));
                    } else if (evt == EVENT_MENU_TAP) {
                        Serial.println("\n[POWER] D9 confirmed -> ENTERING STAGE 2 SLEEP!");
                        ConfigManager::commit_immediate(); // Flush state before sleeping
                        uint32_t mv = read_battery_millivolts();
                        enter_stage2_deep_sleep(mv, calculate_battery_percentage(mv));
                    }
                    break;
                }
            }
        }
    }

    // =========================================================================
    // DEBOUNCED UI RENDERING (COALESCES RAPID CLICKS)
    // =========================================================================
    // If in Settings Menu and dirty, render once the rapid clicks pause for 70ms
    if (current_ui_state == UI_STATE_MENU && menu_dirty) {
        if (millis() - menu_dirty_time >= 70) {
            menu_dirty = false;
            uint32_t mv = read_battery_millivolts();
            render_menu_screen(calculate_battery_percentage(mv));
        }
    }

    // If in Jump Page scrubber and dirty, render once the rapid clicks pause for 70ms
    if (current_ui_state == UI_STATE_JUMP && jump_dirty) {
        if (millis() - jump_dirty_time >= 70) {
            jump_dirty = false;
            render_jump_screen(jump_target_frame, total_frames);
        }
    }

    // =========================================================================
    // BLE WIRELESS BOOK SYNC BACKGROUND HANDLER
    // =========================================================================
    if (current_ui_state == UI_STATE_BLE_SYNC) {
        BleSyncState st = BleSyncManager::get().update();

        static BleSyncState prev_st = BLE_SYNC_INACTIVE;
        static uint32_t last_sync_refresh = 0;
        static uint32_t last_rx_count = 0;

        uint32_t cur_rx = BleSyncManager::get().get_bytes_received();
        bool progress_jump = (cur_rx > last_rx_count + 15000); // refresh display every ~15KB received

        if (st != prev_st || ((st == BLE_SYNC_RECEIVING || st == BLE_SYNC_OTA_RECEIVING) && progress_jump) || 
            (st == BLE_SYNC_ADVERTISING && (millis() - last_sync_refresh >= 10000))) {
            prev_st = st;
            last_sync_refresh = millis();
            last_rx_count = cur_rx;
            uint32_t mv = read_battery_millivolts();
            render_ble_sync_screen(calculate_battery_percentage(mv));
        }

        if (st == BLE_SYNC_TIMEOUT_EXPIRED) {
            Serial.println("[APP] BLE Sync 60s Timeout! Returning to Library.");
            prev_st = BLE_SYNC_INACTIVE;
            delay(1000);
            stop_ble_sync_mode(true);
        } else if (st == BLE_SYNC_SUCCESS) {
            if (BleSyncManager::get().is_ota_in_progress()) {
                Serial.println("[APP] BLE OTA Succeeded! Displaying reboot message and waiting for restart...");
                prev_st = BLE_SYNC_INACTIVE;
                uint32_t mv = read_battery_millivolts();
                render_ble_sync_screen(calculate_battery_percentage(mv));
                while (1) {
                    BleSyncManager::get().update(); // will trigger esp_restart()
                    delay(50);
                }
            }
            Serial.println("[APP] BLE Sync Succeeded! Switching to new book.");
            prev_st = BLE_SYNC_INACTIVE;
            delay(800);
            const char* new_path = BleSyncManager::get().get_target_file_path();
            if (strlen(new_path) > 0) {
                current_frame = ConfigManager::switch_book(new_path);
                strncpy(current_book_path, new_path, sizeof(current_book_path) - 1);
                current_book_path[sizeof(current_book_path) - 1] = '\0';

                // Re-scan library & update total_frames
                int book_count = StorageManager::get_stored_books(cached_books, MAX_STORED_BOOKS);
                for (int i = 0; i < book_count; i++) {
                    if (strcmp(cached_books[i].filename, current_book_path) == 0) {
                        total_frames = cached_books[i].total_frames;
                        break;
                    }
                }
                if (total_frames <= 0) total_frames = 4;
                current_frame = 0;
                page_stack_depth = 0;
                current_chapter_idx = 0;
                current_text_file_offset = 0;
                EbkFormatVersion v = StorageManager::get_book_version(new_path);
                if (v == EBK_FORMAT_V2_TEXT) {
                    Ebk2ChapterEntry ch0;
                    if (StorageManager::read_v2_chapter_entry(new_path, 0, &ch0)) {
                        current_text_file_offset = ch0.offset;
                    }
                }
            }
            stop_ble_sync_mode(false);
            current_ui_state = UI_STATE_READING;
            uint32_t mv = read_battery_millivolts();
            render_current_reading_page(mv, calculate_battery_percentage(mv), /*fast_bw=*/false);
        }
    }

    // Auto-commit any pending reading progress to NVS flash after 1s settle debounce
    ConfigManager::commit_if_due();

    // =========================================================================
    // INACTIVITY TIMEOUTS
    // =========================================================================
    // Menu & Power confirmation & Library auto-close after 15 seconds of inactivity
    // (Exempt UI_STATE_BLE_SYNC, which runs its own 60-second hardware timer)
    if (current_ui_state != UI_STATE_READING && current_ui_state != UI_STATE_BLE_SYNC && 
        (millis() - menu_active_timer >= MENU_TIMEOUT_MS)) {
        Serial.println("\n[MENU] Menu timed out. Returning to reading mode...");
        menu_dirty = false;
        jump_dirty = false;
        current_ui_state = UI_STATE_READING;
        uint32_t mv = read_battery_millivolts();
        render_current_reading_page(mv, calculate_battery_percentage(mv));
        awake_start = millis();
    }

    // Reading timeout: auto-enter Stage 2 deep sleep (randomized Zzz screensaver) after 1h
    if (millis() - awake_start >= AWAKE_TIMEOUT_MS) {
        Serial.println("\n[APP] Reading timeout reached! Auto-transitioning to STAGE 2 (Zzz Screensaver)...");
        uint32_t mv = read_battery_millivolts();
        enter_stage2_deep_sleep(mv, calculate_battery_percentage(mv));
        awake_start = millis();
    }

    delay(10);
}
