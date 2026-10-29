#pragma once

#include <Arduino.h>
#include <esp_sleep.h>

/**
 * Seeed Studio XIAO ESP32-S3 Pin Mapping Reference:
 * 
 * Silkscreen | ESP32-S3 GPIO | RTC GPIO? | Function in E-Reader
 * --------------------------------------------------------------------------
 * D0         | GPIO 1        | Yes (RTC1)| EPD Chip Select (CS)
 * D1         | GPIO 2        | Yes (RTC2)| Battery ADC Input (10k+10k divider)
 * D2         | GPIO 3        | Yes (RTC3)| Available / Aux GPIO
 * D3         | GPIO 4        | Yes (RTC4)| EPD Busy Indicator (BUSY)
 * D4         | GPIO 5        | Yes (RTC5)| Button 1: Previous Page (EXT1 Wake)
 * D5         | GPIO 6        | Yes (RTC6)| Button 2: Next Page (EXT1 Wake)
 * D6         | GPIO 43       | NO        | EPD Data/Command (DC)
 * D7         | GPIO 44       | NO        | EPD Reset (RST) or Switched GND
 * D8         | GPIO 7        | Yes (RTC7)| Hardware SPI Clock (SCLK)
 * D9         | GPIO 8        | Yes (RTC8)| Button 3: OK / Menu (EXT1 Wake)
 * D10        | GPIO 9        | Yes (RTC9)| Hardware SPI MOSI (SDIN)
 */

// ==============================================================================
// 1. E-Paper Display Pins (HINK-E042A13-A0 / SSD1619, 400x300 SPI)
// ==============================================================================
#define EPD_CS_PIN      D0   // Pin 12 (CS) -> D0 (GPIO 1)
#define EPD_DC_PIN      D6   // Pin 11 (DC) -> D6 (GPIO 43)
#define EPD_RST_PIN     D2   // Pin 10 (RST) -> D2 (GPIO 3)
#define EPD_BUSY_PIN    D3   // Pin 9 (BUSY) -> D3
#define EPD_SCLK_PIN    D8   // Pin 13 (SCLK) -> D8
#define EPD_MOSI_PIN    D10  // Pin 14 (SDIN/MOSI) -> D10
#define EPD_MISO_PIN   -1   // Not used for E-Paper write-only SPI

// Display Geometry
#define EPD_WIDTH     400
#define EPD_HEIGHT    300

// Screen Orientation:
// ORIENT_LANDSCAPE: default 400x300 (rotation 0)
// ORIENT_PORTRAIT:  300x400 rotated 90° CW / right (rotation 1)
enum ScreenOrientation {
    ORIENT_LANDSCAPE = 0,
    ORIENT_PORTRAIT  = 1
};

// ==============================================================================
// 2. Navigation Buttons (Active LOW with internal pull-ups)
// ==============================================================================
#define BTN_PREV_PIN    5   // D4 / GPIO 5 (Previous Page, RTC-capable)
#define BTN_NEXT_PIN    6   // D5 / GPIO 6 (Next Page, RTC-capable)
#define BTN_MENU_PIN    8   // D9 / GPIO 8 (Menu / OK, RTC-capable)

// EXT1 Bitmask for deep-sleep wake (GPIOs 5, 6, 8)
#define EXT1_WAKE_BITMASK ((1ULL << BTN_PREV_PIN) | (1ULL << BTN_NEXT_PIN) | (1ULL << BTN_MENU_PIN))

// ==============================================================================
// 3. Battery Voltage Divider (100k + 100k with Switched Ground on D7)
// ==============================================================================
#define BAT_ADC_PIN         2   // D1 / GPIO 2 (ADC1 Channel 1)
#define BAT_GND_SWITCH_PIN 44   // D7 / GPIO 44 (Switched GND: High-Z in sleep, LOW during read)
#define BAT_DIVIDER_RATIO   2.0f // Equal resistors (100k+100k or 470k+470k) divide voltage by 2

/**
 * Reads battery voltage in millivolts with switched-ground power gating.
 * Stabilised: 7 spaced bursts of 16 samples, out-of-range bursts (ADC glitches such as 6 V readings)
 * are discarded, the median of the rest is taken and smoothed with an EMA kept in RTC memory.
 * Achieves true 0 uA quiescent current drain during deep sleep!
 */
RTC_DATA_ATTR static uint32_t bat_filtered_mv = 0;

inline uint32_t read_battery_millivolts() {
    pinMode(BAT_GND_SWITCH_PIN, OUTPUT);
    digitalWrite(BAT_GND_SWITCH_PIN, LOW);
    delayMicroseconds(800);   // RC settling: 100k||100k (50k) with 10nF cap -> tau = 0.5ms

    uint32_t bursts[7];
    int n = 0;
    for (int b = 0; b < 7; b++) {
        uint32_t sum = 0;
        for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(BAT_ADC_PIN);
        uint32_t mv = (uint32_t)((sum / 16) * BAT_DIVIDER_RATIO);
        if (mv >= 2500 && mv <= 4600) bursts[n++] = mv;   // plausible single-cell LiPo range only
        delay(2);
    }
    pinMode(BAT_GND_SWITCH_PIN, INPUT);   // cut off all current flow

    if (n == 0) return bat_filtered_mv;   // every burst was a glitch: keep the last good value (0 if none)
    for (int i = 1; i < n; i++) {         // insertion sort, median
        uint32_t v = bursts[i]; int j = i - 1;
        while (j >= 0 && bursts[j] > v) { bursts[j + 1] = bursts[j]; j--; }
        bursts[j + 1] = v;
    }
    uint32_t med = bursts[n / 2];
    if (bat_filtered_mv == 0 || (med > bat_filtered_mv ? med - bat_filtered_mv : bat_filtered_mv - med) > 250) {
        bat_filtered_mv = med;   // first reading or genuine step (USB plugged in / out): snap
    } else {
        bat_filtered_mv = (bat_filtered_mv * 7 + med * 3 + 5) / 10;   // EMA, alpha 0.3
    }
    return bat_filtered_mv;
}

/**
 * Maps LiPo millivolts to percentage (0 - 100%) using a realistic discharge curve.
 */
inline uint8_t calculate_battery_percentage(uint32_t mv) {
    if (mv >= 4150) return 100;
    if (mv >= 4000) return 80 + ((mv - 4000) * 20) / 150;
    if (mv >= 3850) return 60 + ((mv - 3850) * 20) / 150;
    if (mv >= 3750) return 40 + ((mv - 3750) * 20) / 100;
    if (mv >= 3650) return 20 + ((mv - 3650) * 20) / 100;
    if (mv >= 3500) return 10 + ((mv - 3500) * 10) / 150;
    if (mv >= 3350) return ((mv - 3350) * 10) / 150;
    return 0;
}

// ==============================================================================
// 4. Power Management Helpers
// ==============================================================================
inline void configure_button_wake() {
    pinMode(BTN_PREV_PIN, INPUT_PULLUP);
    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(BTN_MENU_PIN, INPUT_PULLUP);

    // Arm EXT1 wake: triggers when ANY selected pin is pulled LOW (button press to GND)
    esp_sleep_enable_ext1_wakeup(EXT1_WAKE_BITMASK, ESP_EXT1_WAKEUP_ANY_LOW);
}
