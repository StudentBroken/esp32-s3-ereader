# ESP32-S3 Ultra-Low-Power E-Paper Reader

An ultra-low-power, instant-wake e-reader based on the **Seeed Studio XIAO ESP32-S3** and a salvaged **4.2" Electronic Shelf Label (ESL) display** (HINK-E042A13-A0 with SSD1619 controller, 400x300 pixels).

---

## Hardware Pinout Reference

### 1. E-Paper Display SPI & Control
| Panel Function | XIAO Pin | ESP32-S3 GPIO | Note |
|---|---|---|---|
| **BUSY** | **D3** | GPIO 4 | Input (HIGH = Busy refreshing) |
| **RST** | **D2** | GPIO 3 | Output (Active LOW hardware reset) |
| **DC** | **D6** | GPIO 43 | Output (LOW = Command, HIGH = Data) |
| **CS** | **D0** | GPIO 1 | Output (Active LOW SPI chip select) |
| **SCLK** | **D8** | GPIO 7 | Hardware SPI Clock |
| **SDIN / MOSI** | **D10** | GPIO 9 | Hardware SPI Data |
| **VDD / VDDIO** | **3V3** | 3.3V | Logic and panel power |
| **VSS** | **GND** | GND | Common ground |

### 2. Battery Monitoring (100k + 100k with Switched Ground)
| Circuit Connection | XIAO Pin | ESP32-S3 GPIO | Note |
|---|---|---|---|
| **BAT Divider Out** | **D1** | GPIO 2 | ADC1 Channel 1 (Reads $V_{bat} / 2$) |
| **Divider Top (R1)**| **BAT+** | 3.0V - 4.2V | Top of 100k resistor to LiPo BAT+ |
| **Divider Bottom (R2)**| **D7** | GPIO 44 | Switched Ground (High-Z in sleep $\implies 0\ \mu\text{A}$ drain) |
| **Filter Cap (C1)** | Across R2| D1 to D7 | 10nF ceramic cap (smooths ADC sampling) |

> **Zero Quiescent Drain (Option A + B):** By using 100k + 100k resistors and anchoring the bottom of R2 to `D7` (`GPIO 44`), the ground path is disconnected during deep sleep, resulting in **true $0\ \mu\text{A}$ standby drain**. During measurement on page flip, `D7` is pulled LOW for 800 µs to read the ADC, then returned to high-impedance `INPUT`.

### 3. Navigation Buttons (Active LOW, Wake via EXT1)
| Button Function | XIAO Pin | ESP32-S3 GPIO | RTC GPIO? |
|---|---|---|---|
| **Button 1 (Previous Page)** | **D4** | GPIO 5 | Yes (RTC IO 5) |
| **Button 2 (Next Page)** | **D5** | GPIO 6 | Yes (RTC IO 6) |
| **Button 3 (Menu / Select)** | **D9** (or D8 if custom) | GPIO 8 | Yes (RTC IO 8) |

All buttons connect to ground with internal pull-ups enabled, triggering deep-sleep wakeups via `esp_sleep_enable_ext1_wakeup()` with `ESP_EXT1_WAKEUP_ANY_LOW`.

---

## Flash Memory Layout (8MB Flash)

Configured via `partitions.csv`:
* **NVS Non-Volatile Storage (`nvs`):** 20 KB (`0x5000`) for wear-leveled persistent state, active book, per-book bookmarks, settings, and schema versioning
* **Firmware App (`app0`):** 1.5 MB (`0x180000`)
* **LittleFS Storage (`spiffs`):** ~6.4 MB (`0x660000`) for direct binary `.ebk` storage (~400+ frames per megabyte)

---

## Project Structure

├── hardware.md           # Detailed Hardware Architecture & Technical Reference
├── ux.md                 # UX Specification & State Machine Wireframes
├── include/
│   ├── hardware_config.h     # Pinout definitions, EXT1 wake masks, sleep helpers
│   ├── ebk_format.h          # Packed binary container specification
│   ├── storage_manager.h     # LittleFS mounting, metadata parsing, and frame reading
│   └── config_manager.h      # Future-proof wear-leveled NVS persistent state manager
├── src/
│   ├── main.cpp              # Dual-core touch task, UI state machine & lifecycle
│   ├── config_manager.cpp    # Persistent config storage instances
│   └── GxEPD2_420_Z96.cpp    # Custom fast 1.06s partial LUT EPD driver
├── tools/
│   ├── ebk_converter.py      # Python script to convert text/PDF to 400x300 .ebk
│   ├── sample_book.txt       # Sample document for verification
│   └── web_serial/
│       └── index.html        # Web Serial file uploader & terminal interface
├── partitions.csv            # Custom 8MB flash partition table
└── platformio.ini            # PlatformIO project configuration
```

---

## Building and Flashing

### Compile Firmware
```bash
pio run
```

### Upload to XIAO ESP32-S3
Connect your XIAO ESP32-S3 via USB-C and run:
```bash
pio run -t upload
```

### Monitor Serial Output
```bash
pio device monitor -b 115200
```

---

## Converting Books to `.ebk`

Run the included converter tool:
```bash
python3 tools/ebk_converter.py tools/sample_book.txt -t "The Time Machine"
```
This generates `tools/sample_book.ebk` packed with 400x300 1-bit frames and instant-seek index tables.

---

## Wireless Web Bluetooth & Web Serial Studio

Open [`tools/web_serial/index.html`](file:///Users/xumic/Developer/esp32-s3-ereader/tools/web_serial/index.html) in Google Chrome, Microsoft Edge, Opera, or Bluefy (iOS) to upload `.ebk` books directly to the reader's 6.4 MB LittleFS flash partition.

### Option 1: Wireless Bluetooth Low Energy (BLE)
1. On the e-reader, tap **D9 (Menu)** while reading and select **Book Library / Switch**.
2. Scroll to the bottom and select **`[+] Add More Books (BLE)`**. The reader starts a **60-second BLE advertising window** and displays the countdown on the E-ink screen.
3. On the web page, click **Connect E-Reader via BLE** and choose `ESP32-Ereader`.
4. **Device Library & Storage Manager**:
   - The web page immediately retrieves and displays your e-reader's LittleFS flash storage usage bar (used MB vs total MB).
   - Lists all stored books with titles, filenames, sizes, and current reading progress (page X of Y).
   - **Read**: Switch the e-reader to open and read any book immediately.
   - **Rename**: Rename `.ebk` files stored on the device with automatic bookmark tracking.
   - **Delete**: Free up flash storage by deleting books with one click.
5. **Upload Books**: Drag and drop your `.ebk` file into the upload dropzone and click **Send Book via BLE**. The file is streamed with flow-controlled acknowledgment, saved to LittleFS, and the library view refreshes automatically.
6. **Wireless Firmware OTA Update**:
   - Drag and drop `.pio/build/seeed_xiao_esp32s3/firmware.bin` into the **Wireless Firmware OTA Update** card.
   - Click **Flash Firmware via BLE**. In ~10 seconds (~65 KB/s), the e-reader writes the image to the inactive OTA partition, verifies checksum integrity, and reboots automatically into the new build—all over Bluetooth without touching a USB cable!
7. If no connection is made within 60 seconds (or if cancelled with **D4**), the BLE radio shuts down completely to conserve power.

### Option 2: High-Speed USB Serial
Connect the XIAO ESP32-S3 via USB-C, switch to the **Web Serial (USB-C)** tab on the website, click **Connect USB Serial Port**, and stream books over USB.

## Colour screensaver (WiFi)

- Images: 300x400 portrait, white/black/red, 2 bits per pixel (SSI v1, see `include/screensaver.h`), stored in a Google Sheet as CRC-protected base64 chunks with a parity chunk.
- Prepare them with `screensaver.html` in the Epub tool, import the CSV into a sheet tab `Images`, deploy `tools/screensaver/apps_script.gs`.
- Set WiFi (4 slots) + web app URL (`...exec?k=SECRET`) in the Epub tool's `ble.html` (device menu: Library -> Add More Books / BLE).
- Flow: after sleep, Zzz shows for 30 s (touch cancels), then the next cached image is drawn with a full tri-colour refresh. Every 12 h the device wakes: weekly it downloads 14 random images (retries on the next 12 h wake, max 4 attempts), every 24 h it shows the next cached image. Only runs above 25% battery. Toggle and "Screensaver Now" are in the settings menu.
- Idle auto-sleep is 5 minutes. Library: tap a book for Read / Move up / Move down / Delete. "Jump to Chapter" opens a scrollable table of contents.
