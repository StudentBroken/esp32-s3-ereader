# Hardware Architecture & Technical Reference
## ESP32-S3 Ultra-Low-Power E-Paper Reader System

---

## 1. System Overview

This system is a custom, ultra-low-power electronic paper reader built upon the **Seeed Studio XIAO ESP32-S3** and a repurposed **4.2" Electronic Shelf Label (ESL) display** (HINK-E042A13-A0 with Solomon Systech SSD1619 display controller).

The architecture is engineered around three primary design imperatives:
1. **Sub-second page turn performance** on an e-paper panel traditionally requiring 9–15 seconds for full refresh cycles.
2. **Zero-dropped-input concurrency**, using dual-core FreeRTOS task partitioning so user touch input is never blocked by display refresh latency.
3. **Multi-month battery endurance** through staged power management, switched-ground battery telemetry ($0\ \mu\text{A}$ quiescent draw), and RTC deep-sleep modes (~$20\text{--}30\ \mu\text{A}$ standby).

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      SEEED STUDIO XIAO ESP32-S3                        │
 │                                                                        │
 │   ┌───────────────────────────┐     ┌──────────────────────────────┐   │
 │   │          CORE 0           │     │            CORE 1            │   │
 │   │  touch_task (~66 Hz)      │     │  Arduino setup() / loop()    │   │
 │   │  - Hardware FSM Sampling  │     │  - UI State Machine          │   │
 │   │  - Common-Mode Noise Filter     │  - Storage / .ebk Decoding   │   │
 │   │  - 350ms Blanking Window  │     │  - Fast Differential EPD SPI │   │
 │   │  - Baseline Adaptation    │     │  - Battery Telemetry (ADC)   │   │
 │   └─────────────┬─────────────┘     └──────────────┬───────────────┘   │
 │                 │                                  │                   │
 │                 └──────── FreeRTOS inputQueue ─────┘                   │
 └─────────────────┬──────────────────────────────────┬───────────────────┘
                   │                                  │
         Capacitive Touch Pads                 FSPI 4-Wire Bus
      ┌────────────┼────────────┐          ┌──────────┼──────────┐
      ▼            ▼            ▼          ▼          ▼          ▼
   Pad D4       Pad D5       Pad D9       MOSI       SCLK        CS
  (GPIO 5)     (GPIO 6)     (GPIO 8)    (GPIO 9)   (GPIO 7)   (GPIO 1)
   [PREV]       [NEXT]    [MENU/PWR]
                                           ┌──────────┼──────────┐
                                           ▼          ▼          ▼
                                          DC         RST        BUSY
                                       (GPIO 43)  (GPIO 44)   (GPIO 4)
                                        [Data/Cmd]   [Reset]   [Hardware]
```

---

## 2. Complete Hardware Pinout & Bus Mapping

| Function | XIAO Pin | ESP32-S3 GPIO | Bus / Peripheral | Electrical Characteristics | Purpose |
|---|---|---|---|---|---|
| **EPD CS** | **D0** | `GPIO 1` | FSPI / CS0 | Output, Active LOW | Display SPI Chip Select |
| **BAT ADC** | **D1** | `GPIO 2` | ADC1 Channel 1 | Analog Input ($0\text{--}3.1\text{V}$) | LiPo Voltage Divider ($V_{bat}/2$) |
| **EPD BUSY**| **D3** | `GPIO 4` | GPIO Input | Digital Input (HIGH = Busy) | Hardware Refresh Status |
| **TOUCH D4**| **D4** | `GPIO 5` | Touch Pad 5 / RTC IO 5 | Capacitive Touch Sensor | Previous Page / Up / Back |
| **TOUCH D5**| **D5** | `GPIO 6` | Touch Pad 6 / RTC IO 6 | Capacitive Touch Sensor | Next Page / Down |
| **EPD DC**  | **D6** | `GPIO 43` | GPIO Output | Digital Output (L=Cmd, H=Data)| EPD Data / Command Selector |
| **BAT GND SW** | **D7**| `GPIO 44` | GPIO Open-Drain | Digital Output (L=Measure, Z=Sleep)| Battery Divider Ground Switch |
| **EPD SCLK**| **D8** | `GPIO 7` | FSPI / SCK | 4 MHz Hardware SPI Clock | EPD Serial Clock |
| **TOUCH D9**| **D9** | `GPIO 8` | Touch Pad 8 / RTC IO 8 | Capacitive Touch / Deep Wake | Menu Tap / Hold Power / Wake |
| **EPD MOSI**| **D10**| `GPIO 9` | FSPI / MOSI | Hardware SPI Master-Out | EPD Serial Data In (SDIN) |
| **EPD RST** | **D2** | `GPIO 3` | GPIO Output | Digital Output, Active LOW | Display Hardware Reset |
| **3V3 Rail**| **3V3**| — | Power Rail | 3.3V Output (Onboard LDO) | Logic & E-Paper VDD/VDDIO |
| **GND**     | **GND**| — | Power Ground | Common System Ground | Reference Ground |

---

## 3. E-Paper Display Controller: Solomon Systech SSD1619

### 3.1 Panel Specifications
* **Active Area:** $4.2\text{ inches}$ diagonal ($84.8\text{ mm} \times 63.6\text{ mm}$)
* **Resolution:** $400 \times 300\text{ pixels}$
* **Pixel Density:** $\approx 120\text{ DPI}$
* **Native Color Mode:** 1-bit Monochrome (Black / White)
* **Internal SRAM Frame Buffers:**
  * RAM 1 (`0x24`): Current target image buffer
  * RAM 2 (`0x26`): Previous/old image reference buffer (used by hardware for differential driving)

### 3.2 SPI Communication Protocol
The display utilizes 4-wire SPI (Clock, Data, Chip Select, Data/Command) driven via the ESP32-S3 hardware FSPI peripheral:
* **SPI Mode:** Mode 0 (`CPOL = 0, CPHA = 0`)
* **Bit Order:** MSB First
* **Clock Frequency:** $4.0\text{ MHz}$
* **Command Transmission:** `DC` driven `LOW`, `CS` driven `LOW`, write byte, `CS` driven `HIGH`.
* **Data Transmission:** `DC` driven `HIGH`, `CS` driven `LOW`, burst bytes, `CS` driven `HIGH`.

### 3.3 Factory Waveform vs. Custom Differential Waveform (LUT)
Stock electronic shelf label firmware executes full anti-ghosting cycles lasting **9 to 15 seconds** because ESL tags flip once or twice per day and prioritize zero ghosting over responsiveness. For an interactive e-reader, this makes navigation intolerable.

#### Fast B/W Differential Refresh Mode (~1.06 Seconds)
To achieve sub-second updates, the display driver is configured with a custom Look-Up Table (LUT) that skips intermediate inversion frames:
1. **Target Buffers:** The new monochrome bitmap is written to RAM `0x24` and the previous screen bitmap is mirrored in RAM `0x26`.
2. **Voltage Level Modulation:** Gate and Source voltage pulses drive only pixels whose state has changed ($0 \rightarrow 1$ or $1 \rightarrow 0$). Static pixels receive $0\text{V}$ differential, leaving their micro-capsule particles stationary.
3. **Register Sequence:**
   * `0x2C`: VCOM Voltage register $\rightarrow$ programmed to `0x26`.
   * `0x32`: 70-byte custom partial LUT written to hardware register:
     ```c
     // Custom 1.06s Differential LUT (Black / White transitions only)
     0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
     0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
     0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
     0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
     0x0A,0x00,0x00,0x00,0x00,0x00,0x02,
     0x01,0x00,0x00,0x00,0x00,0x00,0x00,
     0x0A,0x00,0x00,0x00,0x00,0x00,0x02,
     0x00,0x00,0x00,0x00,0x00,0x00,0x00
     ```
   * `0x22`: Display Update Control $\rightarrow$ set to `0x04` (Fast partial update sequence).
   * `0x20`: Master Activation command.
   * `BUSY` pin monitored: hardware signals completion when line drops `LOW` (measured duration: **$1054\text{--}1065\text{ ms}$**).

#### Full Clean Anti-Ghost Mode (~2.64 Seconds)
After every 8 fast differential page turns, or upon user request from the Settings Menu, a full clearing refresh executes:
1. `0x32`: Full-panel inversion LUT applied.
2. `0x22`: Display Update Control $\rightarrow$ set to `0xC4` (Full update sequence).
3. The panel undergoes micro-capsule depolarization to completely eliminate ghosting shadows from previous frames (measured duration: **$2638\text{ ms}$**).

---

## 4. Capacitive Touch Subsystem & Electrical Interference

### 4.1 Touch Peripheral Architecture (ESP32-S3 Touch V2)
The ESP32-S3 incorporates an analog-to-digital capacitive charge/discharge measurement engine:
* **Measurement Principle:** An internal current source charges and discharges the pad's capacitance over a fixed number of clock cycles (`__touchMeasureCycles`, `__touchSleepCycles`). A human finger increases the pad's capacitance to ground, altering the oscillator charge time and generating a higher numeric raw count.
* **Resting Baseline Counts (Untouched):**
  * `D4` (`GPIO 5`): $\approx 21,800\text{--}25,700\text{ counts}$
  * `D5` (`GPIO 6`): $\approx 26,300\text{--}33,300\text{ counts}$
  * `D9` (`GPIO 8`): $\approx 25,300\text{--}31,600\text{ counts}$
* **Finger Contact Counts (Touched):**
  * Raw values increase by $+1,500$ to $+4,300$ counts above baseline.

### 4.2 The Electrical Crosstalk & Ghost Touch Problem

During hardware validation, three distinct electrical anomalies were discovered:

#### Anomaly 1: SPI High-Frequency Bus Crosstalk
When the ESP32-S3 shifts $15,000\text{ bytes}$ of frame data to the panel over 4 MHz SPI, rapid voltage transitions on `MOSI` (`GPIO 9`) and `SCLK` (`GPIO 7`) capacitively couple directly into adjacent silicon and PCB traces for `D9` (`GPIO 8`) and `D5` (`GPIO 6`).
* **Symptom:** Raw delta spikes exceeding $+6,000$ to $+12,000$ counts during SPI bursts.
* **Solution:** Global volatile flag `epd_spi_active` set in the driver. The Core 0 `touch_task` detects `epd_spi_active == true` and yields execution (`vTaskDelay(15)`), eliminating false readings during SPI data transfers.

#### Anomaly 2: High-Voltage Charge Pump Bleed-Down (Post-Refresh Ghost Touches)
The SSD1619 panel integrates an on-glass DC-DC boost converter generating $\pm 15\text{V}$ gate/source driving voltages. When the display controller finishes a frame and lowers the `BUSY` pin, its high-voltage capacitors take **$200\text{--}300\text{ ms}$** to bleed down through the ground plane.
* **Symptom:** The instant `BUSY` dropped to `LOW`, all three touch pads spiked simultaneously:
  ```
  D4: 26,269 (delta +4,389) [X]
  D5: 32,728 (delta +6,384) [X]
  D9: 31,992 (delta +6,647) [X]
  ```
  This triggered false "ghost" page turns and unintended menu selections while hands were nowhere near the device.
* **Solution:**
  1. **350ms Blanking Window:** When `BUSY` transitions from `HIGH` to `LOW`, `touch_task` records `last_busy_end = millis()` and blanks all touch evaluation for **$350\text{ ms}$**, allowing power rails to discharge to zero before sampling resumes.
  2. **Common-Mode Noise Rejection:** A human finger only presses one pad at a time. If 2 or more pads exceed trigger threshold simultaneously (`active_count > 1`), the system rejects all of them as common-mode rail noise.
  3. **40ms Sustained Press Debouncing:** Transient electrical spikes only persist for $10\text{--}20\text{ ms}$. Genuine human contact requires continuous contact for $\ge 40\text{ ms}$.

#### Anomaly 3: Ambient Grip Shift & Baseline Trapping
When a user picks up the plastic e-reader case or changes hand position, grounding shifts cause resting capacitance to drift upward by $+500\text{--}1,400\text{ counts}$.
* **The Trap:** If a sensor baseline is locked at $31,603$ and resting raw capacitance drifts to $33,050$, delta is $+1,447$. If trigger threshold is $500$, the system believes the button is being held indefinitely. Because baseline adaptation was traditionally disabled while `is_touched == true`, the sensor was permanently locked in a pressed state.
* **Solution: 3.5-Second Stuck-Key Watchdog:**
  If any pad remains continuously in the `is_touched == true` state for $> 3.5\text{ seconds}$ (longer than the 2-second long press), the system identifies the state as grip displacement rather than a human press, snaps the baseline directly to the current raw value, and forces an immediate release.

---

## 5. FreeRTOS Dual-Core Architecture

To ensure zero dropped button clicks and instantaneous responsiveness while the e-paper panel performs its 1.06-second partial updates, the firmware partitions responsibilities across both ESP32-S3 Xtensa cores:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        CORE 0: TOUCH SCANNER TASK                      │
│                                                                        │
│   Cycle Period: 15 ms (~66 Hz sampling)                                │
│   Stack Depth:  4096 bytes                                             │
│                                                                        │
│   1. Check SPI Activity: If epd_spi_active == true -> Yield           │
│   2. Check Busy Pin: If BUSY == HIGH -> Update last_busy_end, Yield    │
│   3. Check Blanking: If (millis() - last_busy_end < 350) -> Yield      │
│   4. Sample Sensors: Read D4, D5, D9 with range check (1000..60000)    │
│   5. Common-Mode Filter: If (active_pads > 1) -> Discard all           │
│   6. Debounce Verification: Require >= 40 ms sustained touch           │
│   7. Event Dispatch: Push ButtonEvent to inputQueue                    │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │
                      FreeRTOS Queue (inputQueue)
                        Length: 32 ButtonEvent
                                   │
┌──────────────────────────────────▼─────────────────────────────────────┐
│                    CORE 1: ARDUINO MAIN LOOP & UI                      │
│                                                                        │
│   1. Queue Read: xQueueReceive(inputQueue, &evt, 0)                    │
│   2. Event Processing: State machine (READING, MENU, JUMP, POWER)      │
│   3. Click Coalescing: Peek queue to sum rapid multiple clicks         │
│   4. Display Driver: Push bitmap over SPI and monitor EPD BUSY         │
│   5. Inactivity Watchdogs: Auto-close menu (15s), Sleep (1h)           │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Deep Sleep Lifecycle & Wakeup State Machine

The firmware implements a 2-stage power lifecycle to deliver instant readiness during reading sessions and micro-amp conservation during prolonged storage:

```
                      ┌────────────────────────────┐
                      │    POWER ON / COLD BOOT    │
                      └──────────────┬─────────────┘
                                     ▼
                      ┌────────────────────────────┐
                      │    STAGE 1: ACTIVE READ    │◄─────────────────┐
                      │  Awake time: 1 hour timer  │                  │
                      │  Fast 1.06s partial flips  │                  │
                      │  Current: ~15 mA active    │                  │
                      └──────┬──────────────┬──────┘                  │
                             │              │                         │
            Inactivity (1h)  │              │  Hold D9 (2s)           │
                             ▼              ▼                         │
                      ┌────────────────────────────┐                  │
                      │   POWER CONFIRM DIALOG     │                  │
                      │  D4/D5: Cancel  D9: Sleep  │                  │
                      └──────────────┬─────────────┘                  │
                                     │ Confirm (D9)                   │
                                     ▼                                │
                      ┌────────────────────────────┐                  │
                      │   STAGE 2: DEEP SLEEP      │                  │
                      │  Screen: 'Zzz' Screensaver │                  │
                      │  Wake Source: D9 Touch Pad │                  │
                      │  Current: ~20-30 uA        │                  │
                      └──────────────┬─────────────┘                  │
                                     │ Touch D9                       │
                                     └────────────────────────────────┘
```

### 6.1 The Deep Sleep Wakeup Corruption Bug & Resolution
* **The Root Cause:** When entering Stage 2, `touchSleepWakeUpEnable(TOUCH_MENU_PIN, 250)` programs RTC Touch Pad 8 into deep-sleep channel mode. Upon wake, the ESP32-S3 executes a reset vector into `setup()`. If `touchRead()` is called before resetting the peripheral, ESP-IDF's `touch_pad_init()` returns `ESP_ERR_INVALID_STATE` (fails because the sleep channel is still armed).
* **Stack Corruption:** In the Arduino ESP32 core, when `touch_pad_read_raw_data()` fails due to invalid driver state, it aborts without writing to the output pointer. The calling function reads uninitialized stack memory:
  ```
  [TOUCH] D4: 76775 (b:76775 d:+0) | D5: 132419 (b:132419 d:+0) | D9: 449720 (b:449720 d:+0)
  ```
  The calibration loop then locked these exact stack memory addresses as permanent baselines, freezing touch forever.
* **The Resolution:** At the absolute entry point of `setup()`, the driver executes a full teardown:
  ```cpp
  // Completely reset touch peripheral from any previous deep sleep wake state
  touch_pad_deinit();
  delay(20);
  touch_pad_sleep_channel_enable(TOUCH_PAD_MENU, false);
  touch_pad_sleep_channel_enable(TOUCH_PAD_PREV, false);
  touch_pad_sleep_channel_enable(TOUCH_PAD_NEXT, false);
  delay(10);
  // Re-warm Arduino driver and start hardware FSM cleanly
  touchRead(TOUCH_PREV_PIN);
  touchRead(TOUCH_NEXT_PIN);
  touchRead(TOUCH_MENU_PIN);
  ```

---

## 7. Display Orientation & Coordinate Mapping

The device supports runtime orientation toggling between **Landscape** ($400 \times 300$) and **Portrait 90° Clockwise** ($300 \times 400$).

### 7.1 EBK 1-Bit Bitmap Pixel Rotation Math
The binary `.ebk` e-book container stores frames formatted as standard 1-bit landscape bitmaps ($400\text{ px wide} \times 300\text{ px high}$, $15,000\text{ bytes}$). When in Portrait mode, each pixel $(x, y)$ is transformed on the fly without requiring separate book storage files:

$$\text{new\_x} = \text{old\_y}$$
$$\text{new\_y} = (\text{EBK\_FRAME\_WIDTH} - 1) - \text{old\_x} = 399 - \text{old\_x}$$

```cpp
void render_ebk_bitmap(const uint8_t* src, uint8_t rotation) {
    if (rotation == ORIENT_LANDSCAPE) {
        display.drawBitmap(0, 0, src, 400, 300, GxEPD_WHITE, GxEPD_BLACK);
    } else if (rotation == ORIENT_PORTRAIT) {
        for (int old_y = 0; old_y < 300; old_y++) {
            for (int old_x = 0; old_x < 400; old_x++) {
                int byte_idx = (old_y * 400 + old_x) / 8;
                int bit_idx = 7 - (old_x % 8);
                bool is_white = (src[byte_idx] >> bit_idx) & 1;
                int new_x = old_y;
                int new_y = 399 - old_x;
                display.drawPixel(new_x, new_y, is_white ? GxEPD_WHITE : GxEPD_BLACK);
            }
        }
    }
}
```

### 7.2 Button Inversion in Portrait Mode
Rotating the physical device 90° clockwise places the `D4` button in the lower position and `D5` in the upper position. To maintain natural spatial ergonomics in settings and navigation menus:
* **Landscape Mode:** `D4` = Up / Previous Page / Back; `D5` = Down / Next Page.
* **Portrait Mode:** `D5` = Up / Previous Page / Back; `D4` = Down / Next Page.

---

## 8. Battery Telemetry & Power Consumption

### 8.1 Zero-Quiescent Switched-Ground Divider
Connecting a traditional 100k + 100k resistor divider across a LiPo battery continuously draws:

$$I = \frac{4.2\text{V}}{200\text{ k}\Omega} = 21\ \mu\text{A}$$

Over months of standby, this parasitic drain alone would deplete the cell. The circuit resolves this by terminating the bottom of the divider ($R_2$) at `GPIO 44` (`D7`):

```
LiPo (+) ─── [ R1: 100k ] ───┬─── [ R2: 100k ] ─── GPIO 44 (D7: Switched Ground)
                             │
                             ├─── [ C1: 10nF ] ─── GND
                             │
                      GPIO 2 (D1: ADC1_CH1)
```

* **Deep Sleep Mode:** `GPIO 44` is placed in high-impedance `INPUT` mode. No current path to ground exists (**$0\ \mu\text{A}$ quiescent draw**).
* **Sampling Mode:** `GPIO 44` is pulled `OUTPUT LOW` for $800\ \mu\text{s}$, `analogReadMilliVolts(2)` acquires the voltage across $R_2$, and `GPIO 44` immediately returns to high-Z.

### 8.2 Power Consumption Breakdown
* **Active EPD Fast Refresh (~1.06s):** $\approx 18\text{--}24\text{ mA}$ average during charge pump operation.
* **Active Reading Standby (Awake with Core 0 Touch Task):** $\approx 15\text{ mA}$.
* **Stage 2 Deep Sleep (RTC Touch Pad Wake Source Enabled):** $\approx 22\text{--}28\ \mu\text{A}$.
* **Expected Battery Life (1000 mAh LiPo):**
  * Continuous reading (1 page turn per minute): $\approx 40\text{ hours}$ active reading time (~2,400 page turns).
  * Standby endurance: $\approx 12\text{--}18\text{ months}$ in Stage 2 deep sleep.
