# UX Specification & User Flow Architecture
## ESP32-S3 Ultra-Low-Power E-Paper Reader (3-Button Interface)

---

## 1. Executive Summary & Hardware Interface

This document defines the complete user experience, interaction architecture, edge-case handling, and battery monitoring system for a dedicated 3-button e-paper reader powered by the **Seeed Studio XIAO ESP32-S3** and a **4.2" E-Ink display ($400 \times 300$, SSD1619 controller)**.

### Hardware Interface & Pin Routing
* **Button 1 (`PREV` / Left):** `GPIO 5` (Silkscreen `D4`) — RTC EXT1 Wake
* **Button 2 (`NEXT` / Right):** `GPIO 6` (Silkscreen `D5`) — RTC EXT1 Wake
* **Button 3 (`OK` / Menu / Select):** `GPIO 8` (Silkscreen `D9`) — RTC EXT1 Wake
* **Battery ADC Input (10k+10k Divider):** `GPIO 2` (Silkscreen `D1` / `A1`) — ADC1 Channel 1
* **Display SPI & Control:**
  * `CS`: `GPIO 1` (Silkscreen `D0`)
  * `DC`: `GPIO 43` (Silkscreen `D6` — standard GPIO output)
  * `RST`: `GPIO 44` (Silkscreen `D7` — standard GPIO output)
  * `BUSY`: `GPIO 4` (Silkscreen `D3`)
  * `SCLK`: `GPIO 7` (Silkscreen `D8`)
  * `MOSI`: `GPIO 9` (Silkscreen `D10`)

> [!TIP]
> **Pin Routing Optimization:** By assigning display control lines `DC` and `RST` to silkscreen pins `D6` (`GPIO 43`) and `D7` (`GPIO 44`), silkscreen pin **`D1` (`GPIO 2`)** is freed for the **Battery ADC** input, while preserving `GPIO 5, 6, 8` for deep-sleep RTC wakeup!

---

## 2. Interaction State Machine Overview

```
                      [ STAGE 1: ACTIVE READING (1h Timeout) ]
                                    │
               ┌────────────────────┼────────────────────┐
               │ TOUCH (D4: PREV)   │ TOUCH (D5: NEXT)   │ TOUCH (D9: MENU/POWER)
               ▼                    ▼                    ▼
       [ Turn Page Back ]   [ Turn Page Fwd ]    [ Evaluate Touch Duration ]
               │                    │                    │
               │ (Fast 1.5s B/W)    │ (Fast 1.5s B/W)    │
               │ (Sample Battery)   │ (Sample Battery)   │
               └─────────┬──────────┘                    │
                         │                               │
                         ▼                               ├─ Short Tap (<1.5s) ───► [ QUICK SETTINGS MENU ]
                 [ Page Displayed ]                      │                          (Resume, Orientation, Jump, Sleep)
                 (Auto-sleep 1 hr)                       │
                                                         └─ Long Hold (≥1.5s) ───► [ STAGE 2 DEEP SLEEP (Zzz) ]
                                                                                    (Only D9 Touch Pad Wakes Up)
```

---

## 3. Screen States & Visual Wireframes ($400 \times 300$ Landscape / $300 \times 400$ Portrait)

### Orientation Modes & Coordinate Transformations:
* **Landscape Mode (`current_rotation = 0`):** $400 \times 300\text{ px}$. Standard wide horizontal layout.
* **Portrait Mode (`current_rotation = 3`):** $300 \times 400\text{ px}$. Spun **90° counter-clockwise (to the left)**:
  * The bottom edge of the landscape canvas becomes the **left margin** of the portrait layout.
  * The top edge of the landscape canvas becomes the **right margin** of the portrait layout.
  * The right edge of the landscape canvas becomes the **top header**.
  * The left edge of the landscape canvas becomes the **bottom footer**.
  * Gives a classic handheld paperback book aspect ratio with dynamic paragraph auto-wrapping.

### State A: Reading Mode (Standard Display with Clean Battery Bar)
Full-screen content rendered dynamically or from the `.ebk` 1-bit frame buffer, with an anchored battery indicator (percentage + icon, no raw millivolts clutter).

```
Landscape ($400 \times 300$):
+-------------------------------------------------------------------------+
| THE TIME MACHINE                    Page 1 of 4             [==== ] 82% |
|-------------------------------------------------------------------------|
| Chapter 1: The Time Traveller                                           |
|                                                                         |
| The Time Traveller was expounding a recondite matter to us. His grey    |
| eyes shone and twinkled, and his usually pale face was flushed and      |
| animated. The fire burned brightly, and the soft radiance of the        |
| incandescent lights caught the bubbles that passed in our glasses...    |
|                                                                         |
|-------------------------------------------------------------------------|
| [D4] Prev Page                  [D9] Menu                  [D5] Next Page
+-------------------------------------------------------------------------+

Portrait ($300 \times 400$ - 90° Left):
+-----------------------------------------------+
| THE TIME MACHINE                  [==== ] 82% |
|-----------------------------------------------|
| Chapter 1: The Time Traveller                 |
|                                               |
| The Time Traveller was expounding a           |
| recondite matter to us. His grey eyes         |
| shone and twinkled, and his usually pale      |
| face was flushed and animated.                |
|                                               |
| The fire burned brightly, and the soft        |
| radiance of the incandescent lights           |
| caught the bubbles that passed in our         |
| glasses...                                    |
|                                               |
|-----------------------------------------------|
| [D4] Prev        P.1/4       [D9]   [D5] Next |
+-----------------------------------------------+
```

* **`D4` (Tap):** Previous page (fast 1.5s B/W differential refresh).
* **`D5` (Tap):** Next page (fast 1.5s B/W differential refresh).
* **`D9` (Tap):** Opens Quick Settings & Power Menu.
* **HOLD ANY BUTTON $\ge$ 2.5s (D4, D5, OR D9):** Universal gesture to open the Settings/Power Menu from anywhere!

---

### State B: Quick Settings & Menu Modal
High-contrast drop-shadow card overlaid on the page. Inverted selection box indicates active option.

```
+-------------------------------------------------------------------------+
| SETTINGS & POWER MENU                                       [==== ] 82% |
|-------------------------------------------------------------------------|
|                                                                         |
|         +=============================================+                 |
|         | QUICK OPTIONS                               |                 |
|         |---------------------------------------------|                 |
|         |   [ Resume Reading ]                        |                 |
|         | > [ Orientation: Portrait 90* ]             |                 |
|         |   [ Jump Page (+/- 1) ]                     |                 |
|         |   [ Full Anti-Ghost Refresh ]               |                 |
|         |   [ Power Off / Sleep... ]                  |                 |
|         |---------------------------------------------|                 |
|         | [D4] Up        [D5] Down       [D9] Select  |                 |
|         +=============================================+                 |
|                                                                         |
|-------------------------------------------------------------------------|
| Press [D4] to go back | Auto-closes in 15s                              |
+-------------------------------------------------------------------------+
```

* **`D4` (Tap):** Move selection UP.
* **`D5` (Tap):** Move selection DOWN.
* **`D9` (Tap):** Execute selected action. Selecting `Power Off / Sleep...` opens the confirmation dialog.
* **Auto-Close:** If untouched for 15s, automatically restores reading view.

---

### State B2: Power Off / Deep Sleep Confirmation Dialog
Prevents accidental power-offs with an explicit confirmation step.

```
+-------------------------------------------------------------------------+
| POWER & SLEEP                                               [==== ] 82% |
|-------------------------------------------------------------------------|
|                                                                         |
|         +=============================================+                 |
|         | POWER OFF / DEEP SLEEP?                     |                 |
|         |---------------------------------------------|                 |
|         | Put device to sleep?                        |                 |
|         | Enters Stage 2 (Zzz screensaver).         |                 |
|         | Only D9 touch wakes the device.             |                 |
|         |                                             |                 |
|         |  +-----------------+   +-----------------+  |                 |
|         |  |   [D4] BACK     |   |   [D9] SLEEP    |  |                 |
|         |  |   < Go Back     |   |   Confirm >     |  |                 |
|         |  +-----------------+   +-----------------+  |                 |
|         +=============================================+                 |
|                                                                         |
|-------------------------------------------------------------------------|
| Press [D4] (PREV) to cancel and go back                                 |
+-------------------------------------------------------------------------+
```

* **`D4` ("Previous" Button):** **CANCEL & GO BACK** to reading!
* **`D9` (Tap):** **CONFIRM SLEEP** $	o$ Immediately renders "Zzz" screensaver and enters ~20 µA deep sleep.

---

### State C: Module 1 — Book Selector (Up to 7 Books)
Allows the reader to browse stored `.ebk` files in LittleFS, view metadata, and switch books.

```
+-------------------------------------------------------------------------+
| LIBRARY BROWSER (3 / 7 Stored)          [==== ] 82%     [ < Back to Menu]
|-------------------------------------------------------------------------|
|                                                                         |
|  [ ] 1. The Time Machine (H.G. Wells)                P.  42 / 120 (35%) |
|  [*] 2. The Shadow Over Innsmouth (H.P. Lovecraft)   P.  24 / 180 (13%) |
|  [ ] 3. Meditations (Marcus Aurelius)                P. 110 / 250 (44%) |
|  [ ] 4. Dune (Frank Herbert)                         P.   1 / 580 ( 0%) |
|  [ ] 5. Neuromancer (William Gibson)                 P.  88 / 310 (28%) |
|  [ ] 6. Foundation (Isaac Asimov)                    P.   0 / 240 ( 0%) |
|  [ ] 7. [ Empty Slot / Free Storage: 3.8 MB ]                           |
|                                                                         |
|-------------------------------------------------------------------------|
| [PREV] Move Up                 [OK] Open Book          [NEXT] Move Down |
+-------------------------------------------------------------------------+
```

* **`PREV`:** Scrolls cursor upward.
* **`NEXT`:** Scrolls cursor downward.
* **`OK` (Tap):**
  * If selecting current book: Closes menu, resumes reading.
  * If selecting new book: Saves current book progress to RTC/LittleFS, loads target book header & index, performs a clean full refresh on the new book's last saved frame, and re-enters deep sleep.
* **`OK` (Hold):** Returns to Top Menu Hub.

---

### State D: Module 2 — Page Jump
With only 3 buttons, jumping through a book utilizes **Percentage / Section scrubbing**:

```
+-------------------------------------------------------------------------+
| JUMP TO PAGE: The Shadow Over Innsmouth  [==== ] 82%      [ < Cancel ]  |
|-------------------------------------------------------------------------|
|                                                                         |
|   Current Position: Page 24 of 180 (13%)                                |
|                                                                         |
|   Target Preview:  [ Page 54 of 180 (30%) ]                             |
|                                                                         |
|   <<< [ -10% ]        [ -10 Pages ]    [ +10 Pages ]        [ +10% ] >>>|
|                                                                         |
|   [ JUMP TO TARGET PAGE ]                        [ RESTORE ORIGINAL ]   |
|                                                                         |
|-------------------------------------------------------------------------|
| [PREV] Scrub Left              [OK] Confirm Jump      [NEXT] Scrub Right|
+-------------------------------------------------------------------------+
```

* `PREV`: Scrub back (e.g. -10 pages or -10%).
* `NEXT`: Scrub forward (e.g. +10 pages or +10%).
* `OK` (Tap): Jump to target page and enter deep sleep.
* `OK` (Hold): Cancel scrub and return to original page.

---

### State E: Module 3 — Bookmark Manager
Each book supports multiple instant bookmarks stored alongside book progress.

```
+-------------------------------------------------------------------------+
| BOOKMARKS: The Shadow Over Innsmouth    [==== ] 82%     [ < Back to Menu]
|-------------------------------------------------------------------------|
|                                                                         |
|  [+] [ ADD BOOKMARK AT CURRENT PAGE (P. 24) ]                           |
|                                                                         |
|  * Bookmark 1: Page 12  ("The decaying waterfront...")      [GO] [DEL]  |
|  * Bookmark 2: Page 24  ("Officials of Federal Gov...")     [GO] [DEL]  |
|  * Bookmark 3: Page 67  ("The Gilman House night clerk...") [GO] [DEL]  |
|  * [ Empty Slot ]                                                       |
|                                                                         |
|-------------------------------------------------------------------------|
| [PREV] Up                      [OK] Select Option               [NEXT] Down
+-------------------------------------------------------------------------+
```

---

## 4. Battery Indicator & Partial Refresh Architecture

### 4.1. Visual Geometry & Design
The battery indicator is positioned at the top-right header:
* **Position:** $X = 328$ to $396$, $Y = 4$ to $18$ (Bounding box: $68 \times 14\text{ px}$).
* **Components:**
  * **Percentage Text:** Small 5x7 or 6x8 font displaying `XX%` (e.g. `82%`).
  * **Battery Shell:** Hollow $22 \times 10\text{ px}$ rectangle (1px border).
  * **Battery Terminal:** $2 \times 4\text{ px}$ solid protrusion on the right.
  * **Fill Gauge:** Proportional solid black bar ($0$ to $18\text{ px}$ width).
  * **Low Battery Warning ($\le 15\%$):** Hollow outline with an internal exclamation mark `[ ! ]` or 1px flashing border.

```
Visual Rendering:
+---------------+  +--+
| █████████     |  |  |  82%
+---------------+  +--+
  <- 18px max ->
```

---

### 4.2. Update Cadence Policy ("Not Too Often")
E-paper partial refreshes should be used judiciously to maximize battery life and prevent micro-flickers.

* **Trigger Source:** **Button Presses on Page Flips ONLY.**
  * The system **never** sets an RTC timer just to wake up and read the battery.
  * When the user presses `NEXT` or `PREV`, the ESP32-S3 must wake up anyway to render the next text frame. The ADC sample takes $< 2\text{ ms}$ of CPU time—adding **zero extra wake penalty**.
* **Threshold & Filtering Policy:**
  * **Hysteresis & Averaging:** Take 16 fast samples using `analogReadMilliVolts(BAT_ADC_PIN)` and compute the average to eliminate ADC noise and voltage sag during wake.
  * **Display Update Rule:** The displayed battery percentage is updated **only if:**
    1. The calculated percentage has changed by $\ge 1\%$ ($|\Delta \%| \ge 1\%$), **OR**
    2. At least **15 page turns** have elapsed since the last battery display update, **OR**
    3. The battery drops across a critical alert threshold ($20\%$, $10\%$, $5\%$).
* **RTC State Tracking:**
  Store in `RTC_DATA_ATTR`:
  * `last_displayed_bat_pct`: Previous percentage shown on screen.
  * `pages_since_bat_update`: Counter incremented on every page flip.

---

### 4.3. Partial Refresh Strategy
* **Integrated Page Turn (Default):**
  When a page turns, the top-right battery icon and percentage are simply rendered into the overall display buffer along with the text. A single partial refresh updates the entire new page and battery together in $\approx 350\text{ ms}$.
* **Standalone Battery Update (Rare):**
  If only the battery area needs to update (e.g. during an extended menu session or HUD check), use GxEPD2's localized partial window:
  ```cpp
  display.setPartialWindow(320, 4, 76, 20); // Restrict refresh to top-right corner
  display.firstPage();
  do {
      render_battery_bar(328, 6, current_bat_pct);
  } while (display.nextPage());
  ```
  This completes in $< 120\text{ ms}$ with zero disturbance or flicker to the rest of the display.
* **Full Refresh Cadence (Anti-Ghosting):**
  After **15 partial refreshes**, or upon opening/closing the Main Menu, issue one **clean full refresh** with waveform clearing to reset particle alignment and maintain contrast.

---

### 4.4. Voltage Divider Electrical & Power Analysis (Combined Option A + B)

```
    BAT+ (3.0V - 4.2V LiPo)
      │
     [R1: 100kΩ (or 470kΩ)]
      │
      ├──────► GPIO 2 (ADC1_CH1 on Xiao S3 D1)
      │   │
      │  [C1: 10nF Ceramic Filter Cap]
      │   │
     [R2: 100kΩ (or 470kΩ)]
      │   │
      └───┴──► GPIO 44 (Xiao S3 D7 — Switched Low-Side Ground)
```

#### Why Combining A + B is the Gold Standard:
* **Option A (Switched Ground on D7):** When the ESP32-S3 is in deep sleep, `GPIO 44` (`D7`) is kept in high-impedance `INPUT` mode. This physically breaks the circuit between BAT+ and GND, guaranteeing **true $0\ \mu\text{A}$ standby drain**.
* **Option B (100k + 100k or 470k + 470k):** Even during the brief active measurement window, the high resistance limits active current to only:
  * With $100\text{k} + 100\text{k}$: $I_{active} = \frac{4.2\text{ V}}{200\text{k}\Omega} = \mathbf{21\ \mu\text{A}}$ (negligible).
  * With $470\text{k} + 470\text{k}$: $I_{active} = \frac{4.2\text{ V}}{940\text{k}\Omega} = \mathbf{4.4\ \mu\text{A}}$.
* **The Role of the 10nF Filter Cap (Crucial for ADC Accuracy):**
  High resistor values create a high source impedance ($R_{source} = 50\text{ k}\Omega$ for $100\text{k}$, or $235\text{ k}\Omega$ for $470\text{k}$), which can cause the ESP32-S3's internal sample-and-hold capacitor to sag the measured voltage. Adding a **$10\text{ nF}$ ceramic capacitor across $R_2$** acts as a low-impedance reservoir, delivering crisp, noise-free samples.
* **Firmware Read Routine:**
  ```cpp
  pinMode(D7_PIN, OUTPUT);
  digitalWrite(D7_PIN, LOW);   // Ground the divider
  delayMicroseconds(800);      // Allow 10nF cap to charge/settle (tau = 0.5ms)
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN) * 2;
  pinMode(D7_PIN, INPUT);      // Return to high-Z (0 uA cutoff)
  ```

---

### 4.5. LiPo Voltage to Percentage Mapping Table
Standard 3.7V Lithium-Polymer cells have a steep non-linear discharge curve:

| Battery Voltage ($V_{bat}$) | Divider Voltage ($V_{adc}$) | Displayed Percentage | Battery Bar Fill | Status |
|---|---|---|---|---|
| $\ge 4.15\text{ V}$ | $\ge 2.07\text{ V}$ | **100%** | Full (18 px) | Full Charge |
| $4.00\text{ V}$ | $2.00\text{ V}$ | **85%** | 15 px | Good |
| $3.85\text{ V}$ | $1.92\text{ V}$ | **65%** | 12 px | Nominal |
| $3.75\text{ V}$ | $1.87\text{ V}$ | **45%** | 8 px | Mid-range |
| $3.65\text{ V}$ | $1.82\text{ V}$ | **25%** | 5 px | Approaching Low |
| $3.50\text{ V}$ | $1.75\text{ V}$ | **10%** | 2 px (Warning Outline) | Low Battery |
| $\le 3.35\text{ V}$ | $\le 1.67\text{ V}$ | **2%** | Empty / `[ ! ]` Icon | Critical (Save & Sleep)|

---

## 5. Analysis of Flaws, Edge Cases, and Concrete Solutions

### Flaw 1: The "Wake-on-Hold" Ambiguity
* **The Problem:** The ESP32-S3 boots in ~15 ms from deep sleep. When waking via `OK`, it cannot immediately distinguish a tap from a menu hold.
* **The Solution:** On boot via `OK`, poll the pin. If released $< 800\text{ ms}$, show the quick 3-second status HUD and sleep. If held $\ge 800\text{ ms}$, launch the Interactive Menu Hub.

### Flaw 2: Getting Trapped in Menus (No Back Button)
* **The Problem:** Submenus can trap the user if they change their mind.
* **The Solution (Triple-Layer Escape):** (1) First item of every list is `[ < Return ]`, (2) Global 1-second long-press of `OK` cancels out, and (3) a 20-second inactivity watchdog aborts to deep sleep.

### Flaw 3: Battery ADC Jitter & Unnecessary Redraws
* **The Problem:** ADC readings naturally fluctuate by $\pm 10\text{ mV}$ due to RF/load noise. If redrawn every turn, the indicator might flicker between 81% and 82% constantly.
* **The Solution:** 16-sample multi-sampling with a strict hysteresis filter: only update screen if $|\Delta \%| \ge 1\%$ or after 15 page turns.

### Flaw 4: Multi-Book Progress Retention
* **The Problem:** Switching between 7 books would overwrite a single scalar progress pointer.
* **The Solution:** Maintain a 7-slot `LibraryState` table in RTC memory (and mirrored in `/spiffs/state.bin` on LittleFS) preserving frame indices and bookmarks independently for all 7 books.

---

## 6. Complete Button Function Matrix

| State / Screen | `PREV` Button (Tap) | `NEXT` Button (Tap) | `OK` Button (Tap) | `OK` Button (Hold $\ge 800\text{ms}$) | Inactivity Timeout |
|---|---|---|---|---|---|
| **Reading Mode** | Prev Page + Bat Check | Next Page + Bat Check | Show HUD Overlay (3s) | **Open Main Menu** | Immediate Deep Sleep |
| **Main Menu Hub** | Select Left Card | Select Right Card | Open Selected Submenu | **Exit to Reading** | 20s $\to$ Deep Sleep |
| **Book Selector** | Cursor Up | Cursor Down | Open Selected Book | **Return to Main Menu**| 20s $\to$ Deep Sleep |
| **Page Jump** | Jump -10 Pages | Jump +10 Pages | Confirm & Load Page | **Cancel & Return** | 20s $\to$ Deep Sleep |
| **Bookmarks** | Cursor Up | Cursor Down | Execute (Go / Add) | **Return to Main Menu**| 20s $\to$ Deep Sleep |
