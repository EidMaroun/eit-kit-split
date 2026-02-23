# EIT Kit Split

## 0) Big-picture: what this code is doing 

1) **Generate an AC excitation current** (AD5930 signal generator configured to ~TEST_FREQ).
2) **Route that current through electrode pairs** using analog MUX chips (selected by `SRC` and `SINK` paths).
3) **Route voltage-sense electrodes** using MUX chips (`VP` and `VN` paths).
4) **Digitize the measured waveform** via a ADC via GPIO bits approach (10-bit parallel input sampled quickly).
5) Compute **RMS magnitude** (and on Teensy path also **magnitude + phase**) of the sensed sinusoid.
6) Repeat over electrode combinations to build a **frame**: an array sized `num_electrodes * num_electrodes`.

### What prints to Serial
- In the constructor, it prints calibration info, then prints an `"origin"` message with a **concatenated list of floating numbers** (no separators).
- In `take_measurements()`, every ~500ms it prints/updates `"framei"` similarly (string created, but actually it doesn’t print the full string; it stores it in `measurements_to_send` for BLE).
- If BLE is enabled, it pushes chunks of that string over BLE (ESP32 path). On Teensy, BLE is irrelevant.

**Key point:** the “data” we care about is `_cur_frame[i]` (smoothed RMS values), where each element corresponds to a (drive pair, sense pair) measurement index.

---

## 1) Teensy 4.1 focus: what to keep, what to delete/ignore

### Keep (Teensy path)
- `__IMXRT1062__` branch logic (Teensy 4.x).
- SPI bit-banging functions: `spi_write()`, AD5930 + AD5270 control.
- MUX routing: `mux_write()`, `mux_write_to_electrode()`, electrode mapping `elec_to_mux[]`.
- Sampling + processing: `gpio_read()`, `gpio_convert()`, `read_signal()`, `read_frame()`, `calibrate_samples()`, `calibrate_gain()`, `calibrate_signal()`.

### Ignore/remove (ESP32/BLE path)
- Everything inside `#if defined(ARDUINO_ARCH_ESP32)`:
  - `BLEStart()`, `ensureBluetoothConnection()`, BLE callback classes
  - MCP23S17 IO expander
  - `vspi_write_*`, `hspi_write_*`
  - Any GPIO mapping meant for ESP32

---

## 2) Proposed module boundaries (input → process → output pipeline)

We want each person’s module output to become the next person’s input.

**Pipeline:**
1) **Hardware Abstraction (pins, SPI, MUX, pots, AD5930)**  → provides “set current”, “select electrodes”, “read raw samples”
2) **Sampling & Signal Processing** → provides `read_signal()` results (rms/mag/phase/error)
3) **Measurement Scheduler (frame builder)** → loops electrodes and builds the frame array
4) **Output & Telemetry** → prints parseable data to Serial (and optionally streams)

---

## 3) Work split

## A) Carmen — Hardware bring-up + pin mapping + SPI/MUX correctness (INPUT STAGE)

### Goal
Make sure the Teensy is driving the board correctly:
- AD5930 is configured and actually outputting TEST_FREQ
- MUX selection actually connects the intended electrodes
- AD5270 gain setting actually changes amplitude

### Own these code areas
- Header constants for Teensy: `CHIP_SEL_*`, `MOSI_PIN`, `SCK_PIN`, ADC input pins.
- `spi_write()`
- `AD5930_Write()`, `AD5930_Set_Start_Freq()`
- `AD5270_Write()`, `AD5270_Set()`, `AD5270_LockUnlock()`, `AD5270_Shutdown()`
- `mux_write()`, `mux_write_to_electrode()`
- `elec_to_mux[]` mapping sanity

### Deliverables (output to others)
1) A single **Teensy 4.1 pinmap header**: `eit_pins_teensy41.h`
   - Verified CS pins, MOSI/SCK pins, MUX CS pins, AD5930 control pins.
2) A tiny “bring-up” test sketch that:
   - sets AD5930 freq,
   - selects a known electrode pair,
   - toggles gains,
   - prints basic sanity (like raw GPIO reads changing).
3) Clarify whether your “ADC bits” are truly wired to those Teensy pins (14–23 in the current Teensy branch).

**Interface to next stage (Serge):**
- Provide stable functions:
  - `set_excitation_freq(freq_hz)`
  - `set_drive_gain(val_0_1023)`
  - `set_meas_gain(val_0_1023)`
  - `select_electrodes(src, sink, vp, vn)`
  - `read_gpio_word()` (raw)

---

## B) Serge — Sampling + signal processing (PROCESS STAGE)

### Goal
Turn “fast raw GPIO reads” into understandable signal metrics:
- RMS (Volts)
- Magnitude (Volts pk-pk or similar)
- Phase (radians) — optional depending on your EIT pipeline
- Error metric (how sinusoidal it is)

### Own these code areas
- `gpio_read()`, `gpio_convert()`, `analog_read()` (if used)
- `calibrate_samples()` and how it sets:
  - `sample_rate`
  - `samples_per_period`
  - `num_samples`
- Teensy `read_signal(rms, mag, phase, error_rate, debug)`
- `sine_compare()` and thresholds

###  fixes you should implement
- Add **CSV-style output** for debug mode:
  - print indices, adc_buf values, ref_buf transitions, computed peaks/troughs.
- Ensure `num_samples` is not too large for stack:
  - Current Teensy `read_signal()` allocates many arrays on stack: `gpio_buf[num_samples][ADC_AVG]`, `adc_buf[num_samples]`, etc.
  - If `num_samples` can be ~1000+, this can crash silently. Move large buffers to static/global or reuse class buffers.

### Deliverables (output to next stage: Elian)
1) A clean function:
   - `SignalMetrics measure_signal() -> {rms_v, mag_v, phase_rad, error}`
2) A short doc:
   - what RMS means here (`rms_10bit * 2.2 / 1024`)
   - where 2.2V reference comes from (is it correct for Teensy 4.1 + your ADC front-end?)
3) Recommended thresholds to treat a reading as “valid” vs “0”.

**Interface to next stage (Elian):**
- `measure_signal()` that assumes electrodes already selected.

---

## C) Elian — Measurement scheduling / frame definition / what index means (PIPELINE + DATA MODEL)

### Goal
Define exactly:
- what one “frame” is,
- what measurement ordering is,
- how many measurements are meaningful (skipping invalid overlaps),
- and how to interpret indices.

### Own these code areas
- `read_frame()` (Teensy branch)
- Drive protocol types: `Meas_t {AD, OP, MONO}`
- Terminal mode `_num_terminals` handling (note: Teensy `read_frame()` currently does NOT implement the 2-terminal logic that ESP32 version has—this is a bug/feature gap).
- Measurement arrays:
  - `_signal_rms[]`, `signal_mag[]`, `_signal_phase[]`, `_cur_frame[]`
- Smoothing:
  - origin: `_cur_frame = 0.8 old + 0.2 new` repeated ~30 times
  - live: `_cur_frame = 0.5 old + 0.5 new` every ~500ms

### Deliverables (output to next stage: Maroun)
1) A precise mapping spec:
   - `index = tx_pair * num_elec + rx_pair`
   - define tx_pair → (src,sink) mapping depending on AD/OP/MONO
   - define rx_pair → (vp,vn) mapping depending on meas type + terminal mode
2) Fix/implement:
   - `_num_terminals == 2` behavior for Teensy (match ESP32 logic).
3) Provide a “frame validity mask”:
   - which indices are invalid because vp/vn overlaps src/sink etc.

**Interface to next stage (Maroun):**
- `Frame build_frame(config) -> arrays {rms[], mag[], phase[], valid_mask[]}`

---

## D) Maroun — Output formatting + serial protocol + “what should we expect” (OUTPUT STAGE)

### Goal
Make the system observable and debuggable:
- Serial output that is parseable and labeled
- Clear expectations for magnitudes/units
- Remove BLE string concatenation behavior

### Own these code areas
- `.ino` example and high-level `EITKitArduino` API usage
- Replace `measurements_to_send` and BLE-specific string chunking with a clean serial publisher
- Add a consistent message format:
  - JSON lines, CSV lines, or framed binary packets

### The key fix: current output is not parseable
Right now the code does:
- `"origin" + streamObj << _cur_frame[i]` (no commas/spaces!)
So you can’t tell where one float ends and the next begins.

### Deliverables
1) Define a serial protocol:
   - Example (CSV):
     - `ORIGIN,<num_elec>,<num_meas>,<rms0>,<rms1>,...,<rmsN>`
     - `FRAME,<t_ms>,<num_elec>,<num_meas>,<rms0>...`
   - Or JSON Lines:
     - `{"type":"frame","t":12345,"num_elec":16,"rms":[...]}`
2) Update `.ino` to:
   - construct eit with BLE disabled (Teensy): `new EITKitArduino(16,1,4,AD,AD,false);`
   - print calibration summary and then periodic frames.
3) Add “expected range” prints:
   - show min/max/mean of frame each cycle.

**Consumes from Elian:**
- `Frame` structure and validity mask to only output valid measurements.

---

## 4)  Refactor Plan

### Proposed new file layout
- `eit_pins_teensy41.h` (Carmen)
- `eit_hal.h/.cpp` (Carmen)
  - SPI write, AD5930/AD5270, mux routing
- `eit_sampling.h/.cpp` (Serge)
  - gpio_read/convert, measure_signal, calibration of samples
- `eit_frame.h/.cpp` (Elian)
  - build_frame, electrode protocol logic, validity mask
- `eit_output.h/.cpp` (Maroun)
  - serial protocol formatting + printing
- `EITKitArduino.h/.cpp` becomes a thin orchestrator that composes the above modules.

---

## 5) Integration contract

### Shared structs
- `struct SignalMetrics { double rms_v; double mag_v; double phase_rad; uint16_t error; bool valid; };`
- `struct Frame { uint16_t num_elec; std::vector<double> rms, mag, phase; std::vector<uint8_t> valid; uint32_t t_ms; };`

### Minimal call sequence (end-to-end)
1) `hal.init();`
2) `sampling.calibrate_samples();`
3) `frame.calibrate_gain();` (or sampling owns it, but choose one owner)
4) `Frame f = frame.build_frame(config);`
5) `output.publish(f);`

---

## 6)  Sanity Check

1) Disable BLE in Teensy build (`false` in constructor, strip BLE codepaths).
2) Make serial output parseable (commas).
3) Print these once after calibration:
   - `num_samples`, `sample_rate`, `samples_per_period`, `current_gain`, `voltage_gain`
4) Print per frame:
   - `min/max/mean` of valid RMS
5) If you see constant zeros or constant max:
   - Carmen checks MUX selection + ADC bit wiring
   - Serge checks sampling buffer sizes and `gpio_convert()` correctness

---

## Who works on what 
- **Carmen:** Pins/SPI/MUX/AD5930/AD5270 hardware-layer correctness (get the board controllable).
- **Serge:** Sampling + signal metrics correctness + performance/stability (get trustworthy rms/mag/phase).
- **Elian:** Frame scheduling + electrode protocol + measurement indexing (get correct EIT dataset layout).
- **Maroun:** Serial output protocol + observability + clean demo sketch (make it understandable + debuggable).
