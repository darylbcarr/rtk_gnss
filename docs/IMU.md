# IMU Design & Examination Log (LSM6DSVQTR)

Living document for the rover's IMU work. Update the **Status** table and **Examination Log** as the investigation progresses.

> Chip figures marked ⚠ are from memory of the datasheet and have not been verified. Check them against the ST datasheet before designing around them.

## Status

| # | Step | State |
|---|------|-------|
| 1 | Live tilt API, OLED bubble, tilt gating, `tilt_deg` in output | Tilt API and OLED bubble built and compiling; not yet tested on hardware. Tilt gating and `tilt_deg` output still to do |
| 2 | Calibration routine (accel offsets, mount rotation, pole length) stored in NVS | Level (X/Y) offset calibration and per-session pole length built and compiling; not yet tested on hardware. Full mount rotation and gyro bias not done |
| 3 | Stillness gating in capture state machine | Not started |
| 4 | FIFO-watermark interrupt, per-state ODR profiles, inactivity auto-dim | Not started |
| 5 | GNSS-aided yaw alignment and per-epoch lever-arm correction | Not started |

## Open questions

- ~~Pole length / IMU placement~~ Answered 2026-10-06: see Design inputs.
- Is INT1/INT2 wired to an ESP32 GPIO on the production board? (Needed for step 4.)
- Verify SFLP FIFO tag IDs for gravity vector and gyro bias in ST's `lsm6dsv_reg.h`.

## Design inputs

- Pole length is typically ~2 m, but the exact height changes per session. It must be a per-session setting (not only an NVS constant) that the operator can edit before capture.
- The device sits directly below the antenna, so the IMU-to-antenna lever arm is small and can be treated as part of the pole length (antenna ARP to ground tip) for now. Record the vertical offset from the IMU to the ARP in the calibration data.

## Current implementation

Step 1 additions (2026-10-06):

- `imu_read_tilt()` in `components/imu`: tilt from the raw accelerometer (not the SFLP quaternion, whose frame convention is unverified). Low-pass filtered, ~0.25 s time constant, and samples are only folded in while |a| is within 1 g ± 0.08 g. Assumes IMU +Z is along the pole axis until the mount calibration (step 2).
- `display_pixel/circle/disc` primitives added to `components/display`.
- Rover menu has a new **Level** screen (`components/ui`): vial with a bubble, 2° tolerance ring, tilt degrees, X/Y components, estimated tip error for a 2 m pole (`s_pole_len_m`, fixed until step 2), and LEVEL / TILTED / MOVING status.
- The main loop wakes every 100 ms while the Level screen is open (`ui_wants_fast_refresh()`); per-epoch logging, BLE and NTRIP work still run once per GNSS epoch. UPS reads are throttled to 1 Hz.
- On-screen axis mapping is tunable via `LEVEL_SWAP_XY / LEVEL_FLIP_X / LEVEL_FLIP_Y` in `ui.c` once the board's mounting is known.
- The OLED I2C bus runs at 50 kHz, so a full redraw takes on the order of 200 ms. The bubble will update at roughly 4–5 Hz, and the redraw briefly occupies the shared bus.

Step 2 additions (2026-10-06):

- **Level calibration** (`components/imu/imu_cal.c`, Rover menu → *Calibrate lvl*): four positions, each rotated 90° about the vertical, ~5 s averaged per position. The table's tilt rotates with the device while the sensor/mount offset does not, so the mean of the four readings is the offset; no level surface is needed. A position restarts if the device moves (spread > 0.5°). The result is rejected if the 0°/180° and 90°/270° estimates disagree by more than 0.5° or the offset exceeds 5°. Host-tested with eyeballed bench readings from 2026-10-06 (those were later superseded by the on-device result; see the log).
- The offset is saved in NVS (`lvl_off_x/y`, micro-units), loaded at boot, and subtracted in `imu_read_tilt()` (`raw_x/raw_y` keep the uncorrected values). The Level screen shows `TILT RAW` until a calibration exists.
- **Pole length** (Rover menu → *Pole length*): 0.50–4.00 m in 1 cm steps via the encoder, persisted in NVS (default 2.00 m), used for the Level screen's tip-error estimate and available to the capture workflow.
- The Rover menu now scrolls (it has 7 entries; the display shows 6).
- Only the X/Y level offset is calibrated. Pole-axis alignment beyond that (the IMU +Z vs the true pole axis) is absorbed into the same offset when the device is calibrated sitting on its flat bottom, which is how it mounts; gyro bias is not touched.

Baseline before step 1:

Files: `components/imu/imu.c`, `components/imu/include/imu.h`. Hardware history (SPI to I2C pivot) is in project memory; I2C is the shipped path.

- I2C @ 0x6A on the shared GPIO8/GPIO9 bus (OLED, UPS, IMU). CS tied high, SA0 low.
- SFLP game rotation vector at 60 Hz. Accel and gyro both high-performance at 60 Hz, fixed. Accel ±4 g, gyro ±500 dps.
- `imu_read()` drains the FIFO one 7-byte frame per I2C transaction and keeps only the last quaternion.
- Exposes `roll`, `pitch` and relative `heading` (no magnetometer, drifts slowly).
- Nothing consumes the output except one debug log line (`main/main.cpp:474`). Capture, GeoJSON and OLED do not use it.

## Chip capabilities

| Feature | Use for the rover |
|---|---|
| Accel (±2/4/8/16 g) and gyro, ODR from ~2 Hz up to kHz, with low-power modes | Static tilt measurement and motion detection |
| SFLP: game rotation vector, gravity vector, gyro bias, batched into the FIFO | Tilt and relative yaw computed on-chip. Gravity vector and gyro bias tags are not enabled yet |
| FIFO with watermark interrupt on INT1/INT2 | Batch samples, wake the MCU only when needed |
| Wake-up, activity/inactivity, 6D, tap, free-fall, finite state machine ⚠ | Hardware motion detection at no ESP32 cost |
| User accel offset registers ⚠ | Store level-calibration correction in the chip |
| Temperature sensor | Temperature-compensate offsets |

No magnetometer. This is the main constraint on heading.

## Accuracy analysis (pole use)

The GNSS measures the antenna; the ground point is `L` below it. Error is `L·sin(tilt)`:

| Pole tilt | Error on a 2 m pole |
|---|---|
| 1° | 3.5 cm |
| 2° | 7 cm |
| 5° | 17 cm |

A bubble level is rarely held better than about 2° in practice, which is larger than RTK Fixed error. Tilt is the largest remaining error source.

### 1. Gravity-referenced tilt needs no heading

Tilt magnitude comes from the gravity vector alone, independent of yaw. Cheapest and largest win:

- Live level bubble on the OLED.
- Gate capture on tilt (pause or discard epochs above a limit, e.g. 2°).
- Inflate reported accuracy by `L·sin(tilt)`; store `tilt_deg` per point in GeoJSON and CSV.

### 2. Accel zero-g offset

Zero-g offset is on the order of ±tens of mg ⚠. At 20 mg that is about 1.1°, or 3.9 cm on 2 m. Fix with a one-time calibration:

- Two-position flip or six-position calibration.
- Level the pole on a known-plumb reference.
- Store mount rotation, accel offsets and pole length in NVS. This also absorbs enclosure misalignment with the pole axis.

### 3. Heading, needed to correct the point rather than only flag it

Compensation needs the compass direction of the lean. The game rotation vector's yaw is relative, arbitrary, and drifts slowly.

Proposed approach: align yaw to the GNSS frame instead of adding a magnetometer.

- When the pole tilts about a fixed tip, the antenna moves horizontally by about `L·sin(tilt)` toward the lean direction.
- Compare the RTK Fixed antenna displacement with the IMU tilt vector in the body frame to get the yaw offset.
- Hold the offset between captures. Gyro drift over a 60-epoch capture is negligible; re-align opportunistically.
- Only works while RTK is Fixed. In Float, the tilt gate (item 1) is the fallback.

Alternative: add a magnetometer. Extra hardware, fails near steel, and the PDD deliberately avoids it. Defer unless GNSS alignment proves unreliable.

### 4. Stillness gating

Accel-based tilt is wrong while the pole moves. Check gyro magnitude and accel variance per epoch and exclude moving epochs from the 60-epoch, 95% Fix rule.

## Battery analysis

The chip draws microamps to a fraction of a milliamp. That is negligible next to the ESP32 with WiFi/BLE, the LC29H, the OLED and the LoRa radio (tens to ~100+ mA each; typical figures, not measured). The IMU saves battery indirectly, by not keeping the MCU awake and by driving power-state changes.

1. **Match IMU rate to state:**

   | State | Configuration |
   |---|---|
   | Walking or idle | Accel low-power ~15 Hz, gyro off, SFLP off, activity/inactivity interrupt armed |
   | Approaching a point | SFLP 15–30 Hz for the live bubble |
   | Capturing | SFLP 30 Hz; per-epoch tilt needs nothing near 60 Hz at 1 Hz GNSS |

2. **Interrupt instead of polling.** INT1 to a GPIO, FIFO watermark, ESP32 sleeps between batches. Read multiple frames per burst instead of one transaction per frame.
3. **Inactivity drives the large savings.** After N seconds still or stowed: blank the OLED, reduce BLE advertising/update rate, light-sleep the ESP32, wake on the chip's motion interrupt. Keep the LC29H running; re-converging RTK costs more than it saves.
4. **Measure.** The INA219 on the UPS can log current per state to verify before tuning.

## Alternative IMU devices

Graded A (best) to D on the three criteria. Grades are my judgement from the cited and recalled specs, not bench measurements. Prices are single-unit and drift with stock.

| Device | Accuracy (tilt) | Battery | Price | On-chip fusion | Notes |
|---|---|---|---|---|---|
| **LSM6DSV (current)** | B+ | A | A (~$5.8) | SFLP 6-axis | Accel zero-g offset ±12 mg, gyro noise 2.8 mdps/√Hz, accel LP1 4 µA at 1.875 Hz, gyro power-down 2.6 µA. Already working in firmware |
| LSM6DSV16X / 16BX | B+ | A | A (~$5.6–5.8) | SFLP | Same core IMU class; extra features (MLC, Qvar) do not help tilt. No reason to switch |
| TDK ICM-45686 | A- | A- | B+ (~$6.7) | None (host fusion) | Accel noise 70 µg/√Hz, gyro 3.8 mdps/√Hz, 6-axis 0.22 mA (low power) to 0.42 mA (low noise). Better on-paper temperature stability. New driver and host-side fusion needed |
| TDK ICM-42688-P | A- | A- | A (~$3 at volume) | None | Good part, but DigiKey stock was 0 with restock listed for 2027 |
| Bosch BMI270 | C | A- | A | None | 685 µA full 6-axis, ~10 µA accel-only low power. Community testing finds drift poor; not recommended |
| CEVA BNO085 / BNO086 | B (heading A-/B+) | C | D (~$11–16) | 9-axis, absolute heading | Magnetometer gives north-referenced heading (static error ~2°), but fusion MCU draws mA. BNO085 supply is constrained; BNO086 in stock. Magnetometer is exposed to pole, battery and antenna interference |
| Murata SCL3300 (inclinometer) | A+ | B | D (~$22–29) | None | Dedicated tilt sensor, ~0.001°/√Hz noise. No gyro, so it needs a second sensor for dynamics. SPI interface (verify), which this project had trouble with on the LSM6DSV |
| LSM6DSV + add-on magnetometer (e.g. LIS2MDL class) ⚠ | B+ tilt, heading from mag | A- | A (+~$2–4) | SFLP + host mag fusion | Keeps current chip and adds absolute heading. Same interference caveat as BNO08x |

### Interpretation

Chip choice is not the accuracy bottleneck:

- 1 mg of accel error is ~0.057°, which is ~2 mm on a 2 m pole. After a level calibration, any listed 6-axis part resolves tilt to a few millimetres at the tip. What remains is temperature drift of the offset, which needs bench logging to compare parts (not found in the searches).
- Yaw error matters more. Residual lateral error is about `L·sin(tilt)·sin(yaw error)`. At 5° tilt on 2 m (17 cm raw), a 10° yaw error leaves ~3 cm; at 2° tilt it leaves ~1.2 cm.
- Therefore the decision that moves accuracy is how heading is obtained (GNSS-aided alignment vs magnetometer), not which 6-axis IMU is used.

### Recommendation

1. Keep the LSM6DSV. It is among the best-rated 6-axis parts on price and power, the SFLP offloads fusion from the ESP32, and the driver works on production hardware.
2. Do the planned calibration and tilt work first (steps 1–3), then measure the residual error outdoors.
3. If GNSS-aided yaw alignment (step 5) proves unreliable, add a magnetometer next to the LSM6DSV rather than replacing it with a BNO08x.
4. Consider the ICM-45686 only if bench logging of static accel vs temperature shows LSM6DSV offset drift is the dominant error.
5. A dedicated inclinometer (SCL3300) is the accuracy ceiling but costs 4–5x and adds a part; not justified unless the 2 m tip error budget is below ~1 cm.

## Recommended order

1. `imu_get_tilt()`, OLED bubble, tilt gating, `tilt_deg` in output. No heading needed.
2. Calibration routine with NVS storage.
3. Stillness gating in the capture state machine.
4. FIFO-watermark interrupts, per-state ODR profiles, inactivity auto-dim.
5. GNSS-aided yaw alignment and per-epoch lever-arm correction.

Steps 1–3 deliver most of the accuracy benefit at low risk. Step 5 is the real project and needs outdoor RTK Fixed data to validate.

## Examination Log

Newest entries last. Record findings, measurements and decisions here.

- **2026-10-06** — Initial review of chip capabilities against rover pole use. Conclusions above. No code changes yet.
- **2026-10-06** — Answered open questions: ~2 m pole, length edited per session; IMU directly below the antenna. Reviewed alternative IMUs (see Alternative IMU devices). Conclusion: keep the LSM6DSV; the heading source matters more than the 6-axis chip. Added a to-do: bench-log static accel vs temperature to quantify offset drift.
- **2026-10-06** — Step 1 implemented (tilt API, display primitives, Level screen, fast-refresh main loop). Builds clean with `idf.py build`. Not flashed. To verify on hardware: confirm the pole axis is IMU +Z, check the bubble direction and tune the `LEVEL_*` defines, and compare the reported tilt against a known-plumb reference.
- **2026-10-06** — Eyeballed bubble positions (before the on-screen x/y values were known) suggested an IMU offset of about (0.1°, −0.9°). That was superseded by the on-device calibration below. Built the calibration routine and per-session pole length (step 2). To verify on hardware: run Calibrate lvl, then check that the bubble centres with the device on the table at all four rotations, and that the saved offset survives a reboot.
- **2026-10-06** — First on-device calibration: offset x +0.14°, y −0.05° (about 2.6 mg, ~0.5 cm on 2 m), surface tilt 0.6°, pair agreement 0.26° (limit 0.5°). The eyeballed ~0.9° estimate was wrong; the unit is well inside the ±12 mg datasheet figure. Offset uncertainty is about ±0.13°. Open: confirm the saved offset and tilt reading are consistent at all four rotations, and check bubble direction. The earlier visual report of the bubble touching the 2° ring does not match a 0.75° maximum and is unexplained.
- **2026-10-06** — Post-calibration check on the table, on-screen tilt x/y at 0/90/180/270°: (0.3, 0.0), (−0.4, 0.5), (−0.7, −0.1), (−0.1, −0.7). Surface tilt from the two opposite pairs is 0.5° and 0.62° (perpendicular, consistent). The four points average (−0.22, −0.08), so the saved offset is off by about 0.24°, which is about 0.8 cm on 2 m and the same size as the calibration's own 0.26° pair disagreement. Lifting the left edge at 0° moved the bubble up, so IMU +Y runs to the operator's left and +X away from them; set `LEVEL_SWAP_XY=1`, `LEVEL_FLIP_X=1`. To verify: left edge up moves the bubble left, back edge up moves it up.

### Sources for the alternatives review

- [LSM6DSV datasheet (ST)](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf)
- [IMU Comparison, SlimeVR docs](https://docs.slimevr.dev/diy/imu-comparison.html)
- [LSM6DSV16X on DigiKey](https://www.digikey.com/en/products/detail/stmicroelectronics/LSM6DSV16XTR/16841485)
- [LSM6DSV16BX on DigiKey](https://www.digikey.com/en/products/detail/stmicroelectronics/LSM6DSV16BXTR/16841542)
- [ICM-42688-P on DigiKey](https://www.digikey.com/en/products/detail/tdk-invensense/ICM-42688-P/10824934)
- [ICM-45686 (TDK InvenSense)](https://www.invensense.tdk.com/en-us/products/6-axis/icm-45686)
- [SCL3300 (Murata, DigiKey highlight)](https://www.digikey.com/en/product-highlight/m/murata-electronics-north-america/scl3300-series-3-axis-inclinometer)
- [BNO085 on DigiKey](https://www.digikey.com/en/products/detail/ceva-technologies-inc/BNO085/9445940)
