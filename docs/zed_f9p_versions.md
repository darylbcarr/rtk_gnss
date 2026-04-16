# u-blox ZED-F9P Version Comparison

The ZED-F9P is u-blox's flagship F9-platform high-precision GNSS module. Since its 2018 launch it has gone through six ordering-code revisions. Each revision either corrects hardware errata, ships a newer firmware image, or introduces a distinct feature set.

---

## Version Genealogy

```
ZED-F9P-00B  (2018, obsolete)
      │
ZED-F9P-01B  (2018–2021, HPG 1.12, L1/L2)
      │  PCN UBX-20027480
ZED-F9P-02B  (2021, HPG 1.13, hardware rev + firmware bump)
      │
ZED-F9P-04B  (2022, HPG 1.30/1.32, SPARTN + CLAS)
      │
ZED-F9P-05B  (2024, HPG 1.50/1.51, Galileo OSNMA)

ZED-F9P-15B  (2023, HPG L1L5 1.40, L1/L5 band variant — separate line)
```

---

## Quick-Reference Table

| Feature | 00B | 01B | 02B | 04B | 05B | 15B |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Status** | Obsolete | Legacy | Active | Active | Active | Active |
| **GNSS bands** | L1/L2 | L1/L2 | L1/L2 | L1/L2 | L1/L2 | L1/L5 |
| **Baseline firmware** | HPG 1.00 | HPG 1.12 | HPG 1.13 | HPG 1.30 | HPG 1.50 | L1L5 1.40 |
| **RTK horizontal** | 0.01 m + 1 ppm | 0.01 m + 1 ppm | 0.01 m + 1 ppm | 0.01 m + 1 ppm | 0.01 m + 1 ppm | 0.01 m + 1 ppm |
| **Concurrent GNSS** | 4 | 4 | 4 | 4 | 4 | 4 |
| **SPARTN corrections** | — | — | — | Yes | Yes | Yes |
| **CLAS (Japan)** | — | — | — | Yes | Yes | — |
| **Galileo OSNMA** | — | — | — | — | Yes | — |
| **Moving base** | Yes | Yes | Yes | Yes | Yes | No (v1.40) |
| **UART / SPI / I²C / USB** | Yes | Yes | Yes | Yes | Yes | Yes |
| **Supply voltage** | 2.7–3.6 V | 2.7–3.6 V | 2.7–3.6 V | 2.7–3.6 V | 2.7–3.6 V | 2.7–3.6 V |
| **Operating temp** | −40–85 °C | −40–85 °C | −40–85 °C | −40–85 °C | −40–85 °C | −40–85 °C |
| **Form factor** | 17×22 mm | 17×22 mm | 17×22 mm | 17×22 mm | 17×22 mm | 17×22 mm |
| **Active antenna required** | Yes | Yes | Yes | Yes | Yes | Yes |

---

## Per-Version Details

### ZED-F9P-00B — Original Release (2018, **Obsolete**)

The first production variant. Now marked obsolete by u-blox; no longer orderable through normal distribution. Firmware version history predates the formal HPG versioning scheme used in later releases.

- Shares the same 17×22 mm LCC footprint used by all subsequent variants.
- No SPARTN, no OSNMA, no CLAS.

---

### ZED-F9P-01B — Firmware HPG 1.12 (2018–2021, **Legacy**)

The variant most commonly found on early-generation RTK carrier boards (SparkFun GPS-RTK2, ArduSimple simpleRTK2B first production run, Drotek DP0601).

**GNSS reception (L1/L2)**

| Constellation | Bands |
|---|---|
| GPS / QZSS | L1 C/A, L2C |
| GLONASS | L1OF, L2OF |
| Galileo | E1 B/C, E5b |
| BeiDou | B1I, B2I |
| SBAS | L1 C/A |

**Key firmware capabilities (HPG 1.12)**
- Concurrent reception of all five constellations above.
- RTK base and rover modes; configurable via UBX-CFG-TMODE3.
- Moving-base (heading) mode over UART2/RTCM3.
- 8 Hz RTK output rate (25 Hz raw measurements).
- Survey-in and fixed-position base.

**Limitations**
- No SPARTN/PPP-RTK stream support — can only use RTCM3 corrections.
- UBX-NAV-RELPOSNED heading only valid when both modules are on the same firmware ≥ HPG 1.12.

---

### ZED-F9P-02B — Firmware HPG 1.13 (2021, **Active**)

Introduced via PCN UBX-20027480. This was a minor stepping that combined a hardware tweak (undisclosed silicon errata correction) with a firmware bump.

**Changes from 01B**
- Flash ships with HPG 1.13 (01B units with HPG 1.12 cannot be field-upgraded to 1.13; 02B ships pre-flashed).
- Minor improvement to RTK integer ambiguity fix time under weak-signal conditions.
- Same GNSS band set and interface complement as 01B.
- No new protocol support over 01B.

---

### ZED-F9P-04B — Firmware HPG 1.30 / 1.32 (2022, **Active**)

The first major feature-addition stepping. This is the variant currently recommended by u-blox for new designs using RTCM3 or PPP-RTK corrections.

**New capabilities over 02B**

| Feature | Notes |
|---|---|
| **SPARTN protocol** | Decodes u-blox PointPerfect corrections over IP or L-band. SPARTN v2.0.1 support added in HPG 1.32. |
| **CLAS (Centimeter Level Augmentation Service)** | Japan QZSS correction service, requires HPG 1.30+. |
| **PPP-RTK** | Full PPP-RTK convergence alongside classical RTK. |
| **BeiDou B2I** (enhanced) | Improved multipath mitigation on B2I. |
| **Improved RTK convergence** | Faster ambiguity resolution reduces time-to-fix risk in surveying. |

**HPG 1.30 → HPG 1.32 delta**
- SPARTN BeiDou constellation support added (extends PointPerfect coverage in Asia-Pacific).
- SPARTN protocol bumped to v2.0.1.
- Minor UBX message additions for SPARTN status reporting.

**Still L1/L2 dual-band; full moving-base support retained.**

---

### ZED-F9P-05B — Firmware HPG 1.50 / 1.51 (2024, **Active**, Recommended for New Designs)

The current top-of-line L1/L2 variant. It adds hardware-level signal authentication and is u-blox's answer to the growing threat of GNSS spoofing in UAV, survey, and robotics applications.

**New capabilities over 04B**

| Feature | Notes |
|---|---|
| **Galileo OSNMA** | Open Service Navigation Message Authentication — cryptographic verification that Galileo signals originate from the genuine satellite constellation. First u-blox module to ship OSNMA. |
| **Conservative Ambiguity Resolution** | Optional mode that sacrifices convergence speed for a lower false-fix rate; targeted at high-integrity surveying. |
| **Enhanced jamming/spoofing detection** | Real-time detection events reported over UBX-SEC messages. |
| **SPARTN BeiDou** | Inherited from HPG 1.32 base, present from first 05B firmware. |

**HPG 1.50 → HPG 1.51 delta**
- Additional OSNMA robustness improvements.
- Minor UBX interface additions for security event logging.

**Same physical module, same L1/L2 GNSS bands, same moving-base support as 04B.**

---

### ZED-F9P-15B — Firmware HPG L1L5 1.40 (2023, **Active**)

A parallel product line — not a successor to the 05B but a frequency-band alternative for applications where L5 outperforms L2.

**GNSS reception (L1/L5)**

| Constellation | Bands |
|---|---|
| GPS | L1 C/A, **L5** |
| GLONASS | L1OF *(L2 dropped)* |
| Galileo | E1 B/C, **E5a** |
| BeiDou | B1I, **B2a** |
| QZSS | L1 C/A, L5 |
| SBAS | L1 C/A |

**Why choose L1/L5?**
- L5 signals are wider-band (10.23 MHz vs 1.023 MHz for L1 C/A), giving better multipath mitigation in urban canyons and under tree canopy.
- GPS L5, Galileo E5a, and BeiDou B2a share the same 1176.45 MHz frequency, allowing cross-constellation combination.
- Better ionospheric correction when combining L1 + L5 vs L1 + L2 in some environments.

**Limitations vs L1/L2 variants**
- **Moving-base mode not supported** in firmware HPG L1L5 1.40 — verify before deploying in heading applications.
- Galileo OSNMA not available in this firmware line.
- CLAS (QZSS Japan service) not supported.
- L5 signal availability still growing; older infrastructure correction services primarily deliver L1/L2 corrections.

---

## Firmware Version Map

| Hardware | Shipped FW | Latest FW | Notable Additions |
|---|---|---|---|
| 00B | HPG 1.00 (pre-series) | N/A (obsolete) | — |
| 01B | HPG 1.12 | HPG 1.32¹ | — |
| 02B | HPG 1.13 | HPG 1.32¹ | — |
| 04B | HPG 1.30 | HPG 1.32 | SPARTN, CLAS, PPP-RTK |
| 05B | HPG 1.50 | HPG 1.51 | Galileo OSNMA, Conservative AR |
| 15B | L1L5 1.40 | L1L5 1.40 | L1/L5 dual-band |

¹ Field-upgrade path from 01B/02B to HPG 1.30+ is not officially supported by u-blox; these modules remain on the 1.12/1.13 line.

---

## Correction Service Compatibility

| Correction Service | 01B/02B | 04B | 05B | 15B |
|---|:---:|:---:|:---:|:---:|
| RTCM3 (any NTRIP caster) | Yes | Yes | Yes | Yes |
| u-blox PointPerfect (SPARTN/IP) | — | Yes | Yes | Yes |
| u-blox PointPerfect (L-band) | — | Yes | Yes | Yes |
| QZSS CLAS (Japan) | — | Yes | Yes | — |
| Galileo HAS (PPP, free) | — | Partial | Partial | Partial |
| SPARTN BeiDou | — | HPG 1.32+ | Yes | Yes |

---

## Interface & Electrical Summary (All Variants)

| Parameter | Value |
|---|---|
| UART | 2× (configurable baud up to 921600) |
| SPI | 1× (max 5.5 MHz) |
| I²C (DDC) | 1× (up to 400 kHz) |
| USB | 1× USB 2.0 Full Speed |
| Timepulse outputs | 2× (TIMEPULSE, TIMEPULSE2) |
| Extint input | 1× |
| Supply voltage | 2.71–3.6 V |
| I/O voltage | 1.71–3.6 V (tolerant) |
| Active antenna bias | 3.3 V (via V_ANT pin, max 100 mA) |
| Typical power (tracking) | ~68 mW |
| Package | 17×22 mm LCC, 2.4 mm height |
| Weight | ~2.7 g |

---

## Choosing a Version

| Use case | Recommended variant |
|---|---|
| New L1/L2 design with NTRIP/RTCM3 corrections only | **04B** |
| New L1/L2 design requiring PointPerfect (SPARTN) | **04B** or **05B** |
| High-integrity surveying or anti-spoofing required | **05B** |
| Urban canyon / forestry / heavy-canopy environments | **15B** (L1/L5) |
| Moving-base / dual-antenna heading (UAV, robotics) | **04B** or **05B** (not 15B on v1.40) |
| Japan CLAS free correction service | **04B** or **05B** |
| Existing 01B/02B board, stable firmware requirement | Stay on HPG 1.12/1.13 |

---

## Sources

- [ZED-F9P module page — u-blox](https://www.u-blox.com/en/product/zed-f9p-module)
- [ZED-F9P-01B Datasheet UBX-17051259](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-01B_DataSheet_UBX-17051259.pdf)
- [ZED-F9P-02B Datasheet UBX-21023276](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-02B_DataSheet_UBX-21023276.pdf)
- [ZED-F9P-04B Datasheet UBX-21044850](https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf)
- [ZED-F9P-05B Datasheet UBXDOC-963802114-12824](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-05B_DataSheet_UBXDOC-963802114-12824.pdf)
- [ZED-F9P-15B Datasheet UBX-23009090](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-15B_DataSheet_UBX-23009090.pdf)
- [ZED-F9P Integration Manual UBX-18010802](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
- [ZED-F9P Product Summary UBX-17005151](https://content.u-blox.com/sites/default/files/ZED-F9P_ProductSummary_UBX-17005151.pdf)
- [ZED-F9P FW 1.00 HPG 1.32 Release Note](https://cdn.sparkfun.com/assets/learn_tutorials/2/7/8/1/ZED-F9P-FW100-HPG132_RN_UBX-22004887.pdf)
- [ZED-F9P FW 1.00 HPG 1.51 Release Note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-FW100HPG151_RN_UBXDOC-963802114-13110.pdf)
- [ZED-F9P FW 1.00 HPG L1L5 1.40 Release Note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P_FW100HPGL1L5140_RN_UBX-23010071.pdf)
- [ZED-F9P-00B and ZED-F9P-02B Specification Comparison — Ovaga Technologies](https://www.ovaga.com/vs/zed-f9p-00b_zed-f9p-02b)
- [u-blox OSNMA firmware announcement — GPS World](https://www.gpsworld.com/u-blox-enhances-jamming-and-spoofing-protection-with-osnma-firmware-update/)
- [ArduPilot F9P Firmware Update Guide](https://ardupilot.org/copter/docs/common-gps-ublox-firmware-update.html)
- [u-blox Community: ZED-F9P-02B vs ZED-F9P-04B differences](https://portal.u-blox.com/s/question/0D52p0000Bw0ZWrCQM/what-are-the-differences-between-the-zedf9p02b-and-zedf9p04b)
