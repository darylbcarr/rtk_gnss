# Product Design Document: Dual-Mode RTK GNSS System

**Project:** `rtk_gnss`
**Platform:** ESP32 (esp-idf) + u-blox ZED-F9P
**Document Version:** 1.0
**Date:** 2026-04-12

---

## 1. Project Overview

A low-cost, field-deployable dual-mode RTK GNSS device built on an ESP32 microcontroller and a u-blox ZED-F9P GNSS module. The device operates as either a **Rover** (consuming RTCM corrections, outputting cm-accuracy positions) or a **Base Station** (generating RTCM corrections, broadcasting to rovers). Communication is handled by a pluggable transport layer (WiFi/NTRIP, LoRa radio, or direct WiFi link), keeping positioning logic independent of the transmission medium.

Primary use case: field GIS data collection at centimeter accuracy with export-ready metadata for ArcGIS and State Plane coordinate workflows.

---

## 2. Goals and Constraints

### Goals
- Centimeter-level RTK positioning in Rover mode
- Flexible base station operation (survey-in or known fixed position)
- Interchangeable communication without modifying positioning code
- Button-driven point capture with RTK FIX gate
- GIS-ready output (WGS84 + projected coordinates, full metadata)
- Battery-powered, compact, field-rugged
- Low component cost (target: < $150 USD assembled BOM)

### Constraints
- MCU: ESP32 with esp-idf (not Arduino)
- GNSS: u-blox ZED-F9P exclusively (UBX protocol over UART)
- Must support offline field use (no cloud dependency for core positioning)
- 3D-printed enclosure, SMA external antenna port
- Single firmware binary; mode selected at runtime (button or NVS config)

---

## 3. System Architecture

```
┌───────────────────────────────────────────────────────────┐
│                        Application Layer                   │
│  ┌─────────────────┐          ┌─────────────────────────┐ │
│  │   Rover App     │          │    Base Station App     │ │
│  │ - Point capture │          │ - Survey-in control     │ │
│  │ - Fix gating    │          │ - Fix position config   │ │
│  │ - GIS metadata  │          │ - RTCM broadcast mgmt   │ │
│  └────────┬────────┘          └──────────┬──────────────┘ │
└───────────┼───────────────────────────────┼───────────────┘
            │                               │
┌───────────▼───────────────────────────────▼───────────────┐
│                    GNSS Processing Layer                   │
│  - UBX protocol driver (UART DMA)                         │
│  - NMEA / UBX-NAV-PVT parser                              │
│  - RTCM frame framer / passthrough                        │
│  - Fix quality monitor (RTK FLOAT / FIX state machine)    │
│  - Survey-in progress tracking                            │
└───────────────────────────┬───────────────────────────────┘
                            │
┌───────────────────────────▼───────────────────────────────┐
│                   Transport Abstraction Layer              │
│  transport_t interface:                                    │
│    init() / deinit()                                      │
│    send(buf, len)   — outbound RTCM bytes                 │
│    recv(buf, len)   — inbound RTCM bytes                  │
│    status()         — CONNECTED / DISCONNECTED / ERROR    │
│                                                           │
│  Implementations:                                         │
│  ┌──────────────┐ ┌──────────────┐ ┌────────────────────┐│
│  │ NTRIP Client │ │ NTRIP Caster │ │  LoRa / Radio      ││
│  │ (Rover WiFi) │ │ (Base WiFi)  │ │  (Rover + Base)    ││
│  └──────────────┘ └──────────────┘ └────────────────────┘│
│  ┌──────────────┐                                         │
│  │ Direct WiFi  │  (ESP-NOW or TCP SoftAP peer link)      │
│  │ (local only) │                                         │
│  └──────────────┘                                         │
└───────────────────────────────────────────────────────────┘
```

### Key Design Principle
The GNSS layer never calls transport functions directly. The application layer owns the data flow, routing RTCM bytes between the GNSS layer and whatever transport is active. Swapping transports requires no changes below the application layer.

---

## 4. Hardware Design

### 4.1 Core Components

| Component | Part | Notes |
|---|---|---|
| MCU | ESP32 (WROOM-32 or S3) | Dual-core, WiFi+BT, UART×3 |
| GNSS | u-blox ZED-F9P | L1/L2, UBX+NMEA, UART2 |
| Radio (optional) | Ebyte E22-900T22S (SX1262) | 915 MHz LoRa, SPI |
| Display | 128×64 OLED (SSD1306) | I2C, status + fix quality |
| Battery | 18650 Li-ion, 1–2 cells | 3.7 V nominal |
| PMIC | TP4056 + protection board | Charge via USB-C |
| Boost/LDO | HT7333 or AMS1117-3.3 | 3.3 V rail for ESP32 |
| GNSS antenna | L1/L2 survey/patch, SMA | External mount required |
| Button(s) | Tactile, 3× minimum | Mode, Capture, Menu |
| LEDs | 3× (Power, Fix, Comms) | Optional if OLED present |
| SD card | SPI micro-SD breakout | Point log + config backup |
| USB-UART | CP2102 or CH340 | Programming + serial debug |

### 4.2 Electrical Connections

```
ESP32 Pin   Direction   Peripheral
─────────────────────────────────────────────────
GPIO16 (U2_RX)  ←       ZED-F9P UART TX
GPIO17 (U2_TX)  →       ZED-F9P UART RX
GPIO21 (SDA)    ↔       OLED SDA
GPIO22 (SCL)    →       OLED SCL
GPIO18 (SCLK)   →       LoRa SPI SCLK
GPIO23 (MOSI)   →       LoRa SPI MOSI
GPIO19 (MISO)   ←       LoRa SPI MISO
GPIO5  (CS)     →       LoRa NSS
GPIO4           →       LoRa RESET
GPIO34          ←       LoRa DIO1 (IRQ)
GPIO14          →       SD SCLK (shared or separate SPI)
GPIO13          →       SD MOSI
GPIO12          ←       SD MISO
GPIO15          →       SD CS
GPIO0           ←       Button: CAPTURE
GPIO35          ←       Button: MODE SELECT
GPIO39          ←       Button: MENU / CONFIRM
GPIO2           →       LED: FIX status
GPIO33          →       LED: COMMS status
```

**ZED-F9P UART baud rate:** 460800 bps (configured via UBX-CFG-PRT on startup).
**ZED-F9P TIMEPULSE:** GPIO25 ← PPS (optional, for future sync use).

### 4.3 Power Budget (estimate)

| State | Current |
|---|---|
| ESP32 active, WiFi on | ~200 mA |
| ESP32 active, WiFi off | ~80 mA |
| ZED-F9P tracking | ~65 mA |
| LoRa TX (22 dBm) | ~120 mA peak |
| OLED on | ~20 mA |
| **Total active (WiFi mode)** | **~285–350 mA** |
| **Total active (LoRa mode)** | **~185–250 mA** |

Single 3000 mAh 18650 cell → ~8–12 hours field use (LoRa mode).
Two cells → ~16–24 hours.

### 4.4 Enclosure

- 3D-printed PLA/PETG, ~120×70×45 mm
- SMA bulkhead for L1/L2 antenna cable
- USB-C port (charging) + USB-UART port (programming/data)
- 3× recessed buttons flush with top face
- OLED window (clear acrylic insert)
- M3 brass inserts for lid screws
- Tripod 1/4"-20 boss on base for pole or bipod mount
- Gasket groove for field weather resistance (optional IP54 seal)

---

## 5. Firmware Architecture

### 5.1 Directory Layout (esp-idf components)

```
rtk_gnss/
├── CMakeLists.txt
├── main/
│   ├── main.cpp              — app_main, mode selection, task launch
│   └── CMakeLists.txt
└── components/
    ├── gnss/                 — ZED-F9P driver + fix state machine
    │   ├── gnss_driver.c/h   — UART init, UBX send/recv, config
    │   ├── ubx_parser.c/h    — UBX frame parser
    │   ├── nmea_parser.c/h   — NMEA GGA/RMC/GST parser
    │   ├── rtcm_relay.c/h    — RTCM passthrough buffer
    │   └── fix_monitor.c/h   — RTK state machine, quality events
    ├── transport/            — Pluggable transport abstraction
    │   ├── transport.h       — transport_t interface definition
    │   ├── ntrip_client.c/h  — NTRIP client (rover WiFi)
    │   ├── ntrip_caster.c/h  — NTRIP caster (base WiFi)
    │   ├── lora_transport.c/h— LoRa radio (SX1262 via SPI)
    │   └── direct_wifi.c/h   — ESP-NOW or TCP SoftAP link
    ├── capture/              — Point capture workflow
    │   ├── capture.c/h       — Fix-gated capture state machine
    │   ├── point_store.c/h   — SD card write, JSON/CSV output
    │   └── coordinate_xform.c/h — WGS84 → State Plane projection
    ├── ui/                   — Display + button handling
    │   ├── ui.c/h            — OLED menu, status screens
    │   ├── buttons.c/h       — Debounced GPIO input
    │   └── led_status.c/h    — LED state mapping
    ├── config/               — NVS-backed settings
    │   ├── config.c/h        — Load/save to NVS
    │   └── config_defaults.h — Compile-time defaults
    └── base_station/         — Base mode logic
        ├── survey_in.c/h     — Survey-in controller
        ├── fixed_base.c/h    — Fixed-position config
        └── rtcm_broadcaster.c/h — RTCM → transport routing
```

### 5.2 RTOS Task Map

| Task | Core | Priority | Stack | Responsibility |
|---|---|---|---|---|
| `gnss_rx_task` | 1 | 20 | 4 KB | UART DMA read → UBX/NMEA parse queue |
| `gnss_monitor_task` | 1 | 18 | 3 KB | Fix state machine, event generation |
| `transport_task` | 0 | 16 | 6 KB | Transport send/recv, reconnect logic |
| `rtcm_relay_task` | 0 | 15 | 4 KB | Route RTCM: transport→GNSS (rover) or GNSS→transport (base) |
| `capture_task` | 0 | 10 | 4 KB | Button events → fix-gated point capture |
| `ui_task` | 0 | 5 | 4 KB | OLED update, button debounce |
| `sd_write_task` | 0 | 4 | 6 KB | Async SD writes from capture queue |

### 5.3 Key Data Structures

```c
/* fix_monitor.h */
typedef enum {
    RTK_NO_FIX = 0,
    RTK_SINGLE,
    RTK_FLOAT,
    RTK_FIX,
} rtk_fix_quality_t;

typedef struct {
    double latitude;        /* WGS84 degrees */
    double longitude;       /* WGS84 degrees */
    double altitude_msl;    /* metres */
    double altitude_ellipsoid;
    float  h_accuracy;      /* metres, 1-sigma */
    float  v_accuracy;
    float  pdop;
    uint8_t satellites_used;
    rtk_fix_quality_t fix_quality;
    int64_t timestamp_us;   /* esp_timer_get_time() */
    struct tm utc_time;
} gnss_fix_t;

/* transport.h */
typedef struct transport_s transport_t;
struct transport_s {
    esp_err_t (*init)(transport_t *t, const void *config);
    esp_err_t (*deinit)(transport_t *t);
    int       (*send)(transport_t *t, const uint8_t *buf, size_t len);
    int       (*recv)(transport_t *t, uint8_t *buf, size_t max_len, uint32_t timeout_ms);
    int       (*status)(transport_t *t);   /* returns transport_status_t */
    void      *priv;                       /* implementation private state */
};

/* capture/point_store.h */
typedef struct {
    gnss_fix_t  fix;
    char        point_id[32];    /* user-assigned or auto-incremented */
    char        description[128];
    double      northing;        /* projected, e.g. State Plane */
    double      easting;
    char        proj_zone[32];   /* e.g. "NAD83 / California zone 3" */
    uint32_t    avg_count;       /* fixes averaged for this point */
    float       avg_h_accuracy;  /* mean of averaged fixes */
} captured_point_t;
```

---

## 6. Rover Mode

### 6.1 Operational Flow

```
Boot → Load config (NVS) → Init GNSS (ZED-F9P, Rover cfg) →
Select transport (NTRIP / LoRa / DirectWiFi) → Connect transport →

Loop:
  ├── gnss_rx_task: receive UBX-NAV-PVT, parse fix, update fix_monitor
  ├── rtcm_relay_task: recv RTCM from transport → write to ZED-F9P UART
  ├── ui_task: display fix quality, satellite count, accuracy estimate
  └── capture_task: wait for CAPTURE button
         └── if fix_quality == RTK_FIX AND h_accuracy < threshold:
               collect N fixes (configurable, default 5) → average →
               store captured_point_t to SD → display confirmation
```

### 6.2 ZED-F9P Rover Configuration (UBX-CFG messages)

- **UBX-CFG-VALSET** at startup:
  - `CFG-UART2-BAUDRATE = 460800`
  - `CFG-UART2INPROT-RTCM3X = 1` (accept RTCM on UART2)
  - `CFG-UART2OUTPROT-UBX = 1` (output UBX on UART2)
  - `CFG-MSGOUT-UBX_NAV_PVT-UART2 = 1` (1 Hz position fix)
  - `CFG-MSGOUT-UBX_NAV_SAT-UART2 = 5` (satellite info, 0.2 Hz)
  - `CFG-MSGOUT-UBX_NAV_STATUS-UART2 = 1`
  - `CFG-NAVSPG-DYNMODEL = 2` (stationary) or `4` (automotive), configurable
  - `CFG-RTCM-DF003_IN = 0` (accept any base ID)

### 6.3 NTRIP Client Transport

- Connects to configurable NTRIP caster (host, port, mountpoint, credentials stored in NVS)
- Sends GGA sentence to caster at 5 s interval (required by most casters)
- Reconnects automatically on TCP disconnect (exponential backoff, max 30 s)
- RTCM frame sync: validate 3-byte preamble `0xD3` + 10-bit length before forwarding to GNSS

### 6.4 Point Capture Workflow

1. User presses CAPTURE button.
2. System checks: `fix_quality == RTK_FIX` AND `h_accuracy ≤ cfg.max_h_accuracy` (default 0.03 m).
3. If not met: display "WAITING FOR FIX" with current quality, blink FIX LED.
4. Once conditions met: collect `cfg.avg_count` consecutive fixes (default 5, ~5 s at 1 Hz).
5. Average latitude, longitude, altitude. Compute mean accuracy.
6. Project to configured State Plane zone (or other EPSG target) via coordinate_xform component.
7. Write `captured_point_t` to SD as a new line in `points.csv` and update `points.geojson`.
8. Display point ID and rounded easting/northing on OLED. Audible or LED confirmation.
9. Auto-increment point ID.

### 6.5 Output File Formats

**CSV (`points.csv`)** — ArcGIS-importable:
```
point_id,timestamp_utc,latitude_wgs84,longitude_wgs84,altitude_msl_m,
northing,easting,proj_zone,h_accuracy_m,v_accuracy_m,satellites,
fix_quality,avg_count,description
```

**GeoJSON (`points.geojson`)** — direct GIS import:
```json
{
  "type": "FeatureCollection",
  "crs": {"type": "name", "properties": {"name": "EPSG:4326"}},
  "features": [
    {
      "type": "Feature",
      "geometry": {"type": "Point", "coordinates": [lon, lat, alt]},
      "properties": { ...full metadata... }
    }
  ]
}
```

---

## 7. Base Station Mode

### 7.1 Operational Flow

```
Boot → Load config (NVS) → Init GNSS (ZED-F9P, Base cfg) →

Sub-mode A — Survey-In:
  survey_in_task: command CFG-TMODE3 survey-in
    (min duration: cfg.survey_min_dur, default 120 s;
     accuracy limit: cfg.survey_acc_limit, default 2.0 m)
  → poll UBX-NAV-SVIN until active=false AND valid=true
  → display progress (elapsed time, current mean position accuracy)
  → on completion: lock position, enable RTCM output

Sub-mode B — Fixed Known Position:
  fixed_base_task: write CFG-TMODE3 fixed mode
    (ECEF or lat/lon/alt loaded from NVS or entered via UI)
  → immediately enable RTCM output

RTCM output loop:
  gnss_rx_task: receive RTCM frames from ZED-F9P on UART2
  rtcm_broadcaster: forward to active transport(s)
```

### 7.2 ZED-F9P Base Configuration (UBX-CFG)

- `CFG-UART2OUTPROT-RTCM3X = 1`
- `CFG-MSGOUT-RTCM_3X_TYPE1005-UART2 = 5` (stationary RTK reference, 0.2 Hz)
- `CFG-MSGOUT-RTCM_3X_TYPE1074-UART2 = 1` (GPS MSM4, 1 Hz)
- `CFG-MSGOUT-RTCM_3X_TYPE1084-UART2 = 1` (GLONASS MSM4, 1 Hz)
- `CFG-MSGOUT-RTCM_3X_TYPE1094-UART2 = 1` (Galileo MSM4, 1 Hz)
- `CFG-MSGOUT-RTCM_3X_TYPE1124-UART2 = 1` (BeiDou MSM4, 1 Hz)
- `CFG-MSGOUT-RTCM_3X_TYPE1230-UART2 = 5` (GLONASS code-phase bias, 0.2 Hz)

### 7.3 RTCM Broadcaster

- Maintains a list of active transports (up to N simultaneously, e.g., LoRa + NTRIP caster).
- For each RTCM frame received from ZED-F9P, sends to all registered transports.
- Tracks per-transport byte rate and error count for UI display.
- NTRIP caster transport: embedded minimal HTTP/NTRIP server on port 2101, single-mountpoint.

### 7.4 Survey-In Display

```
OLED status during survey-in:
  Line 1: "BASE: SURVEY-IN"
  Line 2: "T: 00:01:47 / 00:02:00"
  Line 3: "Acc: 0.842m / 2.000m"
  Line 4: "Sats: 22  Status: ACTIVE"
```

---

## 8. Transport Layer — Implementation Details

### 8.1 NTRIP Client (Rover)

- RFC 2616 HTTP/1.1 GET to `ntrip://host:port/mountpoint`
- Authorization: Basic (base64 user:pass)
- GGA update interval: 5 s
- Reconnect: detect TCP close or 10 s recv timeout
- Sourcetable fetch: on first connect, cache mountpoint list to NVS

### 8.2 NTRIP Caster (Base)

- Embedded TCP server (esp_netif + BSD sockets)
- Responds to `GET /mountpoint HTTP/1.0` with NTRIP 1.0 handshake
- Streams RTCM frames to all connected rovers
- Handles up to 3 simultaneous rover connections
- Sourcetable served at `GET /` for NTRIP clients

### 8.3 LoRa Transport (SX1262)

- Library: custom SPI driver (or ported RadioLib for esp-idf)
- Frequency: 915 MHz (US) — configurable for 868 MHz (EU)
- Spreading factor: SF7 (default, ~5 km, ~5 kbps effective)
- Bandwidth: 125 kHz
- Coding rate: 4/5
- Max RTCM throughput: ~1200 bytes/s (sufficient for MSM4 set at ~700 bytes/s)
- Fragmentation: RTCM frames > 255 bytes split with simple sequence header
- CRC: SX1262 hardware CRC enabled; also verify RTCM CRC-24Q at receiver
- Address: base uses fixed address 0x01; rovers use configurable address

### 8.4 Direct WiFi (ESP-NOW / SoftAP)

- **ESP-NOW mode:** Base broadcasts RTCM as ESP-NOW frames (250-byte max payload with fragmentation). No WiFi association needed. Useful for <500 m outdoor range.
- **SoftAP TCP mode:** Base runs SoftAP + TCP server on port 9999. Rover connects as STA and opens TCP stream. Useful for indoor or close-range use.
- Mode selected via config.

---

## 9. Configuration System

All persistent settings stored in ESP32 NVS (non-volatile storage), with defaults in `config_defaults.h`. Configurable via:
1. **Serial CLI** over USB-UART (at boot, press any key within 3 s)
2. **BLE provisioning** (future: ESP-IDF BT stack or NimBLE)
3. **SD card** `config.json` (loaded on boot if present, overrides NVS)

### Key Configuration Parameters

```
[device]
  mode = ROVER | BASE
  device_name = "RTK-001"

[gnss]
  dynamic_model = STATIONARY | PEDESTRIAN | AUTOMOTIVE
  avg_count = 5           # fixes to average per captured point
  max_h_accuracy = 0.030  # meters, capture gate

[transport]
  type = NTRIP_CLIENT | NTRIP_CASTER | LORA | DIRECT_WIFI_ESPNOW | DIRECT_WIFI_AP
  wifi_ssid = ""
  wifi_pass = ""
  ntrip_host = ""
  ntrip_port = 2101
  ntrip_mountpoint = ""
  ntrip_user = ""
  ntrip_pass = ""
  lora_freq_mhz = 915
  lora_sf = 7

[base]
  mode = SURVEY_IN | FIXED
  survey_min_dur_s = 120
  survey_acc_limit_m = 2.0
  fixed_lat = 0.0
  fixed_lon = 0.0
  fixed_alt_m = 0.0

[capture]
  proj_epsg = 26943        # e.g., NAD83 / California zone 3 (meters)
  output_csv = true
  output_geojson = true
  point_id_prefix = "PT"
```

---

## 10. GIS Integration

### 10.1 Coordinate Projection

The `coordinate_xform` component implements a lightweight Transverse Mercator projection using the WGS84 ellipsoid parameters. For State Plane support:

- Zone parameters (central meridian, false easting/northing, scale factor, latitude of origin) are stored in a lookup table indexed by EPSG code.
- Initial release targets the most common US State Plane zones (California, Texas, Florida, NY, WA, CO) — ~20 zones.
- Accuracy: sub-millimeter for points within the valid zone extent.
- Alternatively, if linking is feasible, use `proj` library as an esp-idf component (evaluate RAM/flash budget; ~300 KB flash).

### 10.2 ArcGIS Workflow

1. Field device writes `points.geojson` and `points.csv` to SD card.
2. Field operator transfers SD card (or USB cable) to laptop.
3. ArcGIS Pro: Add Data → GeoJSON file → auto-detected WGS84 points.
4. For State Plane layers: import CSV with Northing/Easting columns; define projection as EPSG:XXXXX.
5. Attribute table includes all metadata fields for QA/QC.

### 10.3 Accuracy Metadata

Each captured point records:
- `h_accuracy_m` and `v_accuracy_m` — ZED-F9P reported accuracy (1-sigma, from UBX-NAV-PVT)
- `fix_quality` — RTK_FIX, RTK_FLOAT, or SINGLE (only RTK_FIX allowed through capture gate)
- `avg_count` — number of fixes averaged
- `pdop` — position dilution of precision

---

## 11. Development Phases

### Phase 1 — GNSS Core (Milestone: stable fix readout)
- [ ] UART2 DMA driver for ZED-F9P
- [ ] UBX frame parser (UBX-NAV-PVT, UBX-NAV-SVIN, UBX-ACK)
- [ ] UBX config writer (CFG-VALSET/VALGET)
- [ ] NMEA GGA parser (fallback / NTRIP GGA generation)
- [ ] Fix state machine (NO_FIX → SINGLE → FLOAT → FIX)
- [ ] OLED display: fix quality, lat/lon, accuracy

### Phase 2 — Rover via NTRIP (Milestone: RTK FIX over WiFi)
- [ ] WiFi STA connection (esp_wifi)
- [ ] NTRIP client transport implementation
- [ ] RTCM relay task (NTRIP → ZED-F9P)
- [ ] RTK FIX confirmation on OLED + LED
- [ ] NVS config for WiFi + NTRIP credentials

### Phase 3 — Point Capture + GIS Output (Milestone: field data collection)
- [ ] Button debounce driver
- [ ] Fix-gated capture state machine
- [ ] SD card driver (SDSPI)
- [ ] CSV + GeoJSON output
- [ ] Coordinate transform (TM projection, EPSG lookup table)
- [ ] Auto-increment point ID, description entry via UI

### Phase 4 — Base Station Mode (Milestone: working base + rover pair)
- [ ] Survey-in controller + OLED progress display
- [ ] Fixed-position base config
- [ ] RTCM broadcaster task
- [ ] NTRIP caster transport implementation
- [ ] End-to-end test: base (NTRIP caster) → rover (NTRIP client) → RTK FIX

### Phase 5 — LoRa Transport (Milestone: wireless base/rover pair, no internet)
- [ ] SX1262 SPI driver
- [ ] LoRa transport (send/recv, fragmentation, CRC)
- [ ] Range test: 1 km, 3 km, 5 km open field
- [ ] RTCM throughput validation (confirm latency < 2 s end-to-end)

### Phase 6 — Polish + Field Hardening
- [ ] Serial CLI config interface
- [ ] SD card config.json loader
- [ ] Power optimization (WiFi modem sleep when not needed)
- [ ] Watchdog on all tasks
- [ ] Direct WiFi (ESP-NOW) transport
- [ ] 3D enclosure design + print
- [ ] Field test: known benchmark vs. RTK result

---

## 12. Open Questions / Decisions Deferred

| # | Question | Options | Notes |
|---|---|---|---|
| 1 | ESP32 variant | WROOM-32 vs. S3 vs. C6 | S3 has more RAM/flash, better for LoRa + WiFi simultaneous; C6 adds 802.15.4 |
| 2 | LoRa library | Custom SPI vs. ported RadioLib | RadioLib is battle-tested but needs esp-idf port validation |
| 3 | PROJ library | Embedded lookup table vs. full PROJ port | Full PROJ ~300 KB flash; lookup table < 5 KB but limited zones |
| 4 | BLE provisioning | NimBLE vs. skip for v1 | Adds UX convenience; defer to Phase 6 |
| 5 | Multi-transport base | Single transport or broadcast to all | Design supports list; default to one for simplicity |
| 6 | RTCM message selection | MSM4 vs. MSM7 | MSM7 is higher precision but ~2× larger; SF7 LoRa may not keep up — benchmark needed |
| 7 | Averaging logic | Simple mean vs. weighted by accuracy | Weighted by 1/σ² is more correct; may be overkill at RTK FIX accuracy levels |

---

## 13. References

- u-blox ZED-F9P Integration Manual (UBX-18010802)
- u-blox ZED-F9P Interface Description (UBX-18010802-R08)
- RTCM SC-104 Standard v3.3
- NTRIP Technical Specifications v2.0 (BKG)
- ESP-IDF Programming Guide v5.x
- EPSG Geodetic Parameter Registry (epsg.org)
- Semtech SX1262 Datasheet
