This is a comprehensive system design. I'll create a detailed technical reference document with multiple diagrams covering every layer of the architecture.Here is the complete system design. Click any block in the diagram to drill deeper into a topic.


## System overview

This is a dual-mode RTK GNSS platform built around the u-blox ZED-F9P and ESP32. The architecture has three clean separation planes: the GNSS core (talks only to the F9P), the transport layer (pluggable communications), and the application logic (mode management, capture workflow, GIS output). Swapping from NTRIP to LoRa to direct WiFi requires no changes to positioning logic.


## 1. Hardware design

### Bill of materials

| **Component** | **Part** | **Notes** |
| :-: | :-: | :-: |
| GNSS module | u-blox ZED-F9P breakout | SparkFun GPS-16481 or ArduSimple simpleRTK2B |
| MCU | ESP32-S3 DevKitC | Dual-core, 8MB PSRAM, USB-OTG |
| Antenna | Tallysman TW3710 or u-blox ANN-MB-00 | Dual-band L1/L2/L5, SMA |
| Radio (optional) | Ebyte E22-900T22D (SX1262, 915 MHz) | Or RFM95W for LoRa |
| Display | SSD1306 0.96" OLED | I2C |
| Battery | 3.7V 4000 mAh LiPo | With TP4056 + DW01 protection |
| Charge IC | IP5306 or BQ24073 | USB-C PD preferred |
| Voltage reg | 3.3V LDO (AMS1117) | For F9P + ESP32 |

### Wiring

```
`ZED-F9P                     ESP32-S3`

`─────────────────────────────────────────────────`

`TXD1 (UART1 out)    →       GPIO16 (UART1 RX)`

`RXD1 (UART1 in)     ←       GPIO17 (UART1 TX)`

`3V3                 ←       3V3`

`GND                 ─        GND`

`SDA (I2C alt)       ↔       GPIO8  (optional)`

`SCL (I2C alt)       ↔       GPIO9  (optional)`

`TIMEPULSE           →       GPIO5  (PPS, optional)`


`SSD1306 OLED                ESP32-S3`

`─────────────────────────────────────────────────`

`SDA                 ↔       GPIO21`

`SCL                 ↔       GPIO22`


`LoRa E22/RFM95               ESP32-S3`

`─────────────────────────────────────────────────`

`MOSI                ←       GPIO11 (SPI)`

`MISO                →       GPIO13`

`SCK                 ←       GPIO12`

`NSS/CS              ←       GPIO10`

`RST                 ←       GPIO6`

`DIO0/IRQ            →       GPIO7`


`Buttons                     ESP32-S3`

`─────────────────────────────────────────────────`

`BTN\_MODE            →       GPIO1  (BOOT/mode)`

`BTN\_CAPTURE         →       GPIO2`

`BTN\_CONFIG          →       GPIO3`
```

All UART runs at **460800 baud** between ESP32 and F9P — fast enough for full RTCM3 + UBX throughput without overflow.


## 2. ESP32 firmware architecture

The firmware uses FreeRTOS with four independent tasks communicating through queues and shared protected state.\#\#\# Core firmware modules

**`gnss\_core.c` — GNSS abstraction**

```
`// gnss\_core.h`

`typedef struct \{`

`    double lat, lon, alt\_msl;`

`    float accuracy\_h, accuracy\_v;   // meters (1-sigma)`

`    uint8\_t fix\_type;               // 0=none,1=DR,2=2D,3=3D,4=GNSS+DR,5=time`

`    uint8\_t carr\_soln;              // 0=none,1=float,2=FIX (RTK)`

`    uint8\_t num\_sv;`

`    uint32\_t i\_tow;`

`    int16\_t pdop;`

`    float hdop;`

`    bool gnss\_fix\_ok;`

`\} gnss\_pvt\_t;`


`typedef enum \{ MODE\_BASE, MODE\_ROVER \} device\_mode\_t;`


`// Public API`

`void gnss\_init(uart\_port\_t port, int baud);`

`void gnss\_configure\_rover(void);`

`void gnss\_configure\_base\_surveyin(uint32\_t min\_dur\_s, float min\_acc\_m);`

`void gnss\_configure\_base\_fixed(double lat, double lon, double alt);`

`bool gnss\_read\_pvt(gnss\_pvt\_t \*out);`

`void gnss\_inject\_rtcm(const uint8\_t \*buf, size\_t len);`

`bool gnss\_is\_rtk\_fix(const gnss\_pvt\_t \*pvt);`
```

The GNSS core speaks only UBX binary. All configuration uses `CFG-VAL` (generation 9 interface) commands. Key messages to enable:

```
`// Messages to enable on F9P via UBX-CFG-VALSET`

`// Rover mode`

`UBX\_CFG\_MSGOUT\_UBX\_NAV\_PVT\_UART1 = 1    // 1 Hz PVT output`

`UBX\_CFG\_MSGOUT\_UBX\_NAV\_STATUS\_UART1 = 1`

`UBX\_CFG\_MSGOUT\_NMEA\_ID\_GGA\_UART1 = 1     // for NTRIP GGA feedback`

`UBX\_CFG\_RATE\_MEAS = 200                   // 5 Hz measurement rate`

`UBX\_CFG\_NMEA\_HIGHPREC = 1                // 8 decimal degree output`


`// Base mode — adds:`

`UBX\_CFG\_MSGOUT\_UBX\_RXM\_RTCM\_UART1 = 1   // RTCM input monitor`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1005\_UART1 = 1`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1077\_UART1 = 1  // GPS MSM7`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1087\_UART1 = 1  // GLONASS MSM7`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1097\_UART1 = 1  // Galileo MSM7`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1127\_UART1 = 1  // BeiDou MSM7`

`UBX\_CFG\_MSGOUT\_RTCM\_3X\_TYPE1230\_UART1 = 5  // GLONASS code-phase bias`
```

**`transport.c` — pluggable interface**

```
`// transport.h — the only interface any module calls`

`typedef struct \{`

`    esp\_err\_t (\*init)(void \*cfg);`

`    esp\_err\_t (\*send)(const uint8\_t \*buf, size\_t len);`

`    int       (\*recv)(uint8\_t \*buf, size\_t max\_len, uint32\_t timeout\_ms);`

`    void      (\*deinit)(void);`

`    const char \*name;`

`\} transport\_t;`


`// Concrete implementations registered at build time:`

`// transport\_ntrip\_client.c  — rover WiFi corrections`

`// transport\_ntrip\_server.c  — base caster mode`

`// transport\_lora.c          — SX1262 radio bridge`

`// transport\_wifi\_ap.c       — base as WiFi AP, TCP server`

`// transport\_wifi\_sta.c      — rover connects to base AP`


`// Runtime selection`

`void transport\_set(const transport\_t \*t, void \*cfg);`
```


## 3. Base station configuration**Survey-in configuration (UBX-CFG-TMODE3):**

```
`// Minimum 300 seconds, 2.0 m 3D accuracy requirement`

`// For production base: use 3600 s + 0.5 m, or use a known benchmark`

`ubx\_cfg\_tmode3\_t cfg = \{`

`    .version = 0,`

`    .flags = 0x0001,             // survey-in mode`

`    .svin\_min\_dur = 300,         // seconds`

`    .svin\_acc\_limit = 20000,     // 0.1 mm units = 2.0 m`

`\};`
```

For a known benchmark: use `flags = 0x0002` (fixed mode) and provide ECEF coordinates from a geodetic survey or OPUS solution. This eliminates the ~decimeter error introduced by an imperfect survey-in and produces true geodetic accuracy.


## 4. Rover configuration and point capture workflow

```
`// rover\_capture.c — point averaging state machine`

`typedef enum \{`

`    CAPTURE\_IDLE,`

`    CAPTURE\_WAITING\_FIX,    // wait for carr\_soln == 2`

`    CAPTURE\_AVERAGING,      // accumulate N epochs`

`    CAPTURE\_DONE,`

`    CAPTURE\_REJECTED        // fix lost during capture`

`\} capture\_state\_t;`


`\#define CAPTURE\_MIN\_EPOCHS   60    // 1 min at 1 Hz minimum`

`\#define CAPTURE\_MIN\_FIX\_PCT  95    // 95% of epochs must be RTK FIX`


`typedef struct \{`

`    char point\_id\[32\];`

`    double lat\_dd, lon\_dd;         // WGS84 decimal degrees`

`    double alt\_msl\_m;`

`    float h\_accuracy\_m;`

`    float v\_accuracy\_m;`

`    uint16\_t n\_epochs;`

`    float fix\_pct;`

`    char timestamp\_utc\[32\];        // ISO 8601`

`    char crs\_wgs84\[16\];            // "EPSG:4326"`

`    char notes\[128\];`

`    float hdop;`

`    uint8\_t num\_sv;`

`\} captured\_point\_t;`
```

The capture workflow requires the operator to hold the capture button for 2 seconds to begin, displays live accuracy on the OLED, and rejects the point if RTK FIX drops below 95% during averaging. Accepted points are appended to a GeoJSON file on SPIFFS and optionally transmitted via BLE to a collector app.


## 5. Transport layer implementations

### NTRIP client (rover mode)

```
`// ntrip\_client.c — key sequence`

`// 1. Connect TCP to caster (e.g. rtk2go.com:2101)`

`// 2. Send HTTP/1.1 GET with base64 credentials`

`// 3. Parse sourcetable, select nearest mountpoint`

`// 4. Stream RTCM frames → inject to F9P`

`// 5. Send GGA every 10 s for distance-based correction blending`


`static const char \*NTRIP\_REQUEST =`

`    "GET /%s HTTP/1.1\\r\\n"`

`    "Host: %s\\r\\n"`

`    "Ntrip-Version: Ntrip/2.0\\r\\n"`

`    "User-Agent: NTRIP ESP32RTK/1.0\\r\\n"`

`    "Authorization: Basic %s\\r\\n"`

`    "\\r\\n";`
```

### LoRa transport (base ↔ rover)

```
`// lora\_transport.c`

`// Base: reads RTCM3 frames from F9P, packetizes, transmits at 1 Hz`

`// Rover: receives packets, reassembles frames, injects to F9P`


`// LoRa settings for 10+ km RTCM relay:`

`lora\_config\_t lora\_cfg = \{`

`    .frequency\_hz  = 915000000,    // 915 MHz (US) or 868 MHz (EU)`

`    .bandwidth     = BW\_125kHz,    // balance range vs throughput`

`    .spreading     = SF\_10,        // ~980 bps — enough for RTCM`

`    .coding\_rate   = CR\_4\_5,`

`    .tx\_power\_dbm  = 22,           // max legal (check local regs)`

`    .sync\_word     = 0xAB,         // private network ID`

`    .preamble\_len  = 8,`

`\};`


`// RTCM3 packet wrapper (custom header for LoRa)`

`typedef struct \_\_packed \{`

`    uint8\_t  magic\[2\];    // 0xD3 0xC0`

`    uint16\_t seq;`

`    uint16\_t payload\_len;`

`    uint8\_t  payload\[240\]; // max per LoRa packet`

`    uint16\_t crc16;`

`\} lora\_rtcm\_frame\_t;`
```

For RTCM3 over LoRa, budget ~500 bytes/second typical load (GPS + GLONASS MSM7). At SF10/BW125 you get ~980 bps on-air, so this is workable with minimal fragmentation if you prioritize message 1005 and alternate GPS/GLONASS.


## 6. GIS output format

### GeoJSON point log (on-device SPIFFS)

```
`\{`

`  "type": "FeatureCollection",`

`  "crs": \{ "type": "name", "properties": \{ "name": "EPSG:4326" \} \},`

`  "features": \[`

`    \{`

`      "type": "Feature",`

`      "geometry": \{ "type": "Point", "coordinates": \[-89.6501234, 40.9512345, 228.45\] \},`

`      "properties": \{`

`        "id": "PT001",`

`        "timestamp\_utc": "2025-08-14T14:23:11Z",`

`        "h\_accuracy\_m": 0.012,`

`        "v\_accuracy\_m": 0.021,`

`        "fix\_type": "RTK\_FIX",`

`        "carr\_soln": 2,`

`        "n\_epochs": 60,`

`        "fix\_pct": 100,`

`        "hdop": 0.8,`

`        "num\_sv": 24,`

`        "base\_dist\_km": 8.4,`

`        "antenna\_ht\_m": 2.000,`

`        "operator": "J.Smith",`

`        "notes": "NW corner property pin"`

`      \}`

`    \}`

`  \]`

`\}`
```

### State Plane / projected output

The ESP32 cannot run a full PROJ transform natively. Two options:

**Option A:** Output WGS84 + epoch timestamp; reproject in ArcGIS or QGIS using the GeoJSON file. Import with the "Add Data" dialog, then use "Project" (Data Management Tools) to NAD83 State Plane.

**Option B:** Implement Helmert 7-parameter transform on-device for a specific State Plane zone (lightweight — only needs 7 constants + grid constants for the zone). Publish a second CSV column with Easting/Northing in meters.

```
`// Approximate Transverse Mercator for State Plane (Illinois East: EPSG:26971)`

`// Use for on-device preview only — full PROJ accuracy requires post-processing`

`void wgs84\_to\_statplane\_il\_east(double lat\_dd, double lon\_dd,`

`                                 double \*e\_m, double \*n\_m);`
```


## 7. Enclosure and field deployment

**3D-printed enclosure design recommendations:**

The enclosure needs two chambers: an electronics bay (watertight, IP65) and an antenna pass-through port. Print in PETG for UV resistance in field conditions. Design for an SMA bulkhead connector flush-mounted to the top face, with the GNSS antenna on a short mast above or mounted to a survey pole via 5/8-11 thread adapter.

```
`Top view (approximate):`

`┌──────────────────────────────┐`

`│  \[SMA\]  \[LED: R/G/B status\]  │`

`│  \[USB-C port\]                │`

`│  \[BTN1\] \[BTN2\] \[BTN3\]        │`

`│  \[OLED 0.96"\]                │`

`│  \[LiPo charge indicator\]     │`

`└──────────────────────────────┘`


`Side: LoRa SMA antenna port (if radio installed)`

`Bottom: 1/4"-20 or 5/8"-11 tripod mount boss`
```

Use a PRY-SAFE silicone gasket around the lid, and Dow 795 silicone at all penetrations. Desiccant packet in the electronics bay.


## 8. Best practices for centimeter-level accuracy

**Base station:**

- Place on a stable, known benchmark or observe for ≥2 hours for OPUS post-processing of position 

- Antenna phase center offset matters — use a geodetic choke-ring or a well-characterized patch antenna (TW3710 spec sheet gives L1/L2 offsets) 

- Keep baseline distance under 30 km for L1/L2 RTK; beyond that, atmospheric errors degrade convergence time 

- Transmit RTCM 1230 (GLONASS code-phase biases) — required for mixed-receiver networks 

**Rover:**

- Wait for `carr\_soln == 2` (RTK FIX) before starting point capture — float (`carr\_soln == 1`) introduces 10–50 cm errors 

- Average a minimum of 60 epochs (60 seconds at 1 Hz); for control-quality work use 180 epochs 

- Record `h\_accuracy` from UBX-NAV-PVT, not HDOP — the F9P's internal covariance estimate is far more useful 

- Mount antenna at a fixed, measured height above the point; record in `antenna\_ht\_m` metadata 

- Avoid multipath: keep away from vehicles, buildings, and reflective surfaces; look for open sky above 10° elevation 

**Communications:**

- For NTRIP: select the nearest mountpoint within 30 km; use RTCM3 MSM7 not MSM4 for higher accuracy 

- For LoRa: use confirmed delivery with ACK (the correction task should detect stale corrections \>5 s and flag a warning on the OLED) 

- Monitor `UBX-RXM-RTCM` messages to confirm corrections are being decoded by the F9P, not silently dropped 

**Quality control flags in log:**

```
`// Reject any captured point where:`

`pvt.carr\_soln != 2             // not RTK FIX`

`pvt.h\_accuracy \> 0.050         // \> 5 cm horizontal`

`pvt.fix\_pct \< 0.95             // lost fix during capture`

`pvt.num\_sv \< 10                // too few satellites`

`age\_of\_corrections\_s \> 10      // stale base corrections`
```

This architecture gives you a complete, field-deployable RTK GNSS platform. The transport abstraction means you can add a cellular (LTE-M) transport for national CORS access without touching the positioning or GIS logic. Each layer is independently testable — the GNSS core can be exercised against a serial playback of recorded UBX frames, and the transport layer can be swapped to a loopback for integration testing before going to field.

