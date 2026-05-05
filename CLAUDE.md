# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This project uses **ESP-IDF 5.x** (not PlatformIO). All build commands require the IDF environment to be sourced first. The VS Code ESP-IDF extension handles this automatically; in a terminal, run `. $IDF_PATH/export.sh` first.

```bash
idf.py build                   # compile firmware
idf.py flash                   # flash to connected ESP32-S3
idf.py monitor                 # open serial monitor (115200 baud)
idf.py flash monitor           # flash and immediately monitor
idf.py menuconfig              # interactive Kconfig configuration
idf.py fullclean               # wipe build directory
```

Target is **ESP32-S3** (configured in sdkconfig / `.vscode/settings.json`).

The devcontainer at `.devcontainer/` provides a reproducible environment with `espressif/idf:latest` if the local toolchain is not set up.

## Architecture

### Dual-Mode System

A single firmware binary runs on both the **Rover** and **Base Station** units. Mode is selected at runtime via NVS configuration or button press at boot. The design documentation lives in `docs/PDD.md` (product design) and `docs/rtk_gnss.md` (implementation guide).

### Three-Layer Architecture

```
Application (rover_app / base_app)
    │
    ├── gnss/          UBX driver, RTK state, gnss_pvt_t struct
    ├── transport/     Pluggable NTRIP / LoRa / Direct-WiFi interface
    ├── capture/       Point-averaging state machine, SD output (CSV/GeoJSON)
    ├── ui/            OLED menus, button debounce, LED status
    ├── config/        NVS-backed settings, serial CLI
    └── base_station/  Survey-in, fixed-position config, RTCM broadcast
```

The `transport/` layer exposes a `transport_t` vtable interface. GNSS core and application code never call a concrete transport directly — they call through the interface so implementations (NTRIP client, LoRa, ESP-NOW) are interchangeable.

### FreeRTOS Task Model

Seven concurrent tasks: `gnss_rx`, `gnss_monitor`, `transport`, `rtcm_relay`, `capture`, `ui`, `sd_write`. Tasks communicate via FreeRTOS queues; no direct function calls across task boundaries.

### Key Data Structures (defined in docs; implement as written)

- `gnss_pvt_t` — raw position/velocity/time from ZED-F9P
- `gnss_fix_t` — processed fix with RTK status flags
- `transport_t` — vtable interface struct for transport abstraction
- `captured_point_t` — averaged point with accuracy metadata for GIS output

### GNSS Communication

ZED-F9P communicates over UART2 at **460800 baud** using the **UBX binary protocol**. RTCM3 correction messages flow over the same or a separate UART. All UBX message construction and parsing belongs in `gnss/`.

### Rover Capture Workflow

Point capture requires a **minimum 60-epoch average** with **≥95% RTK Fix** epochs. Output is GeoJSON (primary) and CSV, written to SD card over SPI. Coordinates are WGS84 plus a projected CRS for ArcGIS compatibility.

### Base Station Modes

1. **Survey-in**: autonomous convergence until position σ < 2 m over ≥300 s
2. **Fixed**: operator-supplied ECEF or LLH coordinates written via TMODE3

### Transport Implementations

| Transport | Use Case | Notes |
|-----------|----------|-------|
| NTRIP client | Rover receives RTCM from internet caster | WiFi required |
| NTRIP caster | Base serves RTCM over WiFi | SoftAP mode |
| LoRa SX1262 | Base→Rover in the field | SF10/BW125, Ebyte E22-900T22S |
| Direct WiFi | ESP-NOW or SoftAP fallback | short range |

## Hardware Targets

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | ESP32-S3 | — |
| GNSS | ZED-F9P-04B or 05B | UART1 @ 460800, TX=GPIO4, RX=GPIO17 |
| LoRa | Ebyte E22-900T22S (SX1262) | SPI |
| Display | SSD1306 128×64 OLED | I2C |
| SD card | Micro-SD | SPI |

See `docs/rtk_gnss.md` for complete GPIO pin assignments.

## Development Status

The `gnss` component is complete and verified on hardware. The ZED-F9P delivers UBX NAV-PVT at 1 Hz over UART1 (TX=GPIO4, RX=GPIO17) with auto-baud detection, bidirectional TX verification, and RTK status decoding. Standalone 3D fix confirmed (~3 m hAcc). Next phase is RTK correction ingestion via the `transport` component (NTRIP client or LoRa).

Follow the six-phase roadmap in `docs/PDD.md` when adding components. The PDD and implementation guide are the authoritative source of truth for what to build.

## Code Conventions

- Components go under `components/<name>/` following ESP-IDF component layout (`CMakeLists.txt`, `include/`, `*.c`).
- Mixed C/C++: prefer C for components, C++ is acceptable in `main/`.
- Keep UBX protocol details (message IDs, field offsets) isolated in `gnss/` — nothing outside that component should reference UBX constants.
