# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-IDF project for testing the EBYTE E22 LoRa module on an ESP32. Uses the standard ESP-IDF CMake build system.

## Build Commands

Requires ESP-IDF to be installed and the `IDF_PATH` environment variable set. Source the ESP-IDF environment first:

```sh
. $IDF_PATH/export.sh
```

| Task | Command |
|------|---------|
| Configure target chip | `idf.py set-target esp32` (or `esp32s3`, etc.) |
| Build | `idf.py build` |
| Flash | `idf.py -p /dev/ttyUSB0 flash` |
| Monitor serial output | `idf.py -p /dev/ttyUSB0 monitor` |
| Flash + monitor | `idf.py -p /dev/ttyUSB0 flash monitor` |
| Clean | `idf.py fullclean` |
| menuconfig | `idf.py menuconfig` |

## Project Structure

- `CMakeLists.txt` — top-level ESP-IDF project file; sets project name `e22_test`
- `main/e22_test.cpp` — entry point; `app_main()` is the ESP-IDF application entry function (equivalent to `main()`); declared `extern "C"` so the linker can find it from C code
- `main/CMakeLists.txt` — registers the `main` component with ESP-IDF

## Architecture Notes

ESP-IDF uses a component-based architecture. Application logic lives in `main/` (itself a component). Additional components go in a top-level `components/` directory. The build system discovers them automatically.

The `app_main()` function runs in a FreeRTOS task. It must not return; use an infinite loop or `vTaskDelete(NULL)` at the end if the task completes.
