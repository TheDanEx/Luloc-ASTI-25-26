# Main Doc

## Project Overview

Luloc-ASTI is a university mobile robotics project (ASTI Challenge 2025/26) with two main subsystems:
- **ESP32-P4** firmware for real-time motor control, sensors, and telemetry (ESP-IDF, FreeRTOS, C)
- **Raspberry Pi 5** software stack for perception, navigation, and monitoring (Docker Compose, Python, ROS2)

Communication between ESP32-P4 and RPi5 is via **MQTT over Ethernet**.

## Repository Structure

- `01-firmware/ESP32-P4-ETH/` — ESP-IDF project (target: ESP32-P4)
- `02-software/` — Docker services for RPi5 (compose file + per-service directories)
- `03-docs/` — Living documentation (must be kept in sync with code changes)
- `04-assets/` — External files and media

## Build & Run Commands

### Firmware (ESP-IDF)
```bash
cd 01-firmware/ESP32-P4-ETH
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyUSBx flash monitor   # flash and open serial monitor
idf.py menuconfig                       # configure SDK options
```

### Software (Docker on RPi5)
```bash
cd 02-software
docker compose up -d --build    # start all services
docker compose down             # stop all services
docker compose logs -f <service>  # follow logs for a service
```

Active services: `message-broker` (Mosquitto:1883), `influx-db` (InfluxDB 2.7:8086), `telegraf`, `grafana` (:3000). Vision and navigation containers exist but are currently commented out.

Environment variables are in `02-software/.env`.

## Architecture

### Firmware — Dual-Core FreeRTOS Tasks
- **Core 0 (HP — Real-time):** `task_rtcontrol_cpu0` — motor PID, encoder reads, critical reactions
- **Core 1 (HP — Logic/Comms):** `task_comms_cpu1` — complex sensors (IMU, current), MQTT parsing, data prep
- **LP Core:** `task_monitor_lowpower_cpu1` — battery monitoring, watchdogs

Entry: `main/app_main.c` → `system_init.c` initializes all components then spawns tasks from `main/tasks/`.

### Firmware Components (in `components/`)
Key modules: `motors`, `encoder_sensor`, `telemetry_manager`, `mqtt_custom_client`, `shared_memory`, `state_machine`, `ethernet`, `logger`, `audio_player`, `performance_monitor`, `curvature_feedforward`, `test_sensor`

Each component has corresponding documentation in `03-docs/firmware/components/<name>.md`.

### Telemetry Pipeline
ESP32 → MQTT (Influx Line Protocol batched) → Mosquitto → Telegraf → InfluxDB → Grafana

Telemetry uses **batched Influx Line Protocol** (not individual JSON messages). The `telemetry_manager` accumulates high-frequency samples with microsecond timestamps (`esp_timer_get_time()`) and publishes in bulk at configurable intervals (500ms–2000ms). Developer-facing API (`telemetry_add_float`, etc.) is unchanged — batching happens internally.

### Software Services
- **01-vision:** OpenCV + GStreamer + libcamera (Python, CSI camera IMX219)
- **02-navigation:** ROS2 Humble + Nav2 + LiDAR D200 (SLAM, path planning)
- **03-main-controller:** Robot state machine, ROS2↔MQTT bridge, FastAPI backend
- **05-telemetry:** Telegraf config + Grafana provisioning/dashboards
- **06-mqtt-broker:** Mosquitto configuration

## Mandatory Conventions

- **All naming in English:** files, folders, variables, functions, modules. Only prose/documentation content may be in Spanish.
- **Git commits in English** using conventional commits: `feat:`, `fix:`, `docs:`, `refactor:`, etc.
- **C naming:** `snake_case` for functions (prefixed by module, e.g., `motor_mcpwm_set`), `UPPER_SNAKE_CASE` for macros/constants, descriptive state variables (e.g., `is_mqtt_connected`).
- **Documentation sync is mandatory:** Any code change requires updating the corresponding `03-docs/` markdown files. Read existing docs before modifying a component.
- **Minimal comments:** Code should be self-documenting. Only add brief (1–2 line) comments for genuinely complex blocks (quaternion math, register hacks), explaining *why*, not *what*.
- **Firmware error handling:** Use `ESP_ERROR_CHECK()` only during init. In runtime loops, capture `esp_err_t` and handle gracefully (log/telemetry), never crash the chip.
- **No ISR blocking:** Never use blocking calls in ISRs or priority-10 loops. Delegate via `xQueueSendFromISR`.
- **Docker immutability:** Service config must come from `.env` variables, not hardcoded URIs.

## Git Workflow

- Branch naming: `<team>/<contributor>/main` (e.g., `Luloc/TheDanEx/main`, `PabloMotos/MCU-Karman/main`)
- A GitHub Actions workflow auto-creates sync PRs from `main` to contributor branches on push to `main`
- PRs merge contributor branches into `main`
