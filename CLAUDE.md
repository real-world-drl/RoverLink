# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP-IDF v5.1 firmware for the **lower computer** on a Waveshare UGV
(General Driver board, ESP32 WROOM-32). User-owned rewrite of the stock
Arduino firmware at `../ugv_base_general` (upstream reference only —
not editable, not pushable). Talks to an upper computer (Pi 5, planned)
over binary MQTT.

For build instructions, Kconfig reference, MQTT contract, and bring-up
checklist, see `README.md` — don't duplicate it here.

## Build / verify

```bash
. ~/esp/esp-idf-5.5.1/export.sh
idf.py build                         # CI-style verification
idf.py -p /dev/ttyUSB0 flash monitor # real test = flash + watch logs
```

There are no host-side tests. "Verified" means it builds and the right
log lines appear at boot when flashed.

## Architecture, in one paragraph

`app_main.c` spawns five FreeRTOS tasks: `control_task` (100 Hz, core 1
— drains cmd queues, applies kinematics, runs PID or open-loop PWM,
drives motors), `telemetry_task` (50 Hz, core 0 — fans out wheel
telem to all enabled transports; once a second also reads INA219 and
prints the encoder GPIO diagnostic), `imu_task` (100 Hz, core 0 —
reads QMI8658+AK09918, fans out `tel/imu`, feeds firmware odometry),
`display_task` (1 Hz, core 0 — renders OLED with host data if recent,
else firmware odometry), and `uart_rx_task` (core 0 — UART2 framing
FSM, pushes parsed commands into the shared queues). Two transports
feed those queues: MQTT (over WiFi, in `comms.c`) and the UART link
(`uart_link.c`); either or both can be disabled via Kconfig. Sensor
drivers (`ina219`, `qmi8658`, `ak09918`, `imu`, `oled`) sit on a shared
`i2c_bus` at 400 kHz on GPIO 32/33.

## Things to know before changing code

### `main/ugv_packets.h` is the wire contract
Packed little-endian structs, shared with the host. Don't reorder
fields, don't change types, don't change sizes silently — every struct
has a `_Static_assert` size guard so the build fails if you do. When
you legitimately need to evolve a packet, bump the topic version
(`UGV_TOPIC_VERSION` → `"v2"`) rather than breaking v1 in place.

### Kconfig bools emit no macro when `n`
A `bool` with `default n` produces *no* `CONFIG_X` define at all — not
`CONFIG_X=0`. Always test with `#ifdef CONFIG_X` / `#if defined(...)`,
never `if (CONFIG_X)`. The pattern of promoting bools to `1`/`0` macros
at the top of a file (see `motor.c`, `encoder.c`) keeps the call sites
readable.

### New Kconfig options don't auto-appear in existing sdkconfig
Adding an option to `Kconfig.projbuild` and running `idf.py build` is
*not* enough to make it materialize in `sdkconfig` — `build` only
reconfigures if something tells it to. Run `idf.py reconfigure`
explicitly, or directly edit `sdkconfig` and then build. If you find
yourself debugging "my new default isn't taking effect," check that
the option actually appears in `sdkconfig`.

### IMU body-frame inversion is display-only
`CONFIG_UGV_IMU_BODY_INVERT_X/Y/Z` are applied only in `app_main.c`'s
`imu_task` on the path that feeds `odometry_update_imu()`. The
published `tel/imu` packet contains raw chip-frame readings unchanged —
the host runs its own calibration there. Don't accidentally apply the
flags on both paths, or to the published data.

### `pid_t` is taken by `<sys/types.h>`
POSIX `pid_t` (process id) gets pulled in transitively by IDF, so the
PID controller's type is named `ugv_pid_t`, not `pid_t`. Same caution
applies if you introduce any other generic-sounding C-library-adjacent
names.

### Two transports share one set of command queues
`comms.c` owns `s_cmd_vel_q` / `s_cmd_pid_q` / `s_cmd_display_q` (each
capacity 1, overwrite-mode). Both MQTT (`comms.c`) and UART
(`uart_link.c`) call `comms_push_cmd_*` to deliver received commands.
`xQueueOverwrite` is thread-safe so concurrent pushes from the two
transports are fine at the queue level — but **a stale MQTT cmd_vel
arriving 400 ms after a fresh UART one would otherwise stomp the
setpoint**. The mitigation is in `comms_push_cmd_vel(v, from_uart)`:
when `from_uart == false` and `uart_link_recent_rx()` is true, the MQTT
push is dropped. Window controlled by `CONFIG_UGV_UART_PREEMPT_MS`
(default 1 s). If you add a third transport, follow the same pattern.

### Queue init must happen before any transport starts
`comms_queues_init()` is unconditional in `app_main` and runs before
both `comms_init()` (MQTT) and `uart_link_init()` (UART). This is what
makes `CONFIG_UGV_ENABLE_MQTT=n` clean — queues exist regardless of
which transports are compiled in.

### Telemetry fan-out is explicit, not abstracted
Three call sites in `telemetry_task` / `imu_task` each have a pair of
`#ifdef`'d publish calls — one for MQTT, one for UART. If you add a
third transport, add a third `#ifdef`'d block at each call site rather
than building a fan-out abstraction. It's grep-able and the cost is
~6 lines.

### UART framing is `[0xA5][type:1][len:1][payload:N][crc8:1]`
CRC8 poly 0x07, init 0x00. The length byte is technically redundant
with the type → size table but catches host/firmware version skew and
the resync edge case where a payload byte happens to look like 0xA5.
Reuse `tools/ugv_packets.py` from the host side — it has the same
constants and a matching CRC8. Don't add a CRC32 or a different polynomial.

### Heartbeat watchdog is in `control_task`
`set_lin`/`set_ang` are zeroed when `now - last_cmd_us >
HEARTBEAT_TIMEOUT_US`. Anything that injects motion commands needs to
update `last_cmd_us = now_us` — currently only the `cmd/vel` handler
does. Don't add a side path that drives motors without resetting that
timestamp.

### Open-loop mode is the current default
`CONFIG_UGV_OPEN_LOOP=y` because the user's current bot has no
encoders (verified via firmware diagnostic — pins never toggle). In
this mode PID is skipped entirely and `cmd_vel → wheel m/s → ×scale →
PWM`. Firmware odometry feeds setpoints (not measurements) so the OLED
still trends. When closed-loop is restored later, the IF-branch in
`control_task` flips automatically — no other code changes needed.

### Hardware quirks already accounted for
- **OLED is 128×32**, not 128×64 — wrong panel-size init makes output
  garbled-but-recognizable.
- **INA219 reads battery voltage directly** (no divider, despite an
  earlier wrong guess). Scale Kconfig is at 1.0×.
- **GPIO 34/35 are input-only with no internal pull-ups** — the
  `gpio_pullup_en: GPIO number error` lines at boot are from the PCNT
  driver trying to enable them on the left-encoder pins. Cosmetic, no
  fix possible from the public API.
- **LEDC duty must not be written before LEDC is configured** — that's
  why `motor_init` sets direction pins low explicitly instead of
  calling `motor_stop_all()` before the LEDC timer/channels exist.

### The Arduino repo next door
`../ugv_base_general/` is the upstream Waveshare firmware in Arduino.
Useful **as reference** for pin assignments, INA219 config, IMU register
sequences, and which OLED is on the board. Do not edit it, do not
attempt to flash it, and do not copy its `#include`-everything header
style — it's a single translation unit with shared globals across .h
files, an antipattern this project deliberately avoids.

## Memory & user context

The user prefers ESP-IDF over Arduino, packed-binary MQTT over JSON,
and telemetry done with cumulative counts + sequence numbers + device
timestamps so the host can integrate without inheriting firmware
jitter. They are fluent in ESP-IDF — no need to explain `esp_event`,
FreeRTOS pinning, or NVS basics. The upper computer is a Pi 5 (planned)
running iPhone VIO for authoritative pose; firmware odometry is just
for the OLED.
