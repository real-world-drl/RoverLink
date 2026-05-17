# ugv_idf

ESP-IDF firmware for the Waveshare General Driver board (ESP32 WROOM-32),
running as the lower computer on a UGV. Speaks binary MQTT to an upper
computer (Raspberry Pi 5, planned, with iPhone VIO for pose). Replaces
the stock Arduino firmware in the upstream `ugv_base_general` repo.

## Build / flash / monitor

```bash
. ~/esp/esp-idf-5.5.1/export.sh
idf.py -p /dev/ttyUSB0 flash monitor
```

A plain `idf.py build` is enough once env is sourced — both `xtensa-esp32-elf`
and `xtensa-esp32s3-elf` are now registered in `~/.espressif/idf-env.json`,
so `export.sh` puts the correct toolchain on PATH.

The first time you build on a new machine you'll need to install the
ESP32 toolchain:
```bash
python $IDF_PATH/tools/idf_tools.py install --targets esp32
```
The `--targets esp32` flag (vs. `install xtensa-esp32-elf` by tool name)
updates `idf-env.json` so `export.sh` will pick it up automatically.

## Configuration

All tunables live under `idf.py menuconfig` → **UGV firmware**.

### Identity & networking
| Option | Purpose |
|---|---|
| `UGV_ROBOT_ID` | Substituted into all MQTT topics: `ugv/<id>/v1/...` |
| `UGV_WIFI_SSID`, `UGV_WIFI_PASS` | WiFi STA credentials |
| `UGV_WIFI_MAX_RETRY` | Reconnect attempts before giving up |
| `UGV_MQTT_BROKER_URI` | e.g. `mqtt://mqtt-server:1883` |
| `UGV_MQTT_USERNAME`, `UGV_MQTT_PASSWORD` | Blank = anonymous |

### Control mode
| Option | Default | Purpose |
|---|---|---|
| `UGV_OPEN_LOOP` | n | When y, skips PID entirely — cmd_vel maps directly to PWM. Use when the bot has no encoders. |
| `UGV_OPEN_LOOP_PWM_PER_MPS_X10` | 5120 | PWM counts per m/s of commanded velocity, ×10. Default 512.0 matches stock Arduino's open-loop scaling — saturates PWM at ~0.5 m/s commanded. |

### Robot kinematics
| Option | Default | Notes |
|---|---|---|
| `UGV_WHEEL_DIAMETER_MM` | 80 | |
| `UGV_TRACK_WIDTH_MM` | 172 | Wheel separation. |
| `UGV_ENCODER_PPR` | 1650 | After gearing, 2x quadrature decode. Only used in closed-loop. |
| `UGV_MOTOR_LEFT_INVERT` / `_RIGHT_INVERT` | n | Flip if a wheel spins the wrong way at `--linear 0.1`. |
| `UGV_ENCODER_LEFT_INVERT` / `_RIGHT_INVERT` | n | Flip if tick count goes backwards when wheel rolls forward by hand. |
| `UGV_MAX_LINEAR_MPS_X100` | 200 (= 2.0 m/s) | Clamp on incoming cmd_vel. |
| `UGV_MAX_ANGULAR_RPS_X100` | 600 (= 6.0 rad/s) | Clamp on incoming cmd_vel. |

### PID (closed-loop only)
| Option | Default | Notes |
|---|---|---|
| `UGV_PID_KP_X1000` / `_KI_X1000` / `_KD_X1000` | 20000 / 2000000 / 0 | Seeded from stock Beast firmware. Almost certainly needs tuning. |
| `UGV_PID_OUTPUT_CLAMP` | 255 | Also used as the open-loop PWM clamp. |
| `UGV_PID_DEADBAND` | 23 | Output magnitudes below this are zeroed — avoids motor stall hum. Applied in both modes. |

The PID gains can also be pushed live over MQTT — see the `cmd/pid` topic
below — and the change persists across boots because the topic is retained.

### Timing
| Option | Default | Notes |
|---|---|---|
| `UGV_CONTROL_HZ` | 100 | Control loop rate. |
| `UGV_TELEMETRY_HZ` | 50 | Wheel telemetry publish rate. |
| `UGV_BATTERY_HZ` | 1 | Battery telemetry publish rate. |
| `UGV_IMU_HZ` | 100 | IMU sample + publish rate. AK09918 mag runs at 100 Hz natively — packets above 100 Hz will repeat mag values. |
| `UGV_HEARTBEAT_TIMEOUT_MS` | 500 | cmd_vel staleness window; motors stop when exceeded. |

### Sensors & display
| Option | Default | Notes |
|---|---|---|
| `UGV_ENABLE_INA219` | y | Battery voltage/current. |
| `UGV_INA219_VOLTAGE_SCALE_X1000` | 1000 | ×1.0 = chip-reading is actual battery voltage. Tune against multimeter if board has a divider. |
| `UGV_ENABLE_IMU` | y | QMI8658 (6-axis) + AK09918 (3-axis mag). |
| `UGV_IMU_BODY_INVERT_X` / `_Y` / `_Z` | n | Per-axis sign flip for *display-side* odometry only. Published `tel/imu` data is always raw chip-frame — host applies its own calibration there. |
| `UGV_ENABLE_DISPLAY` | y | SSD1306 128×32 OLED at I²C 0x3C. |
| `UGV_DISPLAY_HZ` | 1 | OLED refresh rate. |
| `UGV_DISPLAY_HOST_TIMEOUT_MS` | 2000 | Switches OLED from host pose (`H` indicator) to firmware odometry (`L`) after this. |

## MQTT contract

All packets are **little-endian packed structs**, defined in
`main/ugv_packets.h`. Share that header verbatim with the host. Sizes
are guarded with `_Static_assert` so any drift fails the build.

Topics (with `<id>` = `UGV_ROBOT_ID`):

| Topic | Dir | QoS | Retain | Struct | Size |
|---|---|---|---|---|---|
| `ugv/<id>/v1/cmd/vel`     | sub | 1 | no  | `ugv_cmd_vel_t`       | 16 B |
| `ugv/<id>/v1/cmd/pid`     | sub | 1 | yes | `ugv_cmd_pid_t`       | 20 B |
| `ugv/<id>/v1/cmd/display` | sub | 1 | no  | `ugv_cmd_display_t`   | 28 B |
| `ugv/<id>/v1/tel/wheel`   | pub | 0 | no  | `ugv_wheel_telem_t`   | 36 B |
| `ugv/<id>/v1/tel/imu`     | pub | 0 | no  | `ugv_imu_telem_t`     | 56 B |
| `ugv/<id>/v1/tel/battery` | pub | 0 | no  | `ugv_battery_telem_t` | 16 B |
| `ugv/<id>/v1/status`      | pub | 1 | yes | `"online"`/`"offline"` | string LWT |

Key semantic points:
- **`cmd/vel`** drives motion *and* serves as the heartbeat. No new
  packet within `UGV_HEARTBEAT_TIMEOUT_MS` → motors zeroed.
- **`cmd/pid`** is retained, so the bot picks up the last-set tuning
  across reboots.
- **`cmd/display`** is the host's authoritative pose feed for the OLED.
  Stop publishing it (or kill the host) and the OLED falls back to the
  firmware's odometry estimate within ~2 s.
- **`tel/wheel`** carries cumulative `left_ticks` and `right_ticks` so
  the host integrates exact wheel distance independent of network jitter
  (no encoders on the current bot — these stay at 0). Velocity fields
  are nice-to-have, not authoritative.
- **`tel/imu`** is raw chip-frame readings — accel in m/s², gyro in
  rad/s, mag in µT. A `mag_fresh` flag tells the host whether the mag
  fields were updated this packet or repeat the previous reading (mag
  chip runs at fixed 100 Hz).
- **`status`** uses MQTT LWT — the broker auto-publishes `offline` if
  the bot drops, no firmware code needed.

## OLED layout (128×32, 5×7 font)

```
x:+0.000 y:+0.000   H     ← H = host data fresh, L = firmware odometry
Y:+000.0  P:+000.0
R:+000.0 d
V: 12.34                  ← INA219 battery voltage
```

## Bring-up checklist

1. **Power-on log.** Watch `idf.py monitor` for `mqtt connected`,
   `got ip`, `INA219: init done`, `qmi8658: ready`, `oled: ready`.
2. **Verify topics.** Subscribe to `ugv/<id>/v1/status` — should see
   `online` retained. Kill the bot, should flip to `offline`.
3. **Motor direction.** From host, publish `cmd_vel { linear=0.1, angular=0 }`.
   Both wheels should spin forward. Wrong side? Flip `UGV_MOTOR_*_INVERT`.
4. **Encoder direction** (closed-loop only). Roll each wheel forward by
   hand, watch `tel/wheel.left_ticks` / `right_ticks` increase. Flip
   `UGV_ENCODER_*_INVERT` if either decreases.
5. **Open-loop scale tune.** Publish `cmd_vel { linear=0.1 }`, time how
   long the bot takes to cover a known distance. Adjust
   `UGV_OPEN_LOOP_PWM_PER_MPS_X10` so that displayed velocity matches.
6. **OLED.** With cmd_vel sent and no `cmd/display` from host, OLED
   should show `L` indicator and x/y trending. Start publishing
   `cmd/display` from host — indicator flips to `H` within 1 s, values
   reflect host pose.

## Tools

`tools/cmd_vel_test.py` — publishes a fixed cmd_vel at a configurable
rate. Useful for bench testing motors and verifying packet round-trip.

```bash
pip install paho-mqtt
./tools/cmd_vel_test.py --broker mqtt-server --linear 0.1 --hz 10
```

At `--hz 2` (default) you're right at the 500 ms heartbeat boundary, so
expect intermittent stutter from network jitter. For sustained motion
testing use `--hz 10+`.

## Architecture

- **`control_task`** (100 Hz, core 1): drains `cmd/vel` + `cmd/pid`
  queues, applies kinematics, reads encoders (no-op without hardware),
  computes PWM (PID in closed-loop, direct scale in open-loop), drives
  motors, snapshots state for telemetry.
- **`telemetry_task`** (50 Hz, core 0): consumes the control snapshot,
  publishes `tel/wheel`; once a second also reads INA219 and publishes
  `tel/battery` + runs the encoder GPIO/PCNT diagnostic log.
- **`imu_task`** (100 Hz, core 0): reads QMI8658 + AK09918, publishes
  `tel/imu`, feeds body-frame-corrected values into the firmware
  odometry.
- **`display_task`** (1 Hz, core 0): renders OLED, prefers host pose
  via `cmd/display` if recent, otherwise uses firmware odometry.

The PID lives in `pid.[ch]`, kinematics in `kinematics.[ch]`, firmware
odometry in `odometry.[ch]`. Sensor drivers (`ina219`, `qmi8658`,
`ak09918`, `imu`, `oled`) all sit on the shared `i2c_bus` (GPIO 32 SDA,
33 SCL, 400 kHz). MQTT and WiFi live in `comms.[ch]`.

## Known limitations

- **No wheel encoders on the current bot.** Open-loop mode is on. When
  the encoder-equipped bot arrives, set `UGV_OPEN_LOOP=n` in menuconfig
  — closed-loop code is already in and tested-ish (the PID was seeded
  but never tuned against real wheels).
- **Yaw drift** in firmware odometry: pure gyro integration, no mag
  fusion. Visible over minutes of motion. Not a problem in practice
  because authoritative pose comes from the host (iPhone VIO via Pi5);
  firmware estimate is only for the OLED fallback.
- **OLED font is hand-typed and covers only the glyphs the display
  uses** (~30 characters). If you want to print arbitrary strings from
  somewhere else, extend the `FONT[]` table in `oled.c`.
- **Battery telemetry's `current_a` field is from the chip-side shunt
  and may not reflect actual battery draw** depending on board wiring —
  trend signal only.
- **GPIO 34/35 lack internal pull-ups** (input-only pads on ESP32). The
  IDF PCNT driver tries to enable them and logs an error per pin at
  boot — cosmetic, no flag to suppress.
