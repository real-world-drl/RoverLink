# RoverLink

ESP-IDF firmware for the lower computer on a small differential-drive
rover. Speaks binary MQTT and/or UART to an upper computer (planned:
Raspberry Pi 5 with iPhone VIO for pose). Hardware-targeted at the
Waveshare General Driver board (ESP32 WROOM-32) but the architecture
is platform-agnostic — pin assignments live in `main/Kconfig.projbuild`
and a handful of source files. Independent rewrite of the stock Arduino
firmware — see [Attribution](#attribution) below.

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

### Identity
| Option | Purpose |
|---|---|
| `UGV_ROBOT_ID` | Substituted into all MQTT topics: `ugv/<id>/v1/...` |

### Transports
| Option | Default | Purpose |
|---|---|---|
| `UGV_ENABLE_MQTT` | y | WiFi + MQTT transport. Disable to skip WiFi association on boot and save ~40 KB RAM. |
| `UGV_ENABLE_UART_LINK` | y | Direct binary link to the upper computer. |
| `UGV_UART_PORT` | 0 | UART peripheral. The board's RX/TX header is wired to UART0 (U0TX=GPIO1, U0RX=GPIO3), so the link lives there — which requires the serial console be disabled (see below). |
| `UGV_UART_BAUD` | 921600 | UART line rate. |
| `UGV_UART_TX_PIN` / `UGV_UART_RX_PIN` | 1 / 3 | U0TX/U0RX on the board's RX/TX header. Do **not** use GPIO 17 (motor AIN2) or 16 (right encoder B) — both are taken. |
| `UGV_UART_PREEMPT_MS` | 1000 | While the UART has received a valid frame within this window, MQTT cmd_vel is ignored. Prevents stale broker packets from overriding a live tethered control stream. |

### WiFi (only if `UGV_ENABLE_MQTT=y`)
| Option | Purpose |
|---|---|
| `UGV_WIFI_SSID`, `UGV_WIFI_PASS` | WiFi STA credentials |
| `UGV_WIFI_MAX_RETRY` | Reconnect attempts before giving up |

### MQTT (only if `UGV_ENABLE_MQTT=y`)
| Option | Purpose |
|---|---|
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
| `UGV_ENCODER_PPR` | 2100 | PCNT ticks per wheel revolution, including the firmware's 2x decode. Current bot: N20 hall encoder, 7 pulses/ch × 1:150 gearbox = 1050 single-channel, ×2 decode = 2100. Only used in closed-loop. |
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

## Wire contract (shared by MQTT and UART)

All packets are **little-endian packed structs**, defined in
`main/ugv_packets.h`. Share that header verbatim with the host. Sizes
are guarded with `_Static_assert` so any drift fails the build. The
Python tools in `tools/ugv_packets.py` mirror the same constants.

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
  (encoders are installed and counting on the current bot). Velocity
  fields are nice-to-have, not authoritative.
- **`tel/imu`** is raw chip-frame readings — accel in m/s², gyro in
  rad/s, mag in µT. A `mag_fresh` flag tells the host whether the mag
  fields were updated this packet or repeat the previous reading (mag
  chip runs at fixed 100 Hz).
- **`status`** uses MQTT LWT — the broker auto-publishes `offline` if
  the bot drops, no firmware code needed.

### UART transport

Same six packets and one status string; one type byte selects which.
On the wire:

```
[0xA5 sync] [type:1] [len:1] [payload:len] [crc8:1]   ; CRC over type+len+payload
```

Type IDs (matching `ugv_pkt_type_t` in firmware):

| ID | Direction | Packet |
|---|---|---|
| 0x01 | host → bot | `ugv_cmd_vel_t` |
| 0x02 | host → bot | `ugv_cmd_pid_t` |
| 0x03 | host → bot | `ugv_cmd_display_t` |
| 0x10 | bot → host | `ugv_wheel_telem_t` |
| 0x11 | bot → host | `ugv_imu_telem_t` |
| 0x12 | bot → host | `ugv_battery_telem_t` |
| 0x20 | bot → host | status string ("online" / "offline") |

CRC8 uses poly 0x07, init 0x00 — same algorithm in firmware
(`uart_link.c:crc8`) and Python (`tools/ugv_packets.py:crc8`).

The RX FSM logs counters every ~5 s: `rx_ok`, `rx_bad_crc`, `rx_bad_len`,
`rx_bad_type`, `rx_resync`. With the link on UART0 the serial console is
off (see below), so these go nowhere by default — use `tools/uart_sniff.py`
on the host to confirm framing health, or temporarily re-enable the console
(on a different UART) when debugging.

**The serial console is disabled** (`CONFIG_ESP_CONSOLE_NONE=y`). The
board breaks out its RX/TX header to U0RX/U0TX, the same UART the ESP-IDF
console used to own — so the console had to give up the line for the
binary link to use it. Runtime logs (`ESP_LOG*`) are dropped; rely on MQTT
or reflash with the console pointed at a spare UART when you need them.
Flashing over the **USB connector** is unaffected — the onboard USB-UART
chip drives DTR→IO0 / RTS→EN to auto-enter download mode. Flashing over
the **bare Pi UART header** does *not* auto-reset (the Pi's UART exposes
no DTR/RTS), so you must enter download mode by hand (hold BOOT, tap EN,
release BOOT) or wire EN/IO0 to Pi GPIOs — or use OTA over WiFi.
The first-stage ROM bootloader still emits a few bytes at 115200 on boot;
these look like garbage to a host reading at 921600 and are absorbed by
the `0xA5`/CRC8 resync.

**Pi 5 UART gotcha:** the Pi's primary UART (`/dev/ttyAMA0` on header
pins 8/10) is shared with the Bluetooth HCI by default. Either disable
BT (`dtoverlay=disable-bt` in `/boot/firmware/config.txt`, then use
`/dev/ttyAMA0`) or enable a second UART (`dtoverlay=uart0`, use
`/dev/ttyAMA1`). Skipping this step is the #1 reason "the Pi can't see
the bot."

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

`tools/cmd_vel_test.py` — publishes a fixed cmd_vel over **MQTT** at a
configurable rate.

```bash
pip install paho-mqtt
./tools/cmd_vel_test.py --broker mqtt-server --linear 0.1 --hz 10
```

`tools/uart_cmd_vel.py` — same but over **UART** (`pyserial`). Useful
once the Pi 5 is wired in.

```bash
pip install pyserial
./tools/uart_cmd_vel.py --port /dev/ttyAMA0 --linear 0.1 --hz 10
```

`tools/uart_sniff.py` — decodes telemetry frames coming back over UART
into readable lines. Cross-checks that the bot is publishing without
needing a broker.

```bash
./tools/uart_sniff.py --port /dev/ttyAMA0 --quiet-imu
```

`tools/ota_push.py` — serves a firmware `.bin` over HTTP and publishes
the OTA trigger over MQTT (see [Firmware updates](#firmware-updates-ota)).

```bash
./tools/ota_push.py --broker 192.168.1.100 --id ugv01 --bin build/roverlink.bin
```

`tools/ugv_packets.py` — shared constants, struct formats, framing, and
CRC8 used by both UART tools. Edit this if you ever change the wire
contract on the firmware side.

At `--hz 2` (default for `cmd_vel_test.py`) you're right at the 500 ms
heartbeat boundary, so expect intermittent stutter from network jitter.
For sustained motion testing use `--hz 10+`.

## Firmware updates (OTA)

The rover updates itself over WiFi — no serial cable once it's assembled.
The flash is partitioned into two app slots (`partitions.csv`); an update
is written to the *inactive* slot, then the bot reboots into it.

**One-time bootstrap (requires serial access once).** OTA changes the
partition table, and a partition table can only be installed by a full
serial flash in download mode — it cannot be applied over the air from a
pre-OTA build. So the *first* flash of this OTA-capable firmware must be
done over serial. Since the board's RX/TX header has no auto-reset (see
[the console note](#uart-transport)), enter download mode by hand:

1. Confirm the chip is ≥4 MB: `esptool.py -p <port> flash_id` (WROOM-32 is
   normally 4 MB). OTA needs two app slots and will not fit in 2 MB.
2. Hold **BOOT**, tap **EN/RST**, release **BOOT** to enter download mode.
3. Flash everything (bootloader + partition table + otadata + app):
   `idf.py -p <port> flash`, or the `esptool.py ... write_flash` line that
   `idf.py build` prints.
4. **Verify it connects to MQTT before sealing the rover** — a bootstrap
   image that can't reach the broker will roll back in a loop (see below).

**Every update after that is wireless:**

```bash
idf.py build
./tools/ota_push.py --broker 192.168.1.100 --id ugv01 --bin build/roverlink.bin
```

`ota_push.py` serves `roverlink.bin` over HTTP from your machine and
publishes the URL to `ugv/<id>/v1/cmd/ota`. The firmware downloads it via
`esp_https_ota`, writes the spare slot, and reboots. HTTP (not HTTPS) is
used on the LAN — keep the broker and this server off untrusted networks.

> **Run the push from a host on the rover's LAN** (e.g. the Pi). The rover
> fetches the image *back* from the serving machine's IP, so that machine
> must be inbound-reachable from the rover. A few gotchas:
> - **WSL2:** even in mirrored-networking mode (where WSL gets a real LAN
>   IP), Windows Firewall blocks the inbound HTTP port by default, so the
>   push times out with `bot never fetched` while the rover stays untouched.
>   Either run `ota_push.py` from the Pi, or open inbound TCP `8070` on
>   Windows (`New-NetFirewallRule ... -LocalPort 8070`, plus a
>   `New-NetFirewallHyperVRule` in mirrored mode).
> - If building elsewhere, `scp build/roverlink.bin tools/ota_push.py` to
>   the Pi and run it there.
> - A timeout never touches the running firmware — it's safe to retry.

**Automatic rollback.** With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, a
freshly OTA'd image boots in *pending* state and must prove itself by
reconnecting to the broker within `UGV_OTA_VALIDATE_TIMEOUT_MS` (default
120 s). If it can't — bad build, wrong WiFi, crash loop — the bootloader
reverts to the previous slot on the next reboot. This is the safety net
that makes updating an unreachable rover safe. The flip side: the very
first bootstrap image has nothing to roll back *to*, so verify it works
while you still have serial access.

## Architecture

- **`control_task`** (100 Hz, core 1): drains `cmd/vel` + `cmd/pid`
  queues, applies kinematics, reads encoders, computes PWM (PID in
  closed-loop, direct scale in open-loop), drives motors, snapshots
  state for telemetry.
- **`telemetry_task`** (50 Hz, core 0): consumes the control snapshot,
  fans `tel/wheel` out to whichever transports are enabled; once a
  second also reads INA219 and publishes `tel/battery`.
- **`imu_task`** (100 Hz, core 0): reads QMI8658 + AK09918, fans
  `tel/imu` out to enabled transports, feeds body-frame-corrected
  values into the firmware odometry.
- **`display_task`** (1 Hz, core 0): renders OLED, prefers host pose
  via `cmd/display` if recent, otherwise uses firmware odometry.
- **`uart_rx_task`** (core 0, only if `UGV_ENABLE_UART_LINK=y`): runs
  the UART framing FSM on inbound bytes; on a valid frame calls
  `comms_push_cmd_*` so UART commands feed the same queues as MQTT.

The PID lives in `pid.[ch]`, kinematics in `kinematics.[ch]`, firmware
odometry in `odometry.[ch]`. Sensor drivers (`ina219`, `qmi8658`,
`ak09918`, `imu`, `oled`) all sit on the shared `i2c_bus` (GPIO 32 SDA,
33 SCL, 400 kHz). MQTT + WiFi live in `comms.[ch]`; UART transport
lives in `uart_link.[ch]`. The shared command queues stay in `comms.c`
and are populated by both transports via `comms_push_cmd_*` helpers.

## Known limitations

- **Wheel encoders are installed and functional** on the current bot;
  closed-loop mode (`UGV_OPEN_LOOP=n`) is active. The PID gains were
  seeded but not yet tuned against the real wheels — expect to adjust
  them. `UGV_OPEN_LOOP=y` remains available as a fallback.
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

## Attribution

This project targets the same hardware as Waveshare's open-source
[`ugv_base_general`](https://github.com/effectsmachine/ugv_base_general)
firmware (GPL-3.0) and uses it as a reference for pin assignments,
sensor I²C addresses, and chip configuration values. No source code
was copied; this is an independent ESP-IDF rewrite with a different
architecture (FreeRTOS tasks, binary MQTT/UART, no Arduino runtime).
Thanks to the Waveshare team for documenting the board's wiring well
enough that an independent firmware was feasible.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
