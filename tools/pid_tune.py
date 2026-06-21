#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm
"""Set PID gains live over MQTT and (optionally) watch wheel tracking.

The firmware applies cmd/pid at runtime (kp, ki, kd, output_clamp), so you
can tune gains without an OTA cycle each iteration. Find good values here,
then bake the winners into the Kconfig defaults (UGV_PID_K*_X1000).

Wire format (main/ugv_packets.h::ugv_cmd_pid_t, 20 B little-endian):
    float kp, ki, kd, output_clamp, min_drive
min_drive is the closed-loop sub-stiction floor (PWM): <0 leaves the
firmware's build-time CONFIG_UGV_MIN_DRIVE_PWM unchanged, >=0 overrides it
live (0 disables). It's the dominant lever for turn-in-place — at a small
turn setpoint the PID output is tiny, so whether the wheel breaks free is
mostly down to this floor.

Topics:
    publish:   ugv/<id>/v1/cmd/pid    (QoS 1)
    watch:     ugv/<id>/v1/tel/wheel  (--watch)

Dependencies:
    pip install paho-mqtt

Examples:
    # raise the stiction floor and watch how each wheel tracks its setpoint
    ./tools/pid_tune.py --broker 192.168.86.81 --id ugv01 \
        --kp 120 --ki 300 --min-drive 45 --watch
    # then, in another terminal, command a turn:
    ./tools/cmd_vel_test.py --broker 192.168.86.81 --angular 1.0 --hz 20
"""

import argparse
import struct
import sys
import time

import paho.mqtt.client as mqtt

PID_FMT = "<fffff"   # kp, ki, kd, output_clamp, deadband
WHEEL_FMT = "<QIiiffff"
WHEEL_LEN = struct.calcsize(WHEEL_FMT)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="192.168.86.81", help="MQTT broker host")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--id", default="ugv01",
                    help="robot id, matches CONFIG_UGV_ROBOT_ID")
    # kp/ki optional: omit both with --watch to observe wheel tracking
    # without changing the running gains.
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--clamp", type=float, default=255.0,
                    help="output clamp in PWM counts (default 255)")
    ap.add_argument("--min-drive", type=float, default=-1.0,
                    help="closed-loop stiction floor in PWM; -1 (default) "
                         "leaves the firmware's build-time value, >=0 sets it")
    ap.add_argument("--retain", action="store_true",
                    help="retain cmd/pid so the gains survive a reconnect "
                         "(handy while tuning; clears boot defaults until "
                         "overwritten)")
    ap.add_argument("--watch", action="store_true",
                    help="subscribe to tel/wheel and print setpoint vs measured")
    args = ap.parse_args()

    # Decide whether we're setting gains or just observing. Watch-only needs
    # both kp and ki omitted.
    send_gains = args.kp is not None or args.ki is not None
    if send_gains and (args.kp is None or args.ki is None):
        ap.error("specify both --kp and --ki to set gains "
                 "(or omit both with --watch to observe only)")
    if not send_gains and not args.watch:
        ap.error("nothing to do: pass --kp/--ki to set gains, or --watch")

    cmd_topic = f"ugv/{args.id}/v1/cmd/pid"
    wheel_topic = f"ugv/{args.id}/v1/tel/wheel"

    def on_wheel(_c, _u, m):
        if len(m.payload) != WHEEL_LEN:
            return
        (_ts, seq, _lt, _rt, lv, rv, lsp, rsp) = struct.unpack(WHEEL_FMT, m.payload)
        print(f"seq={seq:6d}  L set={lsp:+.3f} meas={lv:+.3f} err={lsp-lv:+.3f}"
              f"   R set={rsp:+.3f} meas={rv:+.3f} err={rsp-rv:+.3f}", flush=True)

    client = mqtt.Client()
    if args.watch:
        client.on_message = on_wheel
    print(f"connecting to {args.broker}:{args.port} ...", flush=True)
    client.connect(args.broker, args.port, keepalive=10)
    client.loop_start()

    if send_gains:
        payload = struct.pack(PID_FMT, args.kp, args.ki, args.kd,
                              args.clamp, args.min_drive)
        client.publish(cmd_topic, payload, qos=1, retain=args.retain)
        md = "unchanged" if args.min_drive < 0 else f"{args.min_drive}"
        print(f"sent gains -> {cmd_topic}: kp={args.kp} ki={args.ki} "
              f"kd={args.kd} clamp={args.clamp} min_drive={md}"
              f"{' (retained)' if args.retain else ''}", flush=True)
    else:
        print("watch-only: not changing gains", flush=True)

    if not args.watch:
        time.sleep(0.3)   # let the publish flush
        client.loop_stop()
        client.disconnect()
        return 0

    client.subscribe(wheel_topic, qos=0)
    print(f"watching {wheel_topic} (Ctrl-C to stop) ...", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
