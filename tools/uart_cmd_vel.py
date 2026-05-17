#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm
"""Send a fixed cmd_vel over UART to a UGV bot.

Mirror of cmd_vel_test.py (which uses MQTT) — same flags, different
transport. Wire format matches main/uart_link.c.

Dependencies:
    pip install pyserial

Pi 5 wiring note:
    /dev/ttyAMA0 is shared with the Bluetooth HCI by default. Either
    `dtoverlay=disable-bt` in /boot/firmware/config.txt and use ttyAMA0,
    or `dtoverlay=uart0` and use ttyAMA1.
"""

import argparse
import signal
import struct
import sys
import time

import serial

from ugv_packets import (
    FMT_CMD_VEL, PKT_CMD_VEL,
    SIZE_BY_TYPE, frame,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyAMA0", help="serial device")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--linear",  type=float, default=0.1, help="m/s,    +forward")
    ap.add_argument("--angular", type=float, default=0.0, help="rad/s,  +CCW")
    ap.add_argument("--hz", type=float, default=10.0, help="publish rate")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to run, 0 = until Ctrl-C")
    args = ap.parse_args()

    expected_payload = SIZE_BY_TYPE[PKT_CMD_VEL]
    assert struct.calcsize(FMT_CMD_VEL) == expected_payload

    print(f"opening {args.port} at {args.baud} ...", flush=True)
    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    print(f"sending cmd_vel  lin={args.linear} m/s  ang={args.angular} rad/s  "
          f"at {args.hz} Hz")
    print("Ctrl-C to stop\n")

    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))

    period = 1.0 / args.hz
    sent = 0
    t_start = time.monotonic()
    next_t = t_start
    try:
        while not stop["v"]:
            if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
                break

            host_ts_us = time.time_ns() // 1000
            payload = struct.pack(FMT_CMD_VEL, host_ts_us,
                                  args.linear, args.angular)
            ser.write(frame(PKT_CMD_VEL, payload))
            sent += 1
            if sent % max(int(round(args.hz)), 1) == 0:
                print(f"  [{sent:5d}]  lin={args.linear:+.3f}  ang={args.angular:+.3f}")

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()
    finally:
        # Final zero so the bot stops immediately rather than after the
        # firmware heartbeat timeout (~500 ms).
        zero = struct.pack(FMT_CMD_VEL, time.time_ns() // 1000, 0.0, 0.0)
        ser.write(frame(PKT_CMD_VEL, zero))
        ser.flush()
        time.sleep(0.05)
        ser.close()
        print(f"\nsent {sent} frames, published final zero cmd_vel")
    return 0


if __name__ == "__main__":
    sys.exit(main())
