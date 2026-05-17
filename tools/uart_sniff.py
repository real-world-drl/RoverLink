#!/usr/bin/env python3
"""Decode UGV UART telemetry frames in real time.

Implements the same RX state machine as main/uart_link.c so you can verify
the bot is publishing without bringing up the full host stack.

Dependencies:
    pip install pyserial
"""

import argparse
import signal
import struct
import sys
import time

import serial

from ugv_packets import (
    UART_SYNC, SIZE_BY_TYPE, NAME_BY_TYPE,
    FMT_TEL_WHEEL, FMT_TEL_IMU, FMT_TEL_BATT,
    PKT_TEL_WHEEL, PKT_TEL_IMU, PKT_TEL_BATT, PKT_STATUS,
    crc8,
)


def fmt_wheel(payload: bytes) -> str:
    ts, seq, lt, rt, lv, rv, ls, rs = struct.unpack(FMT_TEL_WHEEL, payload)
    return (f"WHEEL seq={seq:6d} L_ticks={lt:+8d} R_ticks={rt:+8d} "
            f"L_vel={lv:+.3f} R_vel={rv:+.3f}  set L={ls:+.3f} R={rs:+.3f}")


def fmt_imu(payload: bytes) -> str:
    (ts, seq,
     ax, ay, az, gx, gy, gz, mx, my, mz,
     temp_c, mag_fresh) = struct.unpack(FMT_TEL_IMU, payload)
    return (f"IMU   seq={seq:6d} a=({ax:+.2f},{ay:+.2f},{az:+.2f}) "
            f"g=({gx:+.3f},{gy:+.3f},{gz:+.3f}) "
            f"m=({mx:+.1f},{my:+.1f},{mz:+.1f}){'*' if mag_fresh else ' '} "
            f"T={temp_c:.1f}C")


def fmt_battery(payload: bytes) -> str:
    ts, v, i = struct.unpack(FMT_TEL_BATT, payload)
    return f"BATT  V={v:.3f}V  I={i:+.3f}A"


def fmt_status(payload: bytes) -> str:
    return f"STAT  {payload.decode('ascii', errors='replace')}"


FORMATTERS = {
    PKT_TEL_WHEEL: fmt_wheel,
    PKT_TEL_IMU:   fmt_imu,
    PKT_TEL_BATT:  fmt_battery,
    PKT_STATUS:    fmt_status,
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--quiet-imu", action="store_true",
                    help="suppress IMU lines (they spam at 100 Hz)")
    args = ap.parse_args()

    print(f"opening {args.port} at {args.baud} ...", flush=True)
    ser = serial.Serial(args.port, args.baud, timeout=0.5)

    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))

    # FSM mirror of uart_link.c:uart_rx_task.
    state = "WAIT_SYNC"
    cur_type = 0
    cur_len = 0
    payload = bytearray()

    stats = {"ok": 0, "bad_crc": 0, "bad_len": 0, "bad_type": 0}
    last_stats_t = time.monotonic()

    try:
        while not stop["v"]:
            b = ser.read(1)
            if not b:
                if time.monotonic() - last_stats_t > 5:
                    print(f"  -- stats: {stats}", file=sys.stderr)
                    last_stats_t = time.monotonic()
                continue
            b = b[0]

            if state == "WAIT_SYNC":
                if b == UART_SYNC:
                    state = "READ_TYPE"

            elif state == "READ_TYPE":
                expected = SIZE_BY_TYPE.get(b)
                # Status is variable-length; accept any len for it.
                if expected is None and b != PKT_STATUS:
                    stats["bad_type"] += 1
                    state = "WAIT_SYNC"
                    continue
                cur_type = b
                cur_len = expected  # may be None for STATUS
                state = "READ_LEN"

            elif state == "READ_LEN":
                if cur_len is not None and b != cur_len:
                    stats["bad_len"] += 1
                    state = "WAIT_SYNC"
                    continue
                cur_len = b  # finalize for STATUS too
                payload = bytearray()
                state = "READ_PAYLOAD" if cur_len > 0 else "READ_CRC"

            elif state == "READ_PAYLOAD":
                payload.append(b)
                if len(payload) >= cur_len:
                    state = "READ_CRC"

            elif state == "READ_CRC":
                hdr_pl = bytes([cur_type, cur_len]) + bytes(payload)
                if crc8(hdr_pl) != b:
                    stats["bad_crc"] += 1
                else:
                    stats["ok"] += 1
                    if cur_type == PKT_TEL_IMU and args.quiet_imu:
                        pass
                    elif cur_type in FORMATTERS:
                        try:
                            print(FORMATTERS[cur_type](bytes(payload)))
                        except struct.error as e:
                            print(f"unpack error for {NAME_BY_TYPE.get(cur_type)}: {e}",
                                  file=sys.stderr)
                state = "WAIT_SYNC"

    finally:
        ser.close()
        print(f"\nfinal stats: {stats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
