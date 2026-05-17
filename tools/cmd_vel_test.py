#!/usr/bin/env python3
"""Send fixed cmd_vel at a configurable rate to a UGV bot over MQTT.

Wire format (must match main/ugv_packets.h::ugv_cmd_vel_t, 16 B little-endian):
    uint64_t host_timestamp_us      (host clock, µs since epoch — host-side stat use only)
    float    linear_x               m/s, +forward
    float    angular_z              rad/s, +CCW viewed from above

Topic: ugv/<robot_id>/v1/cmd/vel  (QoS 1, not retained)

Dependencies:
    pip install paho-mqtt

Notes:
- Firmware heartbeat timeout is CONFIG_UGV_HEARTBEAT_TIMEOUT_MS (default 500 ms).
  At --hz 2 you're publishing every 500 ms, so any network jitter pushes packets
  past the timeout and the bot stutter-stops. For real motion testing run at
  10 Hz+. 2 Hz is fine for verifying packets land.
- On Ctrl-C this sends a final zero cmd_vel before disconnecting; the bot's
  heartbeat watchdog would also stop the motors within ~500 ms regardless.
"""

import argparse
import signal
import struct
import sys
import time

import paho.mqtt.client as mqtt


PACKET_FMT = "<Qff"   # uint64 + float + float, packed, little-endian
PACKET_LEN = struct.calcsize(PACKET_FMT)
assert PACKET_LEN == 16, f"expected 16-byte packet, got {PACKET_LEN}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="192.168.1.100", help="MQTT broker host")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--id", default="ugv01",
                    help="robot id, matches CONFIG_UGV_ROBOT_ID on firmware side")
    ap.add_argument("--linear",  type=float, default=0.1, help="m/s,    +forward")
    ap.add_argument("--angular", type=float, default=0.0, help="rad/s,  +CCW")
    ap.add_argument("--hz", type=float, default=2.0, help="publish rate")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to run, 0 = until Ctrl-C")
    args = ap.parse_args()

    topic = f"ugv/{args.id}/v1/cmd/vel"
    period = 1.0 / args.hz

    client = mqtt.Client()
    print(f"connecting to {args.broker}:{args.port} ...", flush=True)
    client.connect(args.broker, args.port, keepalive=10)
    client.loop_start()

    print(f"publishing -> {topic}  ({PACKET_LEN} B, lin={args.linear} m/s, "
          f"ang={args.angular} rad/s, {args.hz} Hz)")
    print("Ctrl-C to stop\n")

    stop_flag = {"v": False}
    def on_sigint(_sig, _frame):
        stop_flag["v"] = True
    signal.signal(signal.SIGINT, on_sigint)

    sent = 0
    t_start = time.monotonic()
    next_t = t_start
    try:
        while not stop_flag["v"]:
            if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
                break

            host_ts_us = time.time_ns() // 1000
            payload = struct.pack(PACKET_FMT, host_ts_us,
                                  args.linear, args.angular)
            client.publish(topic, payload, qos=1, retain=False)
            sent += 1

            # Log every second.
            if sent % max(int(round(args.hz)), 1) == 0:
                print(f"  [{sent:5d}]  lin={args.linear:+.3f}  ang={args.angular:+.3f}")

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Fell behind; reset schedule to avoid drift accumulation.
                next_t = time.monotonic()
    finally:
        # Final zero so the bot stops immediately (rather than after the
        # heartbeat timeout expires).
        zero = struct.pack(PACKET_FMT, time.time_ns() // 1000, 0.0, 0.0)
        client.publish(topic, zero, qos=1, retain=False)
        time.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print(f"\nsent {sent} packets, published final zero cmd_vel")
    return 0


if __name__ == "__main__":
    sys.exit(main())
