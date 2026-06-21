#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm
"""Push a firmware image to a RoverLink bot over WiFi (OTA).

How it works:
  1. Serves the given .bin over plain HTTP from this host (the Pi).
  2. Publishes the download URL to  ugv/<robot_id>/v1/cmd/ota  (a plain
     UTF-8 string topic, not a packed struct).
  3. The bot downloads it via esp_https_ota, writes the inactive OTA slot,
     and reboots into the new image. On the next MQTT connect the firmware
     marks the image valid (cancelling the rollback watchdog); if it can't
     reconnect within CONFIG_UGV_OTA_VALIDATE_TIMEOUT_MS it rolls back.

This serves over HTTP, not HTTPS — fine on a trusted LAN, which is why the
firmware is built with CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y. Do not expose the
broker or this server to an untrusted network.

Dependencies:
    pip install paho-mqtt

Example (run from the project root after `idf.py build`):
    ./tools/ota_push.py --broker 192.168.1.100 --id ugv01 \
        --bin build/roverlink.bin
"""

import argparse
import http.server
import os
import socket
import sys
import threading
import time

import paho.mqtt.client as mqtt


def local_ip_toward(host: str, port: int) -> str:
    """Best-effort LAN IP this host uses to reach `host` (no traffic sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, port))
        return s.getsockname()[0]
    finally:
        s.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="192.168.1.100", help="MQTT broker host")
    ap.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument("--id", default="ugv01",
                    help="robot id, matches CONFIG_UGV_ROBOT_ID on firmware side")
    ap.add_argument("--bin", default="build/roverlink.bin",
                    help="path to the firmware image to serve")
    ap.add_argument("--http-port", type=int, default=8070,
                    help="port to serve the firmware on")
    ap.add_argument("--host", default=None,
                    help="IP the bot should fetch from (default: auto-detect)")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds to wait for the bot to fetch before giving up")
    args = ap.parse_args()

    if not os.path.isfile(args.bin):
        print(f"error: firmware not found: {args.bin}", file=sys.stderr)
        return 1
    binpath = os.path.abspath(args.bin)
    binname = os.path.basename(binpath)
    size = os.path.getsize(binpath)

    serve_ip = args.host or local_ip_toward(args.broker, args.port)
    url = f"http://{serve_ip}:{args.http_port}/{binname}"

    # Serve only this one file; any path maps to it. Track completed GETs so
    # we can exit once the bot has pulled the whole image.
    done = threading.Event()

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            print(f"  -> {self.client_address[0]} fetching firmware ...", flush=True)
            try:
                with open(binpath, "rb") as f:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(size))
                    self.end_headers()
                    self.wfile.write(f.read())
                print(f"  -> sent {size} bytes to {self.client_address[0]}",
                      flush=True)
                done.set()
            except (BrokenPipeError, ConnectionResetError):
                print("  -> client dropped mid-transfer", flush=True)

        def log_message(self, *a):  # silence default logging
            pass

    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"serving {binname} ({size} B) at {url}", flush=True)

    cmd_topic = f"ugv/{args.id}/v1/cmd/ota"
    status_topic = f"ugv/{args.id}/v1/tel/ota"

    # Echo the firmware's OTA status (begin/progress/finish/validated/err...)
    # as it publishes — this is how failures surface, since the device's
    # serial console is disabled.
    def on_message(_c, _u, m):
        print(f"  [bot] {m.payload.decode(errors='replace')}", flush=True)

    client = mqtt.Client()
    client.on_message = on_message
    print(f"connecting to broker {args.broker}:{args.port} ...", flush=True)
    client.connect(args.broker, args.port, keepalive=10)
    client.loop_start()
    # Clear any stale retained status from a previous run, then subscribe.
    client.publish(status_topic, "", qos=1, retain=True)
    client.subscribe(status_topic, qos=1)
    client.publish(cmd_topic, url, qos=1, retain=False)
    print(f"published OTA trigger -> {cmd_topic}", flush=True)
    print(f"  payload: {url}", flush=True)
    print(f"streaming status from {status_topic} ...", flush=True)

    print(f"waiting up to {args.timeout:.0f}s for the bot to fetch ...", flush=True)
    if done.wait(timeout=args.timeout):
        # Give status publishes (and the bot's "ok rebooting") a moment to
        # arrive before we tear down the connection.
        time.sleep(1.0)
        print("firmware delivered. The bot will reboot into the new image;",
              flush=True)
        print("watch for it to come back 'online' (rollback reverts it if not).",
              flush=True)
        rc = 0
    else:
        print("timed out — bot never fetched. Check it's online and the URL "
              "host/port is reachable from the bot.", file=sys.stderr)
        rc = 2

    client.loop_stop()
    client.disconnect()
    httpd.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
