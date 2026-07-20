#!/usr/bin/env python3
"""Send a zero command and force the S2 chassis into parking mode."""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

from s2_nav2_cmd_vel_bridge import VendorManualWebSocket


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-host", default="192.168.127.10")
    parser.add_argument("--controller-http-port", type=int, default=8080)
    parser.add_argument("--controller-ws-port", type=int, default=6001)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    args = parser.parse_args()

    zero_sent = False
    socket = VendorManualWebSocket(
        f"ws://{args.controller_host}:{args.controller_ws_port}"
        "/api/AMR/ManualMove",
        args.timeout_sec,
    )
    try:
        socket.connect()
        for _ in range(3):
            socket.send_json({"id": args.robot_id, "x": 0.0, "y": 0.0, "w": 0.0})
            time.sleep(0.03)
        zero_sent = True
    except Exception as exc:
        print(f"[s2-estop] zero-command warning: {exc}", file=sys.stderr)
    finally:
        socket.close()

    url = (
        f"http://{args.controller_host}:{args.controller_http_port}"
        "/api/AMR/StartParking"
    )
    request = urllib.request.Request(
        url,
        data=b"{}",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=args.timeout_sec) as response:
            result = json.loads(response.read().decode("utf-8"))
        if int(result.get("ErrorCode", -1)) != 0:
            if str(result.get("ErrorMessage", "")).strip().lower() == "request error: conflict":
                print(
                    "[s2-estop] PARKING CONFIRMED (controller was already parked)"
                    + ("; zero velocity sent" if zero_sent else "")
                )
                return 0
            raise RuntimeError(result.get("ErrorMessage", result))
    except urllib.error.HTTPError as exc:
        # The vendor API returns HTTP 409 when StartParking is requested while
        # the chassis is already parked.  For an emergency-stop operation that
        # is the requested safe end state.
        if exc.code == 409:
            print(
                "[s2-estop] PARKING CONFIRMED (controller was already parked)"
                + ("; zero velocity sent" if zero_sent else "")
            )
            return 0
        print(f"[s2-estop] PARKING NOT CONFIRMED: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"[s2-estop] PARKING NOT CONFIRMED: {exc}", file=sys.stderr)
        return 2

    print(
        "[s2-estop] PARKING CONFIRMED"
        + ("; zero velocity sent" if zero_sent else "")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
