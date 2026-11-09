"""Waits until URSim's dashboard server answers on port 29999 (it takes a while to boot)."""
import socket
import sys
import time

deadline = time.monotonic() + 180
while time.monotonic() < deadline:
    try:
        with socket.create_connection((sys.argv[1], 29999), timeout=2) as s:
            if b"Dashboard" in s.recv(200):
                sys.exit(0)
    except OSError:
        time.sleep(1)
print("URSim dashboard never answered", file=sys.stderr)
sys.exit(1)
