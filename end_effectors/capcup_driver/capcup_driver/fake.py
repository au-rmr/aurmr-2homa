"""Stand-in for the ejector's CIP connection, for tests: records every request, and models the
vacuum rising while sucking on an item."""
import threading
import time
from types import SimpleNamespace

from capcup_driver import ejector as E


class FakeCIP:
    def __init__(self, item_present=True, sealed_mbar=820, rise_s=0.3):
        self.requests = []
        self.item_present = item_present
        self._sealed = sealed_mbar
        self._rise = rise_s
        self._mode, self._since = E.OFF, time.monotonic()
        self._lock = threading.Lock()

    def generic_message(self, service, class_code, instance, attribute, request_data=b""):
        with self._lock:
            self.requests.append((bytes(service), class_code, instance, attribute, bytes(request_data)))
            if instance == E.EJECTOR_CONTROL:
                mode = {v: k for k, v in E.CONTROL_BYTES.items()}[bytes(request_data)]
                self._mode, self._since = mode, time.monotonic()
                return SimpleNamespace(value=b"", error=None)
            if instance in (E.SETPOINT_H2, E.HYSTERESIS_H2):   # the lab ejector's settings
                v = 550 if instance == E.SETPOINT_H2 else 10
                return SimpleNamespace(value=v.to_bytes(2, "little") * 16, error=None)
            level = 0
            if self._mode == E.SUCK and self.item_present:
                done = 1.0 if self._rise <= 0 else (time.monotonic() - self._since) / self._rise
                level = int(self._sealed * min(1.0, done))
            elif self._mode == E.SUCK:
                level = 120   # sucking air
            value = bytes(4) + level.to_bytes(2, "little") + bytes(26)
            return SimpleNamespace(value=value, error=None)

    def close(self):
        pass
