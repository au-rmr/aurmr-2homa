"""Schmalz SCTSi vacuum ejector over EtherNet/IP (CIP), ported from the old ROS 1
VacuumGripperSocketNode.py. Same requests, byte for byte: tests/capcup checks them.

The old code ignores the ejector number: it switches bytes 2 and 3 of the 16-byte control
array (ejectors 3 and 4) and reads the vacuum at bytes 4:6 (ejector 3). Kept as it was, because
that is how the lab's ejector is wired. Manual: Schmalz BAL 10.02.99.10501.
"""
CLASS = 0xA2
ATTRIBUTE = 5
EJECTOR_CONTROL = 13
SYSTEM_VACUUM = 515
OFF, SUCK, BLOW = 0, 1, 2
CONTROL_BYTES = {
    OFF: bytes(16),
    SUCK: bytes([0, 0, 0b01, 0b01] + [0] * 12),
    BLOW: bytes([0, 0, 0b10, 0b10] + [0] * 12),
}


class Ejector:
    def __init__(self, ip, driver_factory=None):
        """driver_factory(ip) returns an open pycomm3 CIPDriver, or a fake in tests."""
        if driver_factory is None:
            from pycomm3 import CIPDriver

            def driver_factory(address):
                d = CIPDriver(address)
                if not d.open():
                    raise ConnectionError(f"ejector at {address} did not answer")
                return d
        self._factory = driver_factory
        self._ip = ip
        self._driver = None

    def _call(self, **kwargs):
        from_pycomm = ()
        try:
            from pycomm3 import CommError
            from_pycomm = (CommError,)
        except ImportError:
            pass
        for attempt in (1, 2):   # the old driver reopened once on a comm error
            try:
                if self._driver is None:
                    self._driver = self._factory(self._ip)
                return self._driver.generic_message(class_code=CLASS, attribute=ATTRIBUTE, **kwargs)
            except (ConnectionError, OSError, *from_pycomm):
                self._driver = None
                if attempt == 2:
                    raise

    def set_mode(self, mode):
        from pycomm3 import Services
        reply = self._call(service=Services.set_attribute_single, instance=EJECTOR_CONTROL,
                           request_data=CONTROL_BYTES[mode])
        if reply is not None and getattr(reply, "error", None):
            raise ConnectionError(f"ejector refused mode {mode}: {reply.error}")

    def system_vacuum_mbar(self):
        from pycomm3 import Services
        reply = self._call(service=Services.get_attribute_single, instance=SYSTEM_VACUUM)
        if reply is None or getattr(reply, "error", None) or len(reply.value) < 6:
            raise ConnectionError("ejector gave no vacuum reading")
        return int.from_bytes(reply.value[4:6], "little")

    def close(self):
        if self._driver is not None:
            self._driver.close()
            self._driver = None
