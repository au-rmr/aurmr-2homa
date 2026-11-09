"""Gives a demo its tool's client, from the tool Robot was connected with.

The demo names its tool once, in Robot.connect(tool=...). Robot reads that tool's tool.yaml;
load_tool builds the client the same file names, so the two can't disagree:

    with Robot.connect("capcup", 50, workcell) as robot, \
         load_tool(robot, needs=["suction", "contact"]) as cup:
        cup.suck()

tool.yaml:  capabilities: [contact, suction]
            client: capcup_driver.client:CapCup     # module:class; leave out for a passive tool

It refuses at startup, before anything moves: a capability the caller needs that the tool
doesn't list, a capability name nobody defined, or a client missing a listed capability's
methods. Robot stays motion-only: it never builds or calls the client.
"""
import importlib

from aurmr_skills.capabilities import CAPABILITIES


class ToolMismatch(Exception):
    pass


class NoClient:
    """The client of a passive tool (a probe, a 3D-printed part): nothing to do."""

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _client_class(path):
    module, _, name = path.partition(":")
    if not module or not name:
        raise ToolMismatch(f"tool.yaml client must be module:Class, got {path!r}")
    return getattr(importlib.import_module(module), name)


def check(tool, cls, needs=()):
    """Raises ToolMismatch unless tool (the tool.yaml dict) and its client class cls agree and
    cover needs."""
    listed = set(tool.get("capabilities") or [])
    unknown = (listed | set(needs)) - set(CAPABILITIES)
    if unknown:
        raise ToolMismatch(f"unknown capabilities {sorted(unknown)}; known: {sorted(CAPABILITIES)}")
    missing = set(needs) - listed
    if missing:
        raise ToolMismatch(f"tool {tool['name']} lacks {sorted(missing)} (has {sorted(listed)})")
    for name in sorted(listed):
        interface = CAPABILITIES[name]
        methods = [m for m in vars(interface) if not m.startswith("_") and callable(getattr(interface, m))]
        lacking = [m for m in methods if not callable(getattr(cls, m, None))]
        if lacking:
            raise ToolMismatch(f"{cls.__name__} lists {name} but has no {lacking}")


def load_tool(robot, needs=(), **kwargs):
    """The client for robot's tool, started. kwargs go to the client's constructor."""
    tool = robot.tool
    path = tool.get("client")
    if path is None:
        if tool.get("capabilities"):
            raise ToolMismatch(f"tool {tool['name']} lists capabilities but names no client")
        check(tool, NoClient, needs)
        return NoClient()
    cls = _client_class(path)
    check(tool, cls, needs)
    return cls(**kwargs)
