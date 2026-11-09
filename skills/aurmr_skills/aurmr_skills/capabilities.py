"""What a tool can do, as small interfaces. A tool's client provides the methods of every
capability its tool.yaml lists. The interfaces are structural (typing.Protocol): a client
doesn't import or subclass them, so end_effectors/ doesn't depend on skills/.

How tool sensing reaches the arm (docs/how_to/tool_sensing.md):
- during a move, motion_server watches the tool's stop_signals topics itself (Stop(contact=True));
- between moves, a demo or skill reads the client, through these interfaces.
"""
from typing import Protocol, runtime_checkable


@runtime_checkable
class Contact(Protocol):
    def contact(self) -> bool:
        """Whether the tool touches something now. Raises if the reading is stale."""


@runtime_checkable
class Suction(Protocol):
    def suck(self) -> None: ...
    def blow(self) -> None: ...
    def off(self) -> None: ...

    def sealed(self) -> bool:
        """Whether the vacuum holds a seal now. Raises if the reading is stale."""


# tool.yaml capability name -> interface
CAPABILITIES = {"contact": Contact, "suction": Suction}
