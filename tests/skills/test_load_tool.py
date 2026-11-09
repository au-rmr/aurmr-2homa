"""aurmr_skills.load_tool: the client comes from the tool Robot was connected with, and
mismatches fail at startup. No ROS graph: a stand-in robot carries only its tool.yaml."""
from types import SimpleNamespace

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from aurmr_skills import Contact, NoClient, Suction, ToolMismatch, load_tool
from aurmr_skills.tools import check


class FakeCup:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def contact(self):
        return False

    def suck(self):
        pass

    def blow(self):
        pass

    def off(self):
        pass

    def sealed(self):
        return False


class NoSeal:
    def contact(self):
        return False

    def suck(self):
        pass


def robot(**tool):
    return SimpleNamespace(tool={"name": "t", **tool})


def test_builds_the_named_client_with_kwargs():
    cup = load_tool(robot(capabilities=["contact", "suction"], client="test_load_tool:FakeCup"),
                    needs=["suction"], timeout_s=3)
    assert isinstance(cup, FakeCup) and cup.kwargs == {"timeout_s": 3}
    assert isinstance(cup, Suction) and isinstance(cup, Contact)


def test_needed_capability_the_tool_lacks_is_refused():
    with pytest.raises(ToolMismatch, match="lacks"):
        load_tool(robot(capabilities=["contact"], client="test_load_tool:FakeCup"),
                  needs=["suction"])


def test_client_missing_a_listed_capability_is_refused():
    with pytest.raises(ToolMismatch, match="sealed"):
        load_tool(robot(capabilities=["suction"], client="test_load_tool:NoSeal"))


def test_unknown_capability_name_is_refused():
    with pytest.raises(ToolMismatch, match="unknown"):
        load_tool(robot(capabilities=["sucton"], client="test_load_tool:FakeCup"))


def test_passive_tool_gets_no_client():
    with load_tool(robot(capabilities=[])) as tool:
        assert isinstance(tool, NoClient)
    with pytest.raises(ToolMismatch, match="lacks"):
        load_tool(robot(capabilities=[]), needs=["contact"])
    with pytest.raises(ToolMismatch, match="names no client"):
        load_tool(robot(capabilities=["contact"]))


def test_capcup_yaml_and_client_agree():
    """The real pair, checked without starting the client (that needs the ROS graph)."""
    from capcup_driver.client import CapCup
    with open(f"{get_package_share_directory('capcup')}/tool.yaml") as f:
        tool = yaml.safe_load(f)
    assert tool["client"] == "capcup_driver.client:CapCup"
    check(tool, CapCup, needs=["contact", "suction"])
