#!/usr/bin/env python3
"""Drops non-Adjacent disable_collisions entries that involve a tool link."""
import re
import sys
import xml.etree.ElementTree as ET

tool_xacro, srdf = sys.argv[1:3]
tool_links = set(re.findall(r'<link name="([^"$]+)"', open(tool_xacro).read()))
tree = ET.parse(srdf)
root = tree.getroot()
dropped = 0
for e in root.findall("disable_collisions"):
    if {e.get("link1"), e.get("link2")} & tool_links and e.get("reason") != "Adjacent":
        root.remove(e)
        dropped += 1
tree.write(srdf)
print(f"dropped {dropped} tool entries, {len(root.findall('disable_collisions'))} left")
