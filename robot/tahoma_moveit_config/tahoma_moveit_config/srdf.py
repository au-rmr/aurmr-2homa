"""Builds the SRDF from three sources: base.srdf, the tool's collision list, the site's named poses."""
import xml.etree.ElementTree as ET

import yaml

ARM_JOINTS = [f"arm_{n}_joint" for n in
              ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]


def compose(base_path, collisions_path, workcell_path):
    root = ET.parse(base_path).getroot()
    with open(workcell_path) as f:
        workcell = yaml.safe_load(f)
    for name, values in workcell["named_poses"].items():
        state = ET.SubElement(root, "group_state", name=name, group="manipulator")
        for joint, value in zip(ARM_JOINTS, values, strict=True):
            ET.SubElement(state, "joint", name=joint, value=str(value))
    for entry in ET.parse(collisions_path).getroot().iter("disable_collisions"):
        root.append(entry)
    return ET.tostring(root, encoding="unicode")
