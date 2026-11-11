"""Window for checking workcell data in RViz: pick a named pose and scene, see MoveIt's verdict.

Reads geometry only. Publishes joint states for robot_state_publisher, so never run it alongside
a real robot (scripts/view_scene.sh runs it with no network).
"""
import argparse
import copy
import sys

import rclpy
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from python_qt_binding import QtCore, QtWidgets
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from tahoma_moveit_config import workcell as W
from tahoma_moveit_config.srdf import ARM_JOINTS

GROUP = "manipulator"
NO_SCENE = "(none)"
TABLE = (0.8, 0.6, 0.4, 0.55)
POD = (0.2, 0.45, 0.85, 0.3)
LIMIT = (0.95, 0.75, 0.2, 0.18)


def object_markers(wc, names, stamp):
    out = MarkerArray()
    out.markers.append(Marker(action=Marker.DELETEALL))
    for i, name in enumerate(names):
        co = W.collision_object(wc, name)
        cube = Marker(ns="objects", id=i, type=Marker.CUBE, action=Marker.ADD)
        cube.header.frame_id = co.header.frame_id
        cube.header.stamp = stamp
        cube.pose = co.primitive_poses[0]
        cube.scale.x, cube.scale.y, cube.scale.z = co.primitives[0].dimensions
        color = TABLE if name.startswith("table") else LIMIT if name.startswith("limits") else POD
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = color
        label = Marker(ns="labels", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        label.header = cube.header
        label.pose = copy.deepcopy(cube.pose)   # a shared Pose would drag the box down to the label's 1.9 m cap
        label.pose.position.z = min(cube.pose.position.z, 1.9)
        label.scale.z = 0.07
        label.text = name
        label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
        out.markers += [cube, label]
    return out


class Viewer(QtWidgets.QWidget):
    def __init__(self, node, model, wc, pose, scene, table):
        super().__init__()
        self.node, self.model, self.wc = node, model, wc
        self.joints = node.create_publisher(JointState, "joint_states", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = node.create_publisher(MarkerArray, "workcell_objects", latched)

        self.setWindowTitle("Workcell check")
        self.pose = QtWidgets.QComboBox()
        self.pose.addItems(list(wc["named_poses"]))
        self.pose.setCurrentText(pose)
        self.scene = QtWidgets.QComboBox()
        self.scene.addItems([NO_SCENE] + list(wc["scenes"]))
        self.scene.setCurrentText(scene)
        self.table = QtWidgets.QCheckBox("table_down")
        self.table.setChecked(table)
        self.verdict = QtWidgets.QLabel()
        self.verdict.setAlignment(QtCore.Qt.AlignCenter)
        form = QtWidgets.QFormLayout(self)
        form.addRow("Pose", self.pose)
        form.addRow("Limits / pod", self.scene)
        form.addRow("Table", self.table)
        form.addRow(self.verdict)
        for combo in (self.pose, self.scene):
            combo.currentTextChanged.connect(self.refresh)
        self.table.toggled.connect(self.refresh)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.refresh()

    def refresh(self, *_):
        names = [] if self.scene.currentText() == NO_SCENE else [self.scene.currentText()]
        names += ["table_down"] if self.table.isChecked() else []
        objects = W.object_names(self.wc, names)
        self.q = self.wc["named_poses"][self.pose.currentText()]
        # moveit_py shares one world between PlanningScene objects, so clear it every time.
        scene = PlanningScene(self.model)
        scene.remove_all_collision_objects()
        for name in objects:
            scene.apply_collision_object(W.collision_object(self.wc, name))
        state = RobotState(self.model)
        state.set_joint_group_positions(GROUP, list(self.q))
        state.update()
        hit = scene.is_state_colliding(state, GROUP)
        self.verdict.setText("MoveIt: IN COLLISION" if hit else "MoveIt: clear")
        color = "#c62828" if hit else "#2e7d32"
        self.verdict.setStyleSheet(f"color: white; background: {color}; padding: 8px;")
        self.markers.publish(object_markers(self.wc, objects, self.node.get_clock().now().to_msg()))

    def tick(self):
        msg = JointState(name=ARM_JOINTS, position=[float(v) for v in self.q])
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.joints.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0)


def main():
    parser = argparse.ArgumentParser()
    for name in ("--urdf", "--srdf", "--workcell"):
        parser.add_argument(name, required=True)
    parser.add_argument("--pose", default="drop_hide")
    parser.add_argument("--scene", default="limits_drop_hide")
    parser.add_argument("--no-table", action="store_true")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node("scene_viewer")
    model = RobotModel(urdf_xml_path=args.urdf, srdf_xml_path=args.srdf)
    app = QtWidgets.QApplication(sys.argv[:1])
    window = Viewer(node, model, W.load(args.workcell), args.pose, args.scene, not args.no_table)
    window.show()
    app.exec_()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
