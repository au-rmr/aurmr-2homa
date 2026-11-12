"""Robot bringup: ros2_control, motion_server, MoveIt.

Args: site_dir (absolute), tool, hardware, robot_ip, start_pose, wrench_topic, speed_scaling_topic,
headless_mode, private_domain.
hardware: ur (the real arm), ur_mock (the UR driver's mock: same controllers, no arm),
mock (generic mock with a plain trajectory controller).

private_domain (default 142): ros2_control, the controllers and the UR driver's helpers run in
this ROS domain, which only this host can see (tahoma_control/config/cyclonedds_private.xml).
motion_server is the only process in both domains, so it is the only way to the arm from the
robot network; it copies joint states, force and speed scaling out to the shared domain.
To reach the private side by hand, on this host: scripts/private_shell.sh. "none" puts
everything on the shared domain (debugging only: then anything on the network can command the
arm).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tahoma_moveit_config.model import robot_description, robot_semantic


def config(name):
    with open(os.path.join(share("tahoma_moveit_config"), "config", name)) as f:
        return yaml.safe_load(f)


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    site, tool, hardware = get("site_dir"), get("tool"), get("hardware")
    if hardware not in ("ur", "ur_mock", "mock"):
        raise RuntimeError(f"hardware must be ur, ur_mock or mock, not {hardware}")
    ur = hardware in ("ur", "ur_mock")
    real = hardware == "ur"
    headless = get("headless_mode") == "true"
    ip = get("robot_ip")
    if real and not ip:
        with open(os.path.join(site, "arm.yaml")) as f:
            ip = yaml.safe_load(f)["robot_ip"]
    urdf = robot_description(site, tool, hardware=hardware, robot_ip=ip or "0.0.0.0",
                             mock_start=None if real else get("start_pose"), headless=headless)
    description = {"robot_description": urdf}
    domain = get("private_domain")
    cfg = os.path.join(share("tahoma_control"), "config")
    if domain == "none":
        private_env, server_env, server_domain = {}, {}, -1
    else:
        private_env = {"ROS_DOMAIN_ID": domain, "AURMR_PRIVATE_DOMAIN": domain,
                       "CYCLONEDDS_URI": f"file://{cfg}/cyclonedds_private.xml"}
        server_env = {"AURMR_PRIVATE_DOMAIN": domain,
                      "CYCLONEDDS_URI": f"file://{cfg}/cyclonedds_shared.xml,"
                                        f"file://{cfg}/cyclonedds_private.xml"}
        server_domain = int(domain)
    controllers = os.path.join(share("tahoma_moveit_config"), "config",
                               "ur_controllers.yaml" if ur else "ros2_controllers.yaml")
    spawner = lambda *names: Node(package="controller_manager", executable="spawner",
                                  arguments=[*names, "--controller-manager-timeout", "30"],
                                  additional_env=private_env)
    with open(os.path.join(share(tool), "tool.yaml")) as f:
        signals = yaml.safe_load(f)["stop_signals"]
    # Each entry: a topic, or {topic, stale_s, max_speed} (tool.yaml).
    signal_topics = [v if isinstance(v, str) else v["topic"] for v in signals.values()]
    signal_stales = [0.1 if isinstance(v, str) else float(v.get("stale_s", 0.1)) for v in signals.values()]
    wrench = get("wrench_topic") or (
        "/force_torque_sensor_broadcaster/wrench" if real else "/wrench")
    scaling = (get("speed_scaling_topic") or "/speed_scaling_state_broadcaster/speed_scaling"
               ) if ur else ""
    arm_controller = spawner("arm_controller")
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[description,
                                  {"robot_description_semantic": robot_semantic(site, tool),
                                   "robot_description_kinematics": config("kinematics.yaml"),
                                   "robot_description_planning": config("joint_limits.yaml"),
                                   "planning_pipelines": ["ompl"],
                                   "default_planning_pipeline": "ompl",
                                   "ompl": config("ompl_planning.yaml"),
                                   "publish_robot_description_semantic": True}])
    # MoveIt gets no controllers on any hardware: it plans, motion_server executes. With the
    # private domain it couldn't reach them anyway.
    nodes = [
        # If the motion server dies, everything stops with it, the driver included.
        Node(package="tahoma_control", executable="motion_server", output="screen",
             on_exit=Shutdown(reason="motion_server exited"), additional_env=server_env,
             parameters=[{"signal_names": list(signals), "signal_topics": signal_topics,
                          "signal_stale_each_s": signal_stales,
                          "use_speed_scaling": bool(scaling), "wrench_topic": wrench,
                          "joints_topic": "/joint_states",
                          "speed_scaling_topic": scaling or "speed_scaling",
                          "private_domain_id": server_domain}]),
        # TF on the shared domain, from the joint states motion_server copies out.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[controllers], remappings=[("~/robot_description", "/robot_description")],
             additional_env=private_env),
        spawner("joint_state_broadcaster"),
        arm_controller,
        # The action server exists before the controller is active and rejects goals until then.
        # The spawner exits once it is active, so move_group being up means execution can work.
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller,
            on_exit=lambda event, _: [move_group] if event.returncode == 0 else [])),
    ]
    if private_env:
        # ros2_control reads the description from this topic, on its own domain.
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          name="robot_state_publisher_private", parameters=[description],
                          additional_env=private_env))
    if ur:
        nodes.append(spawner("speed_scaling_state_broadcaster", "force_torque_sensor_broadcaster",
                             "io_and_status_controller"))
    if real:
        nodes += [
            Node(package="ur_robot_driver", executable="dashboard_client", name="dashboard_client",
                 output="screen", parameters=[{"robot_ip": ip}], additional_env=private_env),
            # Deactivates the controllers while the tablet program isn't running, and brings
            # them back when it is.
            Node(package="ur_robot_driver", executable="controller_stopper_node",
                 name="controller_stopper", output="screen", additional_env=private_env,
                 parameters=[{"headless_mode": headless,
                              "consistent_controllers": ["io_and_status_controller",
                                                         "force_torque_sensor_broadcaster",
                                                         "joint_state_broadcaster",
                                                         "speed_scaling_state_broadcaster"]}]),
            Node(package="ur_robot_driver", executable="robot_state_helper",
                 name="ur_robot_state_helper", output="screen", additional_env=private_env,
                 parameters=[{"headless_mode": headless, "robot_ip": ip}]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site_dir"),
        DeclareLaunchArgument("tool", default_value="capcup"),
        DeclareLaunchArgument("hardware", default_value="ur_mock"),
        DeclareLaunchArgument("robot_ip", default_value=""),   # default: the site's arm.yaml
        DeclareLaunchArgument("start_pose", default_value="home"),
        DeclareLaunchArgument("wrench_topic", default_value=""),
        DeclareLaunchArgument("speed_scaling_topic", default_value=""),
        # true: the driver sends its own program, no tablet program needed. URSim tests use it;
        # the lab arm runs the External Control program from the tablet.
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("private_domain", default_value="142"),
        OpaqueFunction(function=setup),
    ])
