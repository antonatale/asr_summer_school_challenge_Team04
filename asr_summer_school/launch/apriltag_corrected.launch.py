from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

import os
import yaml

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def generate_launch_description():

    sim = LaunchConfiguration("use_sim_time")
    package_name = "turtlebot3_perception"

    params = os.path.join(
        get_package_share_directory(package_name), "config", "apriltag.yaml"
    )
    params = yaml_to_dict(params)

    load_composable_nodes = LoadComposableNodes(
        target_container="/camera/camera_container",
        composable_node_descriptions=[
            ComposableNode(
                namespace="camera",
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                remappings=[
                    ("image_rect", "/camera/color/image_raw"),
                    ("camera_info", "/camera/color/camera_info"),
                ],
                parameters=[params["camera"]["apriltag"]["ros__parameters"], {"use_sim_time": sim}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    landmark_node = Node(
        namespace="camera",
        package="asr_summer_school",
        executable="tag_landmarks.py",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_base_frame": "base_link", "use_sim_time": sim}]
    )

    return LaunchDescription([DeclareLaunchArgument("use_sim_time", default_value="false"), load_composable_nodes, landmark_node])
