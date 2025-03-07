# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_infantry_bringup")

    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    vision_bringup_dir = get_package_share_directory("pb2025_vision_bringup")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    detector = LaunchConfiguration("detector")
    params_file = LaunchConfiguration("params_file")
    robot_name = LaunchConfiguration("robot_name")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_detector_cmd = DeclareLaunchArgument(
        "detector",
        default_value="opencv",
        description="Type of detector to use (option: 'opencv', 'openvino', 'tensorrt')",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "infantry.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="pb2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        vision_bringup_dir, "launch", "rm_vision_reality_launch.py"
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "detector": detector,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        serial_bringup_dir, "launch", "standard_robot_pp_ros2.launch.py"
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "robot_name": robot_name,
                    "params_file": params_file,
                    "use_rviz": use_rviz,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_detector_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
