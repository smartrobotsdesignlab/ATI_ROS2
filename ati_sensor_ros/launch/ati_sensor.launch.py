# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "update_rate",
            default_value="500",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ether_name_L",
            default_value="enp3s0f0",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ether_name_R",
            default_value="enp3s0f1",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "side",
            default_value="dual",
            description="Selected which robot to start.",
        )
    )

    # Initialize Arguments
    update_rate = LaunchConfiguration("update_rate")
    ether_name_L = LaunchConfiguration("ether_name_L")
    ether_name_R = LaunchConfiguration("ether_name_R")
    side = LaunchConfiguration("side")

    # Parameters
    update_rate_param = {"update_rate": update_rate}
    ether_name_L_param = {"ether_name": ether_name_L}
    ether_name_R_param = {"ether_name": ether_name_R}

    ati_node_L = Node(
        package="ati_sensor_ros",
        executable="ati_sensor_ros",
        namespace="left",
        parameters=[update_rate_param, ether_name_L_param],
        condition=IfCondition(PythonExpression(["'", side, "' == 'left'" , " or '", side, "' == 'dual'"])),
        output="both",
    )
    ati_node_R = Node(
        package="ati_sensor_ros",
        executable="ati_sensor_ros",
        namespace="right",
        parameters=[update_rate_param, ether_name_R_param],
        condition=IfCondition(PythonExpression(["'", side, "' == 'right'" , " or '", side, "' == 'dual'"])),
        output="both",
    )

    nodes = [
        ati_node_L,
        ati_node_R,
    ]

    return LaunchDescription(declared_arguments + nodes)
