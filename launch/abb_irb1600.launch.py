# Copyright 2024 Zhihao Liu (zhihaoliu@ieee.org)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="ips_abb",
            description='Namespace.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="dtir_solution_manager",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="abb_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="abb_irb1600_support",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="",
            description="MoveIt configuration package for the robot, e.g. abb_irb1200_5_90_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="irb1600_6_12.xacro",
            description="URDF/XACRO description file with the robot, e.g. irb1200_5_90.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_ip",
            default_value="192.168.1.101",
            description="IP of RWS computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_port",
            default_value="80",
            description="Port at which RWS can be found. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    rws_ip = LaunchConfiguration("rws_ip")
    rws_port = LaunchConfiguration("rws_port")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ns = LaunchConfiguration("namespace")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "rws_ip:=",
            rws_ip,
            " ",
            "rws_port:=",
            rws_port,
            " ",
        ]
    )
    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content)}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config/abb/", controllers_file]
    )

    control_node = Node(
        namespace=ns,
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            robot_controllers,
            {
                "hardware_components_initial_state": {
                    "unconfigured": ["ABBMultiInterfaceHardware"]
                },
            },
        ],
        output="both",
    )

    robot_state_publisher_node = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace=ns,
        package="dtir_solution_manager",
        executable="abb_robot_manager_node",
        parameters=[{"robot_model": "ABBMultiInterfaceHardware"}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "ips_abb/controller_manager",
            "--inactive", # keep the controller to be inactive at first
        ],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "ips_abb/controller_manager", "--inactive",], # keep the controller to be inactive at first
    )
    
    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
