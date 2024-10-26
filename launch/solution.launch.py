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

# File: solution.launch.py
# This is the main launch file for a multi-robot solution, including other launch files. 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the other launch file, replace with what you need.
    abb_irb1600_pkg = 'dtir_solution_manager'
    ur_5_pkg = 'dtir_solution_manager'
    kuka_kr6r700sixx_pkg = 'dtir_solution_manager'

    abb_irb1600_launch_file_path = os.path.join(get_package_share_directory(abb_irb1600_pkg), 'launch', 'abb_irb1600.launch.py')
    ur_5_launch_file_path = os.path.join(get_package_share_directory(ur_5_pkg), 'launch', 'ur_5.launch.py')
    kuka_kr6r700sixx_launch_file_path = os.path.join(get_package_share_directory(kuka_kr6r700sixx_pkg), 'launch', 'kuka_kr6r700sixx.launch.py')

    kuka_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kuka_kr6r700sixx_launch_file_path),
            launch_arguments={
                'namespace': 'ips_kuka',
                'robot_model': 'kr6_r700_sixx',
                'client_ip': '192.168.2.86'
                }.items()
        )
    ur_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_5_launch_file_path),
            launch_arguments={
                'namespace': 'ips_ur',
                'ur_type': 'ur5',
                'robot_ip': '192.168.1.60',
                'controllers_file': 'ur_controllers.yaml',
                'description_package': 'ur_description',
                'description_file': 'ur.urdf.xacro',
                'tf_prefix': ''
                }.items()
        )
    abb_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(abb_irb1600_launch_file_path),
            launch_arguments={
                'namespace': 'ips_abb',
                'description_package': 'abb_irb1600_support',
                'description_file': 'irb1600_6_12.xacro',
                'rws_ip': '192.168.1.101',
                'controllers_file': 'abb_controllers.yaml'
                }.items()
        )

    # Create the solution_manager node
    # solution_manager_node = Node(
    #     package='dtir_solution_manager', 
    #     executable='solution_manager',  
    #     name='ips_solution_manager',
    #     output='screen',
    #     parameters=[{
    #         'robot_namespaces': ['ips_abb', 'ips_kuka', 'ips_ur']
    #     }]
    # )

    # Optionally delay the start of solution_manager_node to ensure other nodes are up
    # delayed_solution_manager = TimerAction(
    #     period=5.0,  # Delay in seconds; adjust as needed
    #     actions=[solution_manager_node]
    # )

    # Return the LaunchDescription with all nodes
    return LaunchDescription([
        abb_launch,
        ur_launch,
        kuka_launch
        #delayed_solution_manager
    ])
