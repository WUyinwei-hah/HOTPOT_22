# Copyright (c) 2020 Samsung Research Russia
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
import math
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import HasNodeParams, RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Variables
    lifecycle_nodes = ['map_saver']

    # Getting directories and launch-files
    
    # bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    rmua19_ignition_simulator_dir = get_package_share_directory('rmua19_ignition_simulator')
    laser_filters_dir = get_package_share_directory('laser_filters')
    simu_launch_file = os.path.join(rmua19_ignition_simulator_dir, 'launch', 'lidar.launch.py')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')
    filter_launch_file = os.path.join(laser_filters_dir, 'launch', 'box_filter.launch.py')
    param_dir = get_package_share_directory('build_map')
    slam_param_file = os.path.join(param_dir, 'slam_param.yaml')
    nav2_config_path = os.path.join(param_dir, 'navigation_ua.yaml')
    rviz_config_file = os.path.join(param_dir, 'config.rviz')
    nav2_launch_path = os.path.join(param_dir, 'ua_navigation.launch.py')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(param_dir, 'nav2_param.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    # Nodes launching commands
    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            parameters=[configured_params])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
   
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(param_dir, 'ekf.yaml'), {'use_sim_time': use_sim_time}])

    # If the provided param file doesn't have slam_toolbox params, we must remove the 'params_file'
    # LaunchConfiguration, or it will be passed automatically to slam_toolbox and will not load
    # the default file
    has_slam_toolbox_params = HasNodeParams(source_file=params_file,
                                            node_name='slam_toolbox')


    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': slam_param_file}.items())
                          
    navigation2_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': nav2_config_path}.items())
                          
    start_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simu_launch_file)
        )
        
    start_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(filter_launch_file)
        )
                       
       
    tf1 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher1",
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'red_standard_robot1/chassis/chassis_imu'],
        )
    tf2 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher2",
            arguments=['0.15', '0.', '-0.03', '0.0', '0.0', str(math.pi), 'base_link', 'red_standard_robot1/front_rplidar_a2/front_rplidar_a2'],
        )
    tf3 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher3",
            arguments=['0.', '0.', '0.0', '0.0', '0.0', '0.0', 'red_standard_robot1/odom', 'red_standard_robot1/chassis'],
        )
    tf4 = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name = "static_transform_publisher4",
            arguments=['0.', '0.', '0.0', '0.0', '0.0', '0.0', 'red_standard_robot1/chassis', 'base_link'],
        )
        
     
    start_rviz_cmd = Node(
	    package='rviz2',
	    executable='rviz2',
	    name='rviz2',
	    output='screen',
    arguments=['-d', rviz_config_file])  
    

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(tf1)
    ld.add_action(tf2)
    # ld.add_action(tf3)
    ld.add_action(tf4)

    ld.add_action(robot_localization_node)
    ld.add_action(start_slam_toolbox_cmd_with_params)
    ld.add_action(start_simulator)
    ld.add_action(start_filter)
    ld.add_action(navigation2_cmd_with_params)
    ld.add_action(start_rviz_cmd)

    
    
    

    return ld
