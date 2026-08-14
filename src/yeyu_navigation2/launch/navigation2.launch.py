# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')

# ---- 로봇 시작 위치 (map 좌표계 기준, RViz에서 확인 후 값 수정) ----
INITIAL_POSE_X = '-2.73'
INITIAL_POSE_Y = '1.75'
INITIAL_POSE_Z_ORIENTATION = '0.04998'   # yaw 0.1 rad -> quaternion z
INITIAL_POSE_W_ORIENTATION = '0.99875'   # yaw 0.1 rad -> quaternion w
INITIAL_POSE_DELAY = '15.0'           # nav2가 완전히 뜨는 데 걸리는 시간(초), 환경에 따라 조정
# ------------------------------------------------------------------



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('yeyu_navigation2'),
            'map',
            'map2.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('yeyu_navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('yeyu_navigation2'),
                'param',
                param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('yeyu_navigation2'),
        'rviz',
        'tb3_navigation2.rviz')

    # AMCL이 완전히 뜬 후 /initialpose를 자동으로 한 번 퍼블리시
    initial_pose_pub = TimerAction(
        period=float(INITIAL_POSE_DELAY),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, '
                    'pose: {pose: {position: {x: ' + INITIAL_POSE_X +
                    ', y: ' + INITIAL_POSE_Y + ', z: 0.0}, '
                    'orientation: {z: ' + INITIAL_POSE_Z_ORIENTATION +
                    ', w: ' + INITIAL_POSE_W_ORIENTATION + '}}, '
                    'covariance: [0.25, 0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, '
                    '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch RViz2'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz)),

        initial_pose_pub,
    ])