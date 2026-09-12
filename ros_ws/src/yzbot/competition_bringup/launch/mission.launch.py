# -*- coding: utf-8 -*-
"""一键启动：任务调度 + LLM 解析 + 移动障碍（仿真与导航需另行启动）。

启动参数：
    api_key     云端 API Key（默认取环境变量 DEEPSEEK_API_KEY）
    mapping     比赛公布的映射规则
    waypoints   航点文件（默认包内 config/waypoints.yaml）
    obstacles   是否启动移动障碍节点（true/false）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_API_KEY = os.environ.get('DEEPSEEK_API_KEY', '')


def generate_launch_description():
    api_key = LaunchConfiguration('api_key')
    mapping = LaunchConfiguration('mapping')
    waypoints = LaunchConfiguration('waypoints')
    obstacles = LaunchConfiguration('obstacles')

    default_wp = os.path.join(
        get_package_share_directory('competition_bringup'), 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('api_key', default_value=DEFAULT_API_KEY),
        DeclareLaunchArgument(
            'mapping', default_value='x代表红色,y代表蓝色; 数量大于等于3的去A区, 其他去C区'),
        DeclareLaunchArgument('waypoints', default_value=default_wp),
        DeclareLaunchArgument('obstacles', default_value='true'),
        Node(
            package='competition_bringup',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{'waypoints_file': waypoints,
                         'use_sim_time': True}],
        ),
        Node(
            package='competition_bringup',
            executable='llm_parser',
            name='llm_parser',
            output='screen',
            parameters=[{'api_key': api_key, 'mapping': mapping,
                         'use_sim_time': True}],
        ),
        Node(
            package='competition_bringup',
            executable='obstacle_mover',
            name='obstacle_mover',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(obstacles),
        ),
        Node(
            package='competition_bringup',
            executable='cmd_vel_watchdog',
            name='cmd_vel_watchdog',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='competition_bringup',
            executable='carry_follower',
            name='carry_follower',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='competition_bringup',
            executable='scan_filter',
            name='scan_filter',
            output='screen',
            parameters=[{'use_sim_time': True}],
            respawn=True,
        ),
    ])