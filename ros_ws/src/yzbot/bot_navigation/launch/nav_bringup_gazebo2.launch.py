#!/usr/bin/python3
# nav_bringup_gazebo2.launch.py — 自愈型导航栈 v3（2026-09-07 重写）
#
# 相比官方 bringup_launch 的关键改动：AMCL 不再启动、不再被 lifecycle 管理。
# 背景（2026-09-06 23:19 事故复盘）：
#   start_all 曾用 `ros2 lifecycle set /amcl shutdown` 停用 AMCL（truth_odom +
#   loc_shim 已接管定位）。shutdown 把 AMCL 打入 finalized 终态；此后任何
#   bond 心跳风暴（2 vCPU 饥荒 4s 即可触发，当时 controller_server 与
#   map_server 心跳同时丢失）都会让两个 lifecycle_manager 全量 reset 并
#   re-bringup——re-bringup 在 AMCL 的 finalized 上必然 configure 失败，
#   "Aborting bringup" 使 map_server 永不激活、/map 消失、global costmap
#   退化为 5x5m 空图、激光消息全部被丢，导航全灭且无法自愈（昨晚整轮
#   零抓取的直接根因）。
# 方案：定位链只保留 map_server（lifecycle_manager_localization 只管它）。
#   map->odom TF 与 /amcl_pose 由 loc_shim 提供。此后任何 reset 后
#   re-bringup 全链成功，map_server 重新激活并 latched 重发 /map，
#   costmap 自动恢复——闭环自愈。

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation2_dir = get_package_share_directory('bot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file = 'mapn3.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(navigation2_dir, f'maps/{map_yaml_file}'))
    nav2_param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(navigation2_dir, 'param', 'originbot_nav2_2.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_path}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_param_path,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # 只管 map_server：AMCL 已从系统移除（见文件头事故复盘）
    localization_nodes = ['map_server']

    load_composable_nodes = LoadComposableNodes(
        target_container='/nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': localization_nodes,
                             # 心跳容忍放宽：2 vCPU 饥荒 4-8s 是常态，
                             # 默认 4s 会频繁触发全量 reset
                             'bond_timeout': 10.0}]),
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=nav2_param_path,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'map', default_value=map_yaml_path,
            description='Full path to map file to load'),

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        GroupAction([
            # 组合容器：官方 bringup_launch 同款（map_server 与导航节点同容器）
            Node(
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                output='screen'),
            # 定位：map_server-only（AMCL 由 truth_odom+loc_shim 替代，见文件头）
            load_composable_nodes,
            # 导航：官方 navigation_launch 原样引入（controller/planner/bt/...）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': nav2_param_path,
                    'use_composition': 'True',
                    'container_name': 'nav2_container'}.items()),
        ]),

        # 无显示环境(GAZEBO_HEADLESS=1)下跳过 RViz：软件渲染会吃掉约 1/3 个 vCPU
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(EnvironmentVariable('RVIZ_ENABLE', default_value='1'))),
    ])
