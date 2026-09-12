from setuptools import find_packages, setup
from glob import glob

package_name = 'competition_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@u22.local',
    description='大模型技术创新赛 任务调度/导航/抓放/LLM解析/状态反馈/移动障碍/出题桥接',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = competition_bringup.mission_node:main',
            'llm_parser = competition_bringup.llm_parser:main',
            'save_pose = competition_bringup.save_pose:main',
            'question_bridge = competition_bringup.question_bridge:main',
            'obstacle_mover = competition_bringup.obstacle_mover:main',
            'cmd_vel_watchdog = competition_bringup.cmd_vel_watchdog:main',
            'carry_follower = competition_bringup.carry_follower:main',
            'scan_filter = competition_bringup.scan_filter:main',
        ],
    },
)
