import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'rover_ctrl'
    pkg_share = get_package_share_directory(package_name)

    start_roue_bot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'rover.launch.py')),
        launch_arguments={'use_ros2_control': 'false',
                          'use_sim_time': 'true'}.items()
    )

    rviz_config_file = os.path.join(pkg_share, 'config', 'sim.rviz')

    # rviz2
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Create the launch description and populate
    return LaunchDescription([
        start_roue_bot_cmd,
        start_rviz_cmd
    ])
