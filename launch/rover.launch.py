import os

from ament_index_python.packages import get_package_share_directory


from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Check if we're told to use sim time
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='false')

    # Process the URDF file
    pkg_share = os.path.join(get_package_share_directory('rover_ctrl'))
    xacro_file = os.path.join(pkg_share, 'description', 'rover.urdf.xacro')

    pkg_gazebo_ros = pkg_gazebo_ros = FindPackageShare(
        package='gazebo_ros').find('gazebo_ros')

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(
        ['xacro ', xacro_file,
         ' use_ros2_control:=', use_ros2_control,
         ' sim_mode:=', use_sim_time])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time}

    # Spawn robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover',
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            # '-z', '0.1'
        ],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(
            pkg_share, 'worlds', 'maze.world')}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity_cmd,
        node_robot_state_publisher
    ])
