import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'atom_urdf'
    urdf_name = "atom_gazebo_control.xacro"
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare('atom_urdf'), 'worlds', 'empty.world']),
        description='World file to load in Gazebo'
    )
    
    # Robot description using Command for better error handling
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([FindPackageShare('atom_urdf'), 'urdf', urdf_name])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher - loads and publishes the robot model
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'atom_robot'],
        output='screen'
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, 
                   PathJoinSubstitution([FindPackageShare('atom_urdf'), 'config', 'atom_controllers.yaml']),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner
    ])

