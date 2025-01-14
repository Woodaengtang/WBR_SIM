from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the model.sdf file
    model_path = PathJoinSubstitution([
        FindPackageShare('wbr_gazebo'),
        'wbr_model',
        'model.sdf'
    ])

    # Path to Gazebo launch file
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    # Launch description
    return LaunchDescription([
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world': model_path,  # Pass the path to the SDF file as the world
            }.items()
        ),

        # Node to start the ROS 2 controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['config/controllers.yaml'],
            output='screen',
        ),

        # Spawn the joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),

        # Spawn the position controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controllers'],
        ),
    ])
