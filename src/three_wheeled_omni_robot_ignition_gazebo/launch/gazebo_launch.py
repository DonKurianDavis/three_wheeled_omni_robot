from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os

def generate_launch_description():
    
    bridge_params = os.path.join(get_package_share_directory('three_wheeled_omni_robot_ignition_gazebo'), 'config', 'gz_bridge.yaml')
    world = os.path.join(get_package_share_directory('three_wheeled_omni_robot_ignition_gazebo'), 'worlds', 'ionic.sdf')
    
    joint_state_broadcaster = Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster'])
    omni_wheel_drive_controller = Node(package='controller_manager', executable='spawner',arguments=['omni_wheel_drive_controller'])
    rviz2 = Node(package='rviz2',executable='rviz2', arguments=['-d', os.path.join(get_package_share_directory('three_wheeled_omni_robot_description'),'config','default.rviz')])
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('three_wheeled_omni_robot_description'),
                    'launch',
                    'bot_launch.py'])
            ]),
            launch_arguments={
                'simulation_mode': 'true'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': ['-r -v4 ',world],
                'on_exit_shutdown': 'true'
            }.items()
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=['-topic', '/robot_description', 
                       '-name', 'three_wheeled_omni_robot',
                       '-z', '0.3'],
            output="screen"),
        
        Node(package="ros_gz_bridge",
             executable="parameter_bridge",
             arguments=['--ros-args', 
                        '-p',
                         f'config_file:= {bridge_params}']),
        
        joint_state_broadcaster,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[rviz2])),

        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[omni_wheel_drive_controller]))
    ])