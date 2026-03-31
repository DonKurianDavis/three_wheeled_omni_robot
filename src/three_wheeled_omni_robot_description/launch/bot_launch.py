import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('simulation_mode', default='true')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='true')

    xacro_file = os.path.join(
            get_package_share_directory('three_wheeled_omni_robot_description'),
            'urdf/robot',
            'omni_robot.urdf.xacro')
    robot_description = Command(['xacro ',xacro_file,' sim_mode:=',use_sim_time,' use_mock_hardware:=',use_mock_hardware])
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("simulation_mode", 
                              default_value="false",
                              description="Start robot in simulation."),
        DeclareLaunchArgument("use_mock_hardware", 
                              default_value="false",
                              description="Start robot with mock hardware mirroring command to its states."),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
    ])
