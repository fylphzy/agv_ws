from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('odom_mode')

    bringup_share = get_package_share_directory('agv_bringup')
    view_robot_launch = os.path.join(bringup_share, 'launch', 'view_robot.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('odom_mode', default_value='circle'),  # circle|static

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_robot_launch),
            launch_arguments={'rviz': use_rviz}.items(),
        ),

        Node(
            package='agv_dummy',
            executable='odom_dummy',
            name='odom_dummy',
            output='screen',
            parameters=[{
                'mode': mode,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'rate_hz': 30.0,
                'radius': 0.5,
                'yaw_rate': 0.3,
            }],
        ),
    ])
