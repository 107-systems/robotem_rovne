import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    pkg_imu = get_package_share_directory('pika_spark_bno085_driver')
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_imu, 'launch', 'imu.py'))
    )

    pkg_t07_robot = get_package_share_directory('t07_robot')
    t07_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_t07_robot, 'launch', 't07_tracked.py'))
    )

    robotem_rovne_node = Node(
        package='robotem_rovne',
        executable='robotem_rovne_node',
        name='robotem_rovne',
        namespace='t07',
        output='screen',
        emulate_tty=True,
        parameters=[
        ]
    )

    mandeye_unicorn_node = Node(
        package='mandeye_unicorn',
        executable='mandeye_unicorn',
        name='mandeye_unicorn',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
        ],
        remappings=[
            ('/cmd_vel', '/t07/cmd_vel'),
        ]
    )

    # Launch them all!
    return LaunchDescription([
        imu,
        t07_robot,
        robotem_rovne_node,
        mandeye_unicorn_node
    ])
