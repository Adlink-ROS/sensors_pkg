from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_type = LaunchConfiguration('camera_type', default='argus')
    open_rviz = LaunchConfiguration('open_rviz', default='false')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('velodyne'), "/launch/velodyne-all-nodes-VLP16-launch.py"
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gmsl_ros2'), "/launch/gmsl_launch.py",
            ]),
            launch_arguments={'camera_type': camera_type,
                              'open_rviz': open_rviz,}.items(),
        ),
        Node(
            package='sensors_pkg',
            executable='msg_filter',
            output='screen',
            name='msg_filter',
        ),
    ])