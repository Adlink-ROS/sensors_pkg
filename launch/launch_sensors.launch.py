from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    camera_type = LaunchConfiguration('camera_type', default='argus')
    open_rviz = LaunchConfiguration('open_rviz', default='false')
    enable_imu = LaunchConfiguration('enable_imu', default='true')

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('bluespace_ai_xsens_mti_driver'), "/launch/xsens_mti_node.launch.py"
            ]),
            condition=IfCondition(PythonExpression(['"', enable_imu, '" == "true"']))
        ),
        Node(
            package='sensors_pkg',
            executable='msg_filter_with_imu',
            output='screen',
            name='msg_filter_with_imu',
            condition=IfCondition(PythonExpression(['"', enable_imu, '" == "true"']))
        ),
        Node(
            package='sensors_pkg',
            executable='msg_filter',
            output='screen',
            name='msg_filter',
            condition=IfCondition(PythonExpression(['"', enable_imu, '" == "false"']))
        ),
    ])