import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name='bauda_gazebo'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    joint_state_publisher_gui = Node(
        name="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_config_file = os.path.join(get_package_share_directory('description_bauda_akm'), 'config', 'view_bot.rviz')
    rviz = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher_gui,
        rviz
    ])