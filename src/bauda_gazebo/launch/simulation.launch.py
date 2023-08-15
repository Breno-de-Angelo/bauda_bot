import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete
from launch.actions import RegisterEventHandler

def generate_launch_description():

    package_name = 'bauda_gazebo'
    package_path = get_package_share_directory(package_name)
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_path, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_world_file = os.path.join(package_path, 'world', 'obstacles.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'pause': 'false', 'world' : gazebo_world_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '0.05'],
        output='screen')
    
    # delay_spawn_entity_after_rsp = RegisterEventHandler(
    #     OnExecutionComplete(
    #         target_action=rsp,
    #         on_completion=[spawn_entity]
    #     )
    # )

    joystick = Node(
        package='joy',
        executable='joy_node'
    )

    teleop_config = os.path.join(
        get_package_share_directory('ackermann_teleop_twist_joy'), 'config', 'xbox.config.yaml')
    
    teleop_joy = Node(
        package='ackermann_teleop_twist_joy',
        executable='teleop_node',
        parameters=[teleop_config],
        remappings=[('/cmd_vel', '/akm_cont/reference_unstamped')]
    )
    
    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["akm_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    # lidar_joint_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["lidar_joint_controller"]
    # )    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joystick,
        teleop_joy,
        drive_spawner,
        joint_broad_spawner,
    ])