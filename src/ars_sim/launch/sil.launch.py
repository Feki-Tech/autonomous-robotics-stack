"""Software-in-the-loop pipeline: Gazebo Harmonic + the full ARS stack.

Starts the depot world (headless by default), bridges cmd_vel / joint_states /
clock, and runs the odometry -> planner -> controller chain. Publish a goal to
drive the robot:

    ros2 launch ars_sim sil.launch.py
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        '{pose: {position: {x: 2.0, y: -1.0}}}'

Pass mission:=true to also run the waypoint mission node, which dispatches the
configured waypoints autonomously instead of waiting for manual goals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('ars_sim')
    world = os.path.join(sim_share, 'worlds', 'depot.sdf')
    bridge_config = os.path.join(sim_share, 'config', 'bridge.yaml')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    headless = LaunchConfiguration('headless')
    mission = LaunchConfiguration('mission')
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run the Gazebo server without a GUI'),
        DeclareLaunchArgument(
            'mission', default_value='false',
            description='Run the waypoint mission node'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r -s {world}'}.items(),
            condition=IfCondition(headless)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
            condition=UnlessCondition(headless)),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}, use_sim_time],
            output='screen'),

        Node(
            package='ars_ros2',
            executable='odometry_node_main',
            name='ars_odometry',
            parameters=[{
                'wheel_radius': 0.1,
                'wheel_separation': 0.5,
                'left_joint': 'left_wheel_joint',
                'right_joint': 'right_wheel_joint',
            }, use_sim_time],
            output='screen'),

        Node(
            package='ars_ros2',
            executable='planner_node_main',
            name='ars_planner',
            parameters=[use_sim_time],
            output='screen'),

        Node(
            package='ars_ros2',
            executable='controller_node_main',
            name='ars_controller',
            parameters=[{'cruise_speed': 0.5, 'goal_tolerance': 0.1}, use_sim_time],
            output='screen'),

        Node(
            package='ars_ros2',
            executable='mission_node_main',
            name='ars_mission',
            parameters=[{
                'waypoints': [2.0, -1.0, 3.0, 1.5, 0.0, 0.0],
                'goal_tolerance': 0.2,
                'progress_timeout': 30.0,
            }, use_sim_time],
            output='screen',
            condition=IfCondition(mission)),
    ])
