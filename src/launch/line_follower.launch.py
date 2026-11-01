#!/usr/bin/env python3
"""
Uses differential drive robot (my_robot) with camera and LiDAR.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, LogInfo, TimerAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    world_file = os.path.join(pkg_dir, 'worlds', 'indoor_line_track.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')

    rviz_config = os.path.join(pkg_dir, 'config', 'line_follower.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Point Gazebo to our models directory (for the final grading mesh)
    set_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Robot state publisher (TF from URDF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Gazebo with room world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file} --render-engine ogre2',
        }.items(),
    )

    # Spawn our custom robot from /robot_description topic
    # Position: on the grading track, facing along the nearby line segment
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', '-0.34',
            '-y', '-9.75',
            '-z', '0.0',
            '-Y', '0.96',
        ],
        output='screen',
    )

    # Bridge: Gazebo <-> ROS2 (matching our robot's topic names)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # RViz with camera image display — delayed for sim clock
    rviz = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting RViz...'),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    # Line follower node — delayed for Gazebo + camera startup
    follower = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='Starting line follower node...'),
            Node(
                package='ros2_line_follower',
                executable='line_follower',
                name='line_follower',
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    error_plotter = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg='Starting live line-error graph...'),
            Node(
                package='ros2_line_follower',
                executable='error_plotter',
                name='line_error_plotter',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        set_resource,
        LogInfo(msg='=== Launching Line Follower — Indoor Room ==='),
        robot_state_pub,
        gz_sim,
        spawn_robot,
        bridge,
        rviz,
        follower,
        error_plotter,
    ])
