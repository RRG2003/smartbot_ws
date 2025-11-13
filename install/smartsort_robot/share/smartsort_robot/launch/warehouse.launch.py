#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =====================================================
    # PATHS AND PACKAGE SETUP
    # =====================================================
    pkg_robot = get_package_share_directory('smartsort_robot')
    pkg_controllers = get_package_share_directory('smartsort_controllers')
    
    urdf_xacro = os.path.join(pkg_robot, 'urdf', 'smartsort_robot.urdf.xacro')
    world_file = os.path.join(pkg_robot, 'worlds', 'warehouse.sdf')
    controllers_yaml = os.path.join(pkg_controllers, 'config', 'controller_manager.yaml')
    
    # Fallback to source workspace
    if not os.path.exists(urdf_xacro):
        urdf_xacro = os.path.expanduser('~/smartbot_ws/src/smartsort_robot/urdf/smartsort_robot.urdf.xacro')
    if not os.path.exists(world_file):
        world_file = os.path.expanduser('~/smartbot_ws/src/smartsort_robot/worlds/warehouse.sdf')
    if not os.path.exists(controllers_yaml):
        controllers_yaml = os.path.expanduser('~/smartbot_ws/src/smartsort_controllers/config/controller_manager.yaml')
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_robot).parent.resolve())]
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value="true",
        description="Use Gazebo Harmonic (Ignition-based)"
    )

    robot_description_content = Command([
        'xacro ', urdf_xacro,
        ' is_ignition:=', LaunchConfiguration("is_ignition")
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', [' -v 4 -r ', world_file])
        ]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_smartsort_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'smartsort_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',  # Spawn slightly above ground
        ],
    )
  
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ]
    )
    
    
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )
    
    # Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )
    
    # =====================================================
    # DELAYED CONTROLLER SPAWNING
    # =====================================================
    
    # Delay joint_state_broadcaster after spawn
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # Delay arm_controller after joint_state_broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[arm_controller_spawner]
                )
            ]
        )
    )
    
    # Delay gripper_controller after arm_controller
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[gripper_controller_spawner]
                )
            ]
        )
    )
    
    # =====================================================
    # ROBOT LOGIC NODES (delayed)
    # =====================================================
    
    robot_brain = TimerAction(
        period=6.0,  # Wait for all controllers to be ready
        actions=[
            Node(
                package='smartsort_robot',
                executable='robot_brain',
                name='smartsort_brain',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    gripper_controller_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='smartsort_robot',
                executable='gripper_controller',
                name='gripper_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # =====================================================
    # RETURN LAUNCH DESCRIPTION
    # =====================================================
    return LaunchDescription([
        # Arguments
        is_ignition_arg,
        
        # Environment
        gazebo_resource_path,
        
        # Core nodes
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        
        # Controller spawning (with delays)
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
        
        # Robot logic (delayed)
        robot_brain,
        gripper_controller_node,
    ])