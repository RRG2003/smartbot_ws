#!/usr/bin/env python3
# """
# SmartSort Warehouse Robot - Launch File (Gazebo Harmonic Compatible)
# ROS2 Jazzy with gz_sim
# """

# import os
# from pathlib import Path
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.substitutions import Command, LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():
    
#     # Package directories
#     pkg_share = get_package_share_directory('smartsort_robot')
    
#     # Paths
#     urdf_file = os.path.join(pkg_share, 'urdf', 'smartsort_robot.urdf')
#     world_file = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    
#     # Check if files exist
#     if not os.path.exists(urdf_file):
#         urdf_file = os.path.join(os.path.expanduser('~/smartbot_ws/src/smartsort_robot/urdf'), 'smartsort_robot.urdf')
#     if not os.path.exists(world_file):
#         world_file = os.path.join(os.path.expanduser('~/smartbot_ws/src/smartsort_robot/worlds'), 'warehouse.sdf')
    
#     # Launch arguments
#     model_arg = DeclareLaunchArgument(
#         name="model",
#         default_value=urdf_file,
#         description="Absolute path to robot urdf file"
#     )
    
#     # Set Gazebo resource path
#     gazebo_resource_path = SetEnvironmentVariable(
#         name="GZ_SIM_RESOURCE_PATH",
#         value=[str(Path(pkg_share).parent.resolve())]
#     )
    
#     # Robot description
#     robot_description = ParameterValue(
#         Command(['cat ', LaunchConfiguration("model")]),
#         value_type=str
#     )
    
#     # Robot state publisher
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{
#             "robot_description": robot_description,
#             "use_sim_time": True
#         }]
#     )
    
#     # Launch Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(get_package_share_directory("ros_gz_sim"), 
#                         "launch", "gz_sim.launch.py")
#         ]),
#         launch_arguments=[
#             ("gz_args", [" -v 4 -r ", world_file])
#         ]
#     )
    
#     # Spawn robot entity
#     gz_spawn_entity = Node(
#         package="ros_gz_sim",
#         executable="create",
#         output="screen",
#         arguments=[
#             "-topic", "robot_description",
#             "-name", "smartsort_robot",
#             "-x", "0.0",
#             "-y", "0.0",
#             "-z", "0.5"
#         ],
#     )
    
#     # ROS-Gazebo bridge for topics
#     gz_ros2_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#             "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
#             "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
#             "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
#             "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
#         ],
#         remappings=[
#             ("/camera/image_raw", "/camera/image_raw"),
#         ]
#     )
    
#     # Robot brain (state machine)
#     robot_brain_node = Node(
#         package='smartsort_robot',
#         executable='robot_brain',
#         name='smartsort_brain',
#         output='screen',
#         parameters=[{'use_sim_time': True}]
#     )
    
#     # Gripper controller
#     gripper_controller_node = Node(
#         package='smartsort_robot',
#         executable='gripper_controller',
#         name='gripper_controller',
#         output='screen',
#         parameters=[{'use_sim_time': True}]
#     )
    
#     return LaunchDescription([
#         model_arg,
#         gazebo_resource_path,
#         robot_state_publisher_node,
#         gazebo,
#         gz_spawn_entity,
#         gz_ros2_bridge,
#         robot_brain_node,
#         gripper_controller_node,
#     ])


import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ----------------------------------------------------------------------
    # 🗂️ Package paths
    # ----------------------------------------------------------------------
    pkg_share = get_package_share_directory('smartsort_robot')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'smartsort_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')

    # Fallback for source workspace
    if not os.path.exists(urdf_xacro):
        urdf_xacro = os.path.expanduser('~/smartbot_ws/src/smartsort_robot/urdf/smartsort_robot.urdf.xacro')
    if not os.path.exists(world_file):
        world_file = os.path.expanduser('~/smartbot_ws/src/smartsort_robot/worlds/warehouse.sdf')

    # ----------------------------------------------------------------------
    # ⚙️ Environment setup
    # ----------------------------------------------------------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_share).parent.resolve())]
    )

    # Detect ROS distribution
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    is_ignition_default = "true" if ros_distro in ["humble", "iron", "jazzy"] else "false"
    physics_engine = "" if ros_distro in ["humble", "iron", "jazzy"] else "--physics-bullet-featherstone-plugin"

    # ----------------------------------------------------------------------
    # 🔧 Launch arguments
    # ----------------------------------------------------------------------
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=urdf_xacro,
        description="Absolute path to the robot Xacro file"
    )

    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value=is_ignition_default,
        description="If true, use Ignition Gazebo; else Gazebo Classic bridge"
    )

    # ----------------------------------------------------------------------
    # 🦾 Robot Description (from Xacro)
    # ----------------------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " is_ignition:=", LaunchConfiguration("is_ignition")
        ]),
        value_type=str
    )

    # ----------------------------------------------------------------------
    # 🦴 Robot State Publisher
    # ----------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # ----------------------------------------------------------------------
    # 🌍 Launch Gazebo Harmonic
    # ----------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r ", world_file, " ", physics_engine])
        ]
    )

    # ----------------------------------------------------------------------
    # 🤖 Spawn Robot
    # ----------------------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "smartsort_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
    )

    # ----------------------------------------------------------------------
    # 🔗 ROS-Gazebo Bridge
    # ----------------------------------------------------------------------
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/camera/image_raw", "/camera/image_raw"),
        ]
    )

    # ----------------------------------------------------------------------
    # 🧠 Robot Logic Nodes
    # ----------------------------------------------------------------------
    robot_brain_node = Node(
        package='smartsort_robot',
        executable='robot_brain',
        name='smartsort_brain',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gripper_controller_node = Node(
        package='smartsort_robot',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ----------------------------------------------------------------------
    # ✅ Launch Description
    # ----------------------------------------------------------------------
    return LaunchDescription([
        model_arg,
        is_ignition_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        robot_brain_node,
        gripper_controller_node,
    ])
