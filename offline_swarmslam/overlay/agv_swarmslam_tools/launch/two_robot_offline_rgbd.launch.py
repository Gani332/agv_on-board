import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def cslam_launch(robot_id):
    tools_share = get_package_share_directory("agv_swarmslam_tools")
    cslam_exp_share = get_package_share_directory("cslam_experiments")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cslam_exp_share, "launch", "cslam", "cslam_rgbd.launch.py")
        ),
        launch_arguments={
            "config_path": os.path.join(tools_share, "config/"),
            "config_file": "agv_realsense_rgbd.yaml",
            "robot_id": str(robot_id),
            "namespace": f"/r{robot_id}",
            "max_nb_robots": "2",
        }.items(),
    )


def bag_play_process(name, bag_arg, robot_ns):
    return ExecuteProcess(
        name=name,
        output="screen",
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration(bag_arg),
            "--clock",
            "--rate",
            LaunchConfiguration("playback_rate"),
            "--remap",
            f"/camera/color/image_raw:=/{robot_ns}/color/image_raw",
            f"/camera/color/camera_info:=/{robot_ns}/color/camera_info",
            f"/camera/aligned_depth_to_color/image_raw:=/{robot_ns}/aligned_depth_to_color/image_raw",
            f"/odom:=/{robot_ns}/odom",
        ],
    )


def generate_launch_description():
    tools_share = get_package_share_directory("agv_swarmslam_tools")
    rviz_config = os.path.join(tools_share, "rviz", "agv_swarmslam.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot0_bag", default_value="/data/ros2/robotA"),
            DeclareLaunchArgument("robot1_bag", default_value="/data/ros2/robotB"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("apriltag_graph_constraints", default_value="false"),
            DeclareLaunchArgument(
                "apriltag_alignment_file",
                default_value="/results/apriltags_pass1/robotA_to_robotB_alignment.json",
            ),
            DeclareLaunchArgument(
                "apriltag_detections_file",
                default_value="/results/apriltags_pass1/detections.csv",
            ),
            cslam_launch(0),
            cslam_launch(1),
            ExecuteProcess(
                name="apriltag_loop_closure_publisher",
                output="screen",
                cmd=[
                    "python3",
                    "-m",
                    "agv_swarmslam_tools.apriltag_loop_closure_publisher",
                    "--ros-args",
                    "-p",
                    ["alignment_file:=", LaunchConfiguration("apriltag_alignment_file")],
                    "-p",
                    ["detections_file:=", LaunchConfiguration("apriltag_detections_file")],
                ],
                condition=IfCondition(LaunchConfiguration("apriltag_graph_constraints")),
            ),
            Node(
                package="agv_swarmslam_tools",
                executable="viz_bridge",
                name="viz_bridge",
                output="screen",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    bag_play_process("play_robot0_bag", "robot0_bag", "r0"),
                    bag_play_process("play_robot1_bag", "robot1_bag", "r1"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
