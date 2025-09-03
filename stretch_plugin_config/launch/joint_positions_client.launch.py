#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    group_name = LaunchConfiguration("group_name").perform(context)
    joint_positions = LaunchConfiguration("joint_positions").perform(context)
    home_positions = LaunchConfiguration("home_positions").perform(context).lower() in ["true", "1", "yes"]

    group_defaults = {
        "stretch_arm": "[0.9263,0.4440,-0.3036,1.9753,0.06358,0.07634,0.07854,0.08951]",
        "stretch_head": "[-3.1658e-05,9.0956e-05]",
        "stretch_gripper": "[0.3986,0.5745]",
        "stretch_base": "[8.6713e-06,2.1752e-05]",
        "stretch_base_arm": "[0.3986,0.5745,2.1752e-05,0.9263,0.4440,-0.3036,8.6713e-06,1.9753,0.06358,0.07634,0.07854,0.08951]",
    }


    home_defaults = {
        "stretch_arm": "[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "stretch_head": "[0.0,0.0]",
        "stretch_gripper": "[0.0,0.0]",
        "stretch_base": "[0.0,0.0]",
        "stretch_base_arm": "[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
    }

    if home_positions:
        joint_positions = home_defaults.get(group_name, "[]")
    elif joint_positions in ("", "[]"):
        joint_positions = group_defaults.get(group_name, "[]")

    return [
        Node(
            package="stretch_plugin_config",
            executable="stretch_joints_client.py",
            name="stretch_joints_client",
            output="screen",
            parameters=[{
                "group_name": group_name,
                "joint_positions": eval(joint_positions),
                "synchronous": LaunchConfiguration("synchronous"),
                "cancel_after_secs": LaunchConfiguration("cancel_after_secs"),
                "planner_id": LaunchConfiguration("planner_id"),
            }]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("group_name", default_value="stretch_arm",
                              description="MoveIt2 planning group to control"),
        DeclareLaunchArgument("joint_positions", default_value="[]",
                              description="Target joint positions (list); uses defaults if not set"),
        DeclareLaunchArgument("home_positions", default_value="False",
                              description="If true, use home joint positions instead of defaults"),
        DeclareLaunchArgument("synchronous", default_value="True",
                              description="Wait until execution finishes"),
        DeclareLaunchArgument("cancel_after_secs", default_value="0.0",
                              description="Cancel goal after N seconds (only if synchronous:=False)"),
        DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault",
                              description="OMPL planner ID"),

        OpaqueFunction(function=launch_setup),
    ])
