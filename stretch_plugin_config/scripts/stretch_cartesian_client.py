#!/usr/bin/env python3
"""
Move the Stretch robot (base + arm) to a Cartesian pose goal.

Usage examples:

# Move to a Cartesian pose (end-effector at position + orientation)
ros2 run stretch_plugin_config pose_goal_client.py --ros-args \
  -p position:="[0.5, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

# Run asynchronously and cancel after 2s
ros2 run stretch_plugin_config pose_goal_client.py --ros-args \
  -p position:="[0.5, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p synchronous:=False -p cancel_after_secs:=2.0
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init()

    node = Node("stretch_pose_goal_client")

    # Declare parameters
    node.declare_parameter("position", [0.5, 0.0, 1.0])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("synchronous", True)
    node.declare_parameter("cancel_after_secs", 0.0)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    callback_group = ReentrantCallbackGroup()

    # --- Stretch full-body group ---
    joint_names = [
        "joint_gripper_finger_right",
        # "joint_head_pan",
        "joint_gripper_finger_left",
        # "joint_head_tilt",
        "joint_right_wheel",
        "joint_wrist_yaw",
        "joint_lift",
        "joint_wrist_pitch",
        "joint_left_wheel",
        "joint_wrist_roll",
        "joint_arm_l0",
        "joint_arm_l1",
        "joint_arm_l2",
        "joint_arm_l3",
    ]

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name="base_link",
        end_effector_name="link_wrist_roll",
        group_name="stretch_base_arm",   # full robot group
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Start executor in background
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # --- Get params ---
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value
    )

    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # --- Execute ---
    node.get_logger().info(
        f"Moving stretch_base_arm to pose "
        f"position={list(position)}, quat={list(quat_xyzw)}"
    )
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
    )

    if synchronous:
        moveit2.wait_until_executed()
    else:
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        future = moveit2.get_execution_future()
        if cancel_after_secs > 0.0:
            node.create_rate(cancel_after_secs).sleep()
            node.get_logger().warn("Cancelling goal")
            moveit2.cancel_execution()

        while not future.done():
            rate.sleep()

        node.get_logger().info(
            f"Result status: {future.result().status}, "
            f"error_code: {future.result().result.error_code}"
        )

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
