#!/usr/bin/env python3
"""
Move Stretch robot groups to a joint configuration via ROS2 parameters.

Usage examples:

# Use arm with custom positions
ros2 run stretch_plugin_config joint_client.py --ros-args \
  -p group_name:=stretch_arm \
  -p joint_positions:=[7.55634e-05,0.46180056,-2.8448e-05,9.66159e-05,0.12993342,0.12993322,0.12996057,0.12990442]

# Use arm with defaults
ros2 run stretch_plugin_config joint_client.py --ros-args -p group_name:=stretch_arm
"""

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init()
    node = Node("stretch_joints_client")

    joint_groups = {
        "stretch_arm": [
            "joint_wrist_yaw",
            "joint_lift",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
        ],
        "stretch_head": ["joint_head_pan", "joint_head_tilt"],
        "stretch_gripper": ["joint_gripper_finger_right", "joint_gripper_finger_left"],
        "stretch_base": ["joint_left_wheel", "joint_right_wheel"],
        "stretch_base_arm": [
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
        ],
    }

    group_defaults = {
        "stretch_arm": [7.55634e-05, 0.46180056, -2.8448e-05,
                        9.66159e-05, 0.12993342, 0.12993322,
                        0.12996057, 0.12990442],
        "stretch_head": [0.0, 0.0],
        "stretch_gripper": [0.0, 0.0],
        "stretch_base": [0.0, 0.0],
        "stretch_base_arm": [0.0, 
                            #  0.0, 
                             0.0, 
                            #  0.0, 
                             0.0,
                         7.72e-05, 0.4982, 5.57e-05,
                         0.0, 6.73e-05, 2.99e-05,
                         8.93e-05, 7.92e-05, 4.23e-05],
    }


    node.declare_parameter("group_name", "stretch_arm")
    node.declare_parameter("joint_positions", [0.0]) 
    node.declare_parameter("synchronous", True)
    node.declare_parameter("cancel_after_secs", 0.0)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

    group_name = node.get_parameter("group_name").get_parameter_value().string_value
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    planner_id = node.get_parameter("planner_id").get_parameter_value().string_value

    if group_name not in joint_groups:
        node.get_logger().error(
            f"Unknown group '{group_name}'. Available: {list(joint_groups.keys())}"
        )
        rclpy.shutdown()
        return

    joint_names = joint_groups[group_name]

    # Fallback to defaults if only [0.0] provided
    if len(joint_positions) == 1 and joint_positions[0] == 0.0:
        node.get_logger().warn(
            f"No joint_positions provided. Using defaults for {group_name}"
        )
        joint_positions = group_defaults[group_name]

    # Check lengths
    if len(joint_positions) != len(joint_names):
        node.get_logger().error(
            f"Mismatch: {len(joint_positions)} positions provided but group '{group_name}' expects {len(joint_names)} joints."
        )
        rclpy.shutdown()
        return

    # --- MoveIt2 interface ---
    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name="base_link",
        end_effector_name="link_gripper_s3_body",
        group_name=group_name,
        callback_group=callback_group,
    )
    moveit2.planner_id = planner_id
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 0.2

    # --- Spin executor ---
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # --- Execution ---
    node.get_logger().info(f"Moving {group_name} to {list(joint_positions)}")
    moveit2.move_to_configuration(joint_positions)

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


if __name__ == "__main__":
    main()
