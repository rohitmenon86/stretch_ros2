from moveit_configs_utils import MoveItConfigsBuilder
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare argument for database flag
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    # MoveIt! configuration builder
    moveit_config = (
        MoveItConfigsBuilder("stretch")
        .robot_description(file_path="config/stretch.urdf.xacro")
        .robot_description_semantic(file_path="config/stretch.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # Reference the MoveIt controller YAML
        .to_moveit_configs()
    )

    # RViz2 visualization node
    rviz_base = os.path.join(
        get_package_share_directory("stretch_moveit_config"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_base],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Path to the ros2_controllers.yaml
    ros2_controllers_path = os.path.join(
        get_package_share_directory("stretch_moveit_config"),
        "config",
        "ros2_controllers.yaml",  # Correct path for ros2_controllers.yaml
    )

    # Launch ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    # Spawning controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    stretch_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stretch_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Database server node if required
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    # Run the MoveGroup node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Return the full launch description
    return LaunchDescription(
        [
            db_arg,
            rviz_node,
            robot_state_publisher,
            run_move_group_node,  # Directly running MoveGroup node here
            ros2_control_node,
            mongodb_server_node,
            joint_state_broadcaster_spawner,
            stretch_controller_spawner,
        ]
    )
