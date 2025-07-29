from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


from launch_ros.actions import Node
import sys

# from hybrid_planning_common import (generate_common_hybrid_launch_description, load_yaml)


def generate_launch_description():
    print("PYTHON EXECUTABLE:", sys.executable)
    moveit_config_path = get_package_share_path('stretch_moveit2')
    stretch_core_path = get_package_share_path('stretch_core')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('pipeline', default_value='ompl', description='specify the planning pipeline'))
    ld.add_action(DeclareLaunchArgument('db', default_value='false', choices=['true', 'false'],
                                        description='By default, we do not start a database (it can be large)'))

    ld.add_action(DeclareLaunchArgument('db_path', default_value=str(moveit_config_path / 'default_warehouse_mongo_db'),
                                        description='Allow user to specify database location'))
    ld.add_action(DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false'],
                                        description='By default, we are not in debug mode'))

    ld.add_action(DeclareLaunchArgument('use_stretch_driver', default_value='true', choices=['true', 'false'],
                                        description='Allow user to launch Stretch Driver separately'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']))

    # Load the URDF, SRDF and other .yaml configuration files
    stretch_description_path = get_package_share_path('stretch_description')
    urdf_path = stretch_description_path / 'urdf' / 'stretch.urdf'
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    print(robot_description_content)

    with open(moveit_config_path / 'config/stretch_description.srdf', 'r') as f:
        semantic_content = f.read()

    robot_description_param = ParameterValue(robot_description_content, value_type=str)
    semantic_param = ParameterValue(semantic_content, value_type=str)

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    ld.add_action(joint_state_publisher)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    ld.add_action(robot_state_publisher)

    #stretch_driver_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
    #    launch_arguments={'mode': 'trajectory', 'broadcast_odom_tf': 'True'}.items(),
    #    condition=IfCondition(LaunchConfiguration('use_stretch_driver'))
    #)
    #ld.add_action(stretch_driver_launch)

    # Run the main MoveIt executable
    move_group_launch_py = PythonLaunchDescriptionSource(str(moveit_config_path / 'launch/move_group.launch.py'))
    move_group_launch_args = {
        'allow_trajectory_execution': 'true',
        'fake_execution': 'false',
        'info': 'true',
        'debug': LaunchConfiguration('debug'),
        'pipeline': LaunchConfiguration('pipeline'),
        'robot_description': robot_description_content,
        'semantic_config': semantic_content,
    }
    move_group_launch = IncludeLaunchDescription(move_group_launch_py,
                                                 launch_arguments=move_group_launch_args.items())
    ld.add_action(move_group_launch)

    movegroup_test_py = PythonLaunchDescriptionSource(str(moveit_config_path / 'launch/movegroup_test.launch.py'))
    movegroup_test = IncludeLaunchDescription(movegroup_test_py,
                                                 launch_arguments=move_group_launch_args.items())
    ld.add_action(movegroup_test)

    # Run Rviz and load the default config to see the state of the move_group node
    moveit_rviz_launch_py = PythonLaunchDescriptionSource(
        str(moveit_config_path / 'launch/moveit_rviz.launch.py')
    )
    moveit_rviz_args = {
        'rviz_config': str(moveit_config_path / 'launch/moveit.rviz'),
        'debug': LaunchConfiguration('debug'),
        'robot_description': robot_description_content,
        'semantic_config': semantic_content,
        #'robot_description': robot_description_param,
        #'semantic_config': semantic_param,
    }
    moveit_rviz_launch = IncludeLaunchDescription(moveit_rviz_launch_py, launch_arguments=moveit_rviz_args.items(),
                                                  condition=IfCondition(LaunchConfiguration('use_rviz')))
    ld.add_action(moveit_rviz_launch)

    # If database loading was enabled, start mongodb as well
    warehouse_launch_py = PythonLaunchDescriptionSource(
        str(moveit_config_path / 'launch/warehouse_db.launch.py')
    )
    warehouse_launch = IncludeLaunchDescription(warehouse_launch_py,
                                                launch_arguments={'moveit_warehouse_database_path':
                                                                  LaunchConfiguration('db_path')}.items(),
                                                condition=IfCondition(LaunchConfiguration('db'))
                                                )
    ld.add_action(warehouse_launch)

    return ld
