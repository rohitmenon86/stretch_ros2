// This is a test script to demonstrate the move group API for MoveIt 2 with the Stretch RE1 robot
// It is based on the MoveGroup C++ tutorial for MoveIt 2

#include <pluginlib/class_loader.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#define GOAL_POS_TOLERENCE 0.000001


void printMotionPlanResults(const planning_interface::MotionPlanResponse& res,
            const moveit::core::JointModelGroup* joint_model_group)
{
    if (!res.trajectory_)
    {
        std::cout << "No trajectory in the planning result." << std::endl;
        return;
    }

    // Meta data
    std::cout << "Planning time: " << res.planning_time_ << " seconds" << std::endl;

    // Number of waypoints
    size_t waypoint_count = res.trajectory_->getWayPointCount();
    std::cout << "Number of waypoints in trajectory: " << waypoint_count << std::endl;

    std::vector<double> positions;

    // Start & goal states
    if (waypoint_count > 0)
    {
        const auto& start_state = res.trajectory_->getFirstWayPoint();
        const auto& goal_state = res.trajectory_->getLastWayPoint();

        std::cout << "Start state joint values:" << std::endl;
        // Print positions for each joint group
        std::vector<double> positions;
        start_state.copyJointGroupPositions(joint_model_group, positions);

        for (double pos : positions)
            std::cout << pos << " ";
        std::cout << std::endl;

        std::cout << "Goal state joint values:" << std::endl;
        goal_state.copyJointGroupPositions(joint_model_group, positions);

        for (double pos : positions)
            std::cout << pos << " ";
        std::cout << std::endl;
    }

    std::cout << "\n--- All Waypoints ---\n";
    for (size_t i = 0; i < waypoint_count; ++i)
    {
        const auto& state = res.trajectory_->getWayPoint(i);
        state.copyJointGroupPositions(joint_model_group, positions);

        double t = res.trajectory_->getWayPointDurationFromStart(i);
        std::cout << "Waypoint " << i << " | Time from start: " << t << "s | Positions: ";
        for (double pos : positions)
            std::cout << pos << " ";
        std::cout << "\n";
    }

    std::cout << "\nTotal trajectory duration: " 
              << res.trajectory_->getDuration() << " seconds\n";

    // Total trajectory duration
    double total_time = res.trajectory_->getDuration();
    std::cout << "Total trajectory duration: " << total_time << " seconds" << std::endl;

}


// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("movegroup_test", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ########################### Setup ##########################
  // Setup
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably
  static const std::string PLANNING_GROUP_HEAD = "stretch_head";
  static const std::string PLANNING_GROUP_GRIPPER = "stretch_gripper";
  static const std::string PLANNING_GROUP_ARM = "stretch_arm";
  static const std::string PLANNING_GROUP_BASE = "mobile_base";
  static const std::string PLANNING_GROUP_BASE_ARM = "mobile_base_arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group_head(move_group_node, PLANNING_GROUP_HEAD);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);
  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_base(move_group_node, PLANNING_GROUP_BASE);
  moveit::planning_interface::MoveGroupInterface move_group_base_arm(move_group_node, PLANNING_GROUP_BASE_ARM);

  // We lower the allowed maximum velocity and acceleration to 80% of their maximum.
  // The default values are 10% (0.1)
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster
  move_group_head.setMaxVelocityScalingFactor(1.0);
  move_group_head.setMaxAccelerationScalingFactor(1.0);
  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);
  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);
  move_group_base.setMaxVelocityScalingFactor(1.0);
  move_group_base.setMaxAccelerationScalingFactor(1.0);
  move_group_base_arm.setMaxVelocityScalingFactor(1.0);
  move_group_base_arm.setMaxAccelerationScalingFactor(1.0);

  // Raw pointers are frequently used to refer to the planning group for improved performance
  const moveit::core::JointModelGroup* joint_model_group;
  const moveit::core::LinkModel* ee_parent_link = 
      move_group_head.getCurrentState()->getLinkModel("link_grasp_center"); // the link name can be changed here to visualize the trajectory of the corresponding link

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "odom", "move_group_visualization",
                                                      move_group_head.getRobotModel());
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Getting Basic Information
  // We can print the name of the reference frame for this robot
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_head.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Now, we call the planner to compute the plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;
  std::vector<double> joint_group_positions;

  // We'll create an pointer that references the current robot's state
  // RobotState is the object that contains all the current position/velocity/acceleration data
  moveit::core::RobotStatePtr current_state;

  // ########################### Test ##########################

  std::vector<std::string> planning_joints = {"position/x", "position/y", "position/theta"};


  robot_model_loader::RobotModelLoader robot_model_loader(move_group_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_BASE);
  //joint_model_group = move_group_base.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE);
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::string planner_plugin_name;
  std::string planner_namespace;

  if (!move_group_node->get_parameter("planning_namespace", planner_namespace))
      RCLCPP_FATAL(LOGGER, "Could not find planner namespace name");

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!move_group_node->get_parameter("planning_plugin", planner_plugin_name))
      RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");

  try
  { 
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
									"moveit_core", "planning_interface::PlannerManager"));
  } 
  catch (pluginlib::PluginlibException& ex)
  { 
      RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  } 
  try
  {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      RCLCPP_INFO(LOGGER, "Hao: namespace: %s, %s", move_group_node->get_namespace(), planner_namespace.c_str());
      if (!planner_instance->initialize(robot_model, move_group_node, "move_group"))
		  RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
      RCLCPP_INFO(LOGGER, "Planning Plugin Name '%s'", planner_plugin_name.c_str());
      RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  { 
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (const auto& cls : classes)
          ss << cls << " ";
      RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                          ex.what(), ss.str().c_str());
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the planning");
  visual_tools.trigger();

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = PLANNING_GROUP_BASE;

  current_state = move_group_base.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  req.workspace_parameters.min_corner.x = -5.0; // min X
  req.workspace_parameters.max_corner.x =  5.0; // max X
  req.workspace_parameters.min_corner.y = -5.0; // min Y
  req.workspace_parameters.max_corner.y =  5.0; // max Y
  req.workspace_parameters.min_corner.z = -2.1; // min Z (often 0 for planar)
  req.workspace_parameters.max_corner.z =  4.2; // max Z (not used in planar, but required)

  req.workspace_parameters.header.frame_id = "odom"; // should match your planning frame

  for(int n = 0; n < 3; n++) {
    req.start_state.joint_state.name.push_back(planning_joints[n]);
    req.start_state.joint_state.position.push_back(joint_group_positions[n]);
  }

  /* dummy goal, as the real goals is generated in the server side*/
  moveit_msgs::msg::Constraints goal;
  goal.name = "Goal_0";
  for(int n = 0; n < 3; n++) {
    moveit_msgs::msg::JointConstraint joint;
    joint.joint_name = planning_joints[n];
    joint.position = 0.0;
    joint.tolerance_above = GOAL_POS_TOLERENCE;
    joint.tolerance_below = GOAL_POS_TOLERENCE;
    joint.weight = 1.0;
    goal.joint_constraints.push_back(joint);
  }
  req.goal_constraints.push_back(goal);

  req.allowed_planning_time = 120;
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

#if 1
  context->solve(res);

  trajectory_processing::IterativeParabolicTimeParameterization time_param;

  success = time_param.computeTimeStamps(*res.trajectory_, /*velocity_scaling=*/0.2);
  if(success)
      std::cout << "Compute timestamp successfully." << std::endl;
  else
      std::cout << "Failed to compute timestamp." << std::endl;

  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  /* Move to the start state */
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  res.trajectory_->getRobotTrajectoryMsg(trajectory_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;
  plan.start_state_ = response.trajectory_start;
  plan.planning_time_ = response.planning_time;

  moveit::core::RobotState start_state(robot_model);
  moveit::core::robotStateMsgToRobotState(plan.start_state_, start_state);
  move_group_base.setJointValueTarget(start_state);
  move_group_base.setPlanningPipelineId("");
  move_group_base.setPlannerId("RRTConnectkConfigDefault");

  if(move_group_base.move() == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Move to start state successfully.");
  } else {
      RCLCPP_INFO(LOGGER, "Failed to move to the start state.");
  }

  printMotionPlanResults(res, joint_model_group);

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the trajectory");

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
          move_group_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);

  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);

  //std::cout << response.trajectory.joint_trajectory.points.back().positions.size() << std::endl;

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the trajectory");

  if(move_group_base.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Move to target successfully.");
  } else {
      RCLCPP_INFO(LOGGER, "Failed to move to the target.");
  }

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");
#endif

  // ########################### Step 4 ##########################
  joint_model_group = move_group_base.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Mobile_Base_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base.getPlanningFrame().c_str());

  current_state = move_group_base.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base to the right and forward, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0.5;   // 0 to inf, position/x
  joint_group_positions[1] = -0.5;    // 0 to inf, position/y
  joint_group_positions[2] = 0;  // 0 to 3.14, position/theta
  move_group_base.setJointValueTarget(joint_group_positions);

  success = (move_group_base.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base.move();

  current_state = move_group_base.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base to the initial position, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  move_group_base.setJointValueTarget(joint_group_positions);

  success = (move_group_base.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base.move();

  // ########################### Step 5 ##########################
  joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Mobile_Base_Arm_Group", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base.getPlanningFrame().c_str());

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to stow the arm and move forward, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0.5;   // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  joint_group_positions[3] = 0.2;    // 0 to 1.1, joint_lift
  joint_group_positions[4] = 0;      // 0 to 0.130, joint_arm_l3
  joint_group_positions[5] = 0;      // 0 to 0.130, joint_arm_l2
  joint_group_positions[6] = 0;      // 0 to 0.130, joint_arm_l1
  joint_group_positions[7] = 0;      // 0 to 0.130, joint_arm_l0
  joint_group_positions[8] = 229;    // 0 to 229, joint_wrist_yaw
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to return to the initial position, plan to the new joint space goal, and visualize the plan
  joint_group_positions[0] = 0;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0.595;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  joint_group_positions[6] = 0;
  joint_group_positions[7] = 0;
  joint_group_positions[8] = 0;
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  // ########################### Step 6 ##########################

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Add_Object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Now, let's define a collision object ROS message for the robot to avoid
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_base_arm.getPlanningFrame();

  // The id of the object is used to identify it
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.025;
  primitive.dimensions[primitive.BOX_Y] = 0.2;
  primitive.dimensions[primitive.BOX_Z] = 0.2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to move the base back, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0.8;   // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  collision_object.operation = collision_object.REMOVE;
  collision_objects.push_back(collision_object);

  // Now, let's remove the collision object from the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Remove object from the world");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Remove_Object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  planning_scene_interface.addCollisionObjects(collision_objects);

  current_state = move_group_base_arm.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to return the base to initial position, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0;      // 0 to inf, position/x
  joint_group_positions[1] = 0;      // 0 to inf, position/y
  joint_group_positions[2] = 0;      // -2 to 4, position/theta
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group_base_arm.move();

  // ########################### Step 7 ##########################

  joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Getting Basic Information
  // We can print the name of the reference frame for this robot
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // We'll save the current pose to use it as a reference point so that we don't have to
  // enter the pose values that must remain constant manually
  geometry_msgs::msg::PoseStamped currentPose;
  currentPose = move_group_arm.getCurrentPose();

  move_group_arm.setStartStateToCurrentState();

  // We create the pose goal
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = currentPose.pose.orientation.x;
  target_pose1.orientation.y = currentPose.pose.orientation.y;
  target_pose1.orientation.z = currentPose.pose.orientation.z;
  target_pose1.orientation.w = currentPose.pose.orientation.w;
  target_pose1.position.x = currentPose.pose.position.x;
  target_pose1.position.y = currentPose.pose.position.y;
  target_pose1.position.z = 1.25;

  // We use the approximate IK solver to get the joint positions
  move_group_arm.setApproximateJointValueTarget(target_pose1, "link_wrist_yaw");

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group_arm.move();

  // ########################### Step 8 ##########################

  joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);
  ee_parent_link = move_group_base_arm.getCurrentState()->getLinkModel("link_grasp_center"); // the link name can be changed here to visualize the trajectory of the corresponding link

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Visualize_Trajectory", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  current_state = move_group_base_arm.getCurrentState(10);

  joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify the joints to stow the arm, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0.5;
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0.2;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 0;
  joint_group_positions[6] = 0;
  joint_group_positions[7] = 0;
  joint_group_positions[8] = 229;
  move_group_base_arm.setJointValueTarget(joint_group_positions);

  success = (move_group_base_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, ee_parent_link, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group_base_arm.move();

  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
