#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_real");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

// Joint positions for different poses
static const std::vector<double> HOME_POSITION = {
    1.5000246206866663,    // shoulder_pan_joint
    -2.4999686680235804,   // shoulder_lift_joint
    1.3929963111877441,    // elbow_joint
    -1.5000360322049637,   // wrist_1_joint
    -1.5500105063067835,   // wrist_2_joint
    -0.0001214186297815445 // wrist_3_joint
};

static const std::vector<double> PRE_GRASP_POSITION = {
    0.673963725566864,   // shoulder_pan_joint
    -1.3480626356652756, // shoulder_lift_joint
    1.2383340040790003,  // elbow_joint
    -1.5000792902759095, // wrist_1_joint
    -1.5499675909625452, // wrist_2_joint
    -6.2290822164357e-05 // wrist_3_joint
};

static const std::vector<double> PRE_GRASP2_POSITION = {
    0.6692178249359131,   // shoulder_pan_joint
    -1.2411320370486756,  // shoulder_lift_joint
    0.7037423292743128,   // elbow_joint
    -1.072423742418625,   // wrist_1_joint
    -1.5498469511615198,  // wrist_2_joint
    -0.004681889210836232 // wrist_3_joint
};

static const std::vector<double> PRE_GRASP3_POSITION = {
    1.5725510120391846,  // shoulder_pan_joint
    -1.2409565907767792, // shoulder_lift_joint
    0.7038052717791956,  // elbow_joint
    -1.072516308431961,  // wrist_1_joint
    -1.5497430006610315, // wrist_2_joint
    -0.00477010408510381 // wrist_3_joint
};

static const std::vector<double> GRASP_POSITION = {
    0.6453410387039185,   // shoulder_pan_joint
    -1.3270834249309083,  // shoulder_lift_joint
    1.4679835478412073,   // elbow_joint
    -1.750256200829977,   // wrist_1_joint
    -1.548866097127096,   // wrist_2_joint
    -0.028667751942769826 // wrist_3_joint
};

class PlanningScene {
public:
  PlanningScene(rclcpp::Node::SharedPtr node) : node_(node) {
    // Initialize node with options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node =
        rclcpp::Node::make_shared("planning_scene_interface", node_options);

    // Setup executor
    executor.add_node(move_group_node);
    std::thread([this]() { this->executor.spin(); }).detach();

    // Declare parameters for object dimensions and positions
    node_->declare_parameter<float>("x_dim", 0.1);
    node_->declare_parameter<float>("y_dim", 0.1);
    node_->declare_parameter<float>("z_dim", 0.1);
    node_->declare_parameter<float>("x_pos", 0.1);
    node_->declare_parameter<float>("y_pos", 0.1);
    node_->declare_parameter<float>("z_pos", 0.1);

    RCLCPP_INFO(LOGGER, "Planning Scene initialized successfully");
  }

  // Parameter getters
  float get_x_dim() { return node_->get_parameter("x_dim").get_value<float>(); }
  float get_y_dim() { return node_->get_parameter("y_dim").get_value<float>(); }
  float get_z_dim() { return node_->get_parameter("z_dim").get_value<float>(); }
  float get_x_pos() { return node_->get_parameter("x_pos").get_value<float>(); }
  float get_y_pos() { return node_->get_parameter("y_pos").get_value<float>(); }
  float get_z_pos() { return node_->get_parameter("z_pos").get_value<float>(); }

  // Create collision object
  moveit_msgs::msg::CollisionObject
  createCollisionObject(const std::string &id, const std::string &frame_id,
                        const shape_msgs::msg::SolidPrimitive &primitive,
                        const geometry_msgs::msg::Pose &pose) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    RCLCPP_INFO(LOGGER, "Created collision object: %s", id.c_str());
    return collision_object;
  }

  // Create box primitive
  shape_msgs::msg::SolidPrimitive createSolidPrimitiveBOX(double x, double y,
                                                          double z) {
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {x, y, z};
    return box;
  }

  // Create pose
  geometry_msgs::msg::Pose createPose(double x, double y, double z, double w) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = w;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
  }

  // Setup initial scene
  void setupInitialScene() {
    // Create the wall
    auto wall_object = createCollisionObject(
        "wall", "base_link", createSolidPrimitiveBOX(2.0, 0.1, 2.0),
        createPose(-0.25, -0.4, 0, 1.0));

    // Create the table
    auto table_object = createCollisionObject(
        "table", "base_link", createSolidPrimitiveBOX(0.85, 1.8, 1.0),
        createPose(0.3, 0.35, -0.501, 1.0));

    // Create the machine
    auto machine_object = createCollisionObject(
        "machine", "base_link", createSolidPrimitiveBOX(0.6, 0.15, 0.4),
        createPose(0.2, 0.85, 0.2, 1.0));

    // Add objects to the scene
    collision_objects_ = {wall_object, table_object, machine_object};
    planning_scene_interface_.addCollisionObjects(collision_objects_);

    RCLCPP_INFO(LOGGER, "Initial scene setup complete");
  }

  bool moveToNamedTarget(const std::vector<double> &target_position,
                         const std::string &position_name) {
    auto move_group_arm =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node, PLANNING_GROUP_ARM);

    // Set very slow speed for safety
    move_group_arm->setMaxVelocityScalingFactor(0.1); // 10% of max speed
    move_group_arm->setMaxAccelerationScalingFactor(0.1);
    move_group_arm->setGoalJointTolerance(0.001);
    move_group_arm->setNumPlanningAttempts(10);
    move_group_arm->setPlanningTime(5.0);

    // Get the current pose
    geometry_msgs::msg::PoseStamped current_pose =
        move_group_arm->getCurrentPose();

    // Set the target joint values and get the target pose
    move_group_arm->setJointValueTarget(target_position);
    geometry_msgs::msg::PoseStamped target_pose =
        move_group_arm->getPoseTarget();

    // Create waypoints for Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    waypoints.push_back(target_pose.pose);

    // Plan Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // 1cm resolution
    double fraction = move_group_arm->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) { // If we can achieve at least 90% of the path
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;

      RCLCPP_INFO(LOGGER, "Executing Cartesian path to %s (%.2f%% achieved)",
                  position_name.c_str(), fraction * 100.0);

      auto execution_result = move_group_arm->execute(cartesian_plan);

      if (execution_result ==
          moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Successfully reached %s position",
                    position_name.c_str());
        return true;
      }
    }

    // Fallback to joint space planning if Cartesian planning fails
    RCLCPP_WARN(LOGGER, "Falling back to joint space planning for %s",
                position_name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    bool success = (move_group_arm->plan(joint_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      auto execution_result = move_group_arm->execute(joint_plan);
      if (execution_result ==
          moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Successfully reached %s position",
                    position_name.c_str());
        return true;
      }
    }

    RCLCPP_ERROR(LOGGER, "Failed to plan/execute movement to %s position",
                 position_name.c_str());
    return false;
  }

  bool controlGripper(bool open) {
    auto move_group_gripper =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node, PLANNING_GROUP_GRIPPER);

    std::vector<std::string> joint_names = move_group_gripper->getJointNames();

    std::string command = open ? "gripper_open" : "gripper_close";
    double target_position = open ? 0.4 : -0.1;

    RCLCPP_INFO(LOGGER, "Executing %s command", command.c_str());

    move_group_gripper->setMaxVelocityScalingFactor(0.5);
    move_group_gripper->setMaxAccelerationScalingFactor(0.5);
    move_group_gripper->setGoalJointTolerance(0.001);
    move_group_gripper->setJointValueTarget(joint_names[0], target_position);

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = (move_group_gripper->plan(gripper_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      auto execution_result = move_group_gripper->execute(gripper_plan);
      if (execution_result ==
          moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "%s completed successfully", command.c_str());
        return true;
      }
    }

    RCLCPP_ERROR(LOGGER, "Failed to execute %s command", command.c_str());
    return false;
  }

  bool executePickSequence() {
    RCLCPP_INFO(LOGGER, "Starting pick sequence");

    // 1. Move to home position
    if (!moveToNamedTarget(HOME_POSITION, "home")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach home position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 2. Open gripper
    if (!controlGripper(true)) {
      RCLCPP_ERROR(LOGGER, "Failed to open gripper");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 3. Move to pre-grasp2
    if (!moveToNamedTarget(PRE_GRASP2_POSITION, "pre_grasp2")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach pre-grasp2 position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 4. Move to pre-grasp
    if (!moveToNamedTarget(PRE_GRASP_POSITION, "pre_grasp")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach pre-grasp position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 5. Move to grasp
    if (!moveToNamedTarget(GRASP_POSITION, "grasp")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach grasp position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 6. Close gripper
    if (!controlGripper(false)) {
      RCLCPP_ERROR(LOGGER, "Failed to close gripper");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 7. Move back to pre-grasp
    if (!moveToNamedTarget(PRE_GRASP_POSITION, "pre_grasp")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach pre-grasp position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 8. Move to pre-grasp2
    if (!moveToNamedTarget(PRE_GRASP2_POSITION, "pre_grasp2")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach pre-grasp2 position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 9. Move to pre-grasp3
    if (!moveToNamedTarget(PRE_GRASP3_POSITION, "pre_grasp3")) {
      RCLCPP_ERROR(LOGGER, "Failed to reach pre-grasp3 position");
      return false;
    }

    RCLCPP_INFO(LOGGER, "Pick sequence completed successfully");
    return true;
  }

  void removeCollisionObjects(
      const std::vector<moveit_msgs::msg::CollisionObject> &collision_objects) {
    std::vector<std::string> object_ids;
    for (const auto &object : collision_objects) {
      object_ids.push_back(object.id);
    }
    planning_scene_interface_.removeCollisionObjects(object_ids);
    RCLCPP_INFO(LOGGER, "Removed collision objects from scene");
  }

private:
  rclcpp::Node::SharedPtr move_group_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("planning_scene_node");
  auto planning_scene = std::make_shared<PlanningScene>(node);

  planning_scene->setupInitialScene();
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(LOGGER, "Planning scene is ready for the real robot");

  if (!planning_scene->executePickSequence()) {
    RCLCPP_ERROR(LOGGER, "Failed to complete pick sequence");
  } else {
    RCLCPP_INFO(LOGGER, "Pick sequence completed successfully");
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }

  return 0;
}