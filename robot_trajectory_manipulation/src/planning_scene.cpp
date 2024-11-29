#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class PlanningScene {
public:
  explicit PlanningScene(rclcpp::Node::SharedPtr node) : node_(node) {
    // Initialize node with options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node =
        rclcpp::Node::make_shared("planning_scene_interface", node_options);

    // Setup executor
    executor.add_node(move_group_node);
    std::thread([this]() { this->executor.spin(); }).detach();

    // Declare parameters
    node_->declare_parameter<bool>("is_robot_sim", false);
    node_->declare_parameter<float>("x_dim", 0.1);
    node_->declare_parameter<float>("y_dim", 0.1);
    node_->declare_parameter<float>("z_dim", 0.1);
    node_->declare_parameter<float>("x_pos", 0.1);
    node_->declare_parameter<float>("y_pos", 0.1);
    node_->declare_parameter<float>("z_pos", 0.1);

    RCLCPP_INFO(rclcpp::get_logger("planning_scene"),
                "Planning Scene initialized successfully");
  }

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
    return collision_object;
  }

  shape_msgs::msg::SolidPrimitive createSolidPrimitiveBOX(double x, double y,
                                                          double z) {
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {x, y, z};
    return box;
  }

  geometry_msgs::msg::Pose createPose(double x, double y, double z, double w) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = w;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
  }

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

    RCLCPP_INFO(rclcpp::get_logger("planning_scene"),
                "Initial scene setup complete");

    // Wait for the planning scene to update
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  void removeCollisionObjects() {
    std::vector<std::string> object_ids;
    for (const auto &object : collision_objects_) {
      object_ids.push_back(object.id);
    }
    planning_scene_interface_.removeCollisionObjects(object_ids);
    RCLCPP_INFO(rclcpp::get_logger("planning_scene"),
                "Removed collision objects from scene");
  }

private:
  rclcpp::Node::SharedPtr move_group_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;
};

class RobotMotionController {
public:
  explicit RobotMotionController(rclcpp::Node::SharedPtr node)
      : node_(node), planning_scene_(node) {

    // Initialize move group node
    move_group_node_ = std::make_shared<rclcpp::Node>(
        "move_group_interface",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // Setup executor and spin in separate thread
    executor_.add_node(move_group_node_);
    spinner_thread_ = std::thread([this]() { this->executor_.spin(); });

    // Initialize move groups
    arm_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "ur_manipulator");
    gripper_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "gripper");

    // Set movement parameters
    arm_group_->setMaxVelocityScalingFactor(
        0.1); // Reduced from 0.3 to 0.1 (10% of max speed)
    arm_group_->setMaxAccelerationScalingFactor(
        0.1); // Reduced from 0.3 to 0.1 (10% of max acceleration)
    arm_group_->setPlanningTime(5.0);
    arm_group_->setNumPlanningAttempts(10);
    arm_group_->setPlannerId("RRTConnect");

    RCLCPP_INFO(rclcpp::get_logger("robot_controller"),
                "Robot controller initialized");
  }

  ~RobotMotionController() {
    if (spinner_thread_.joinable()) {
      executor_.cancel();
      spinner_thread_.join();
    }
  }

  bool moveToNamedTarget(const std::string &target_name) {
    RCLCPP_INFO(rclcpp::get_logger("robot_controller"),
                "Planning movement to: %s", target_name.c_str());

    arm_group_->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool success = static_cast<bool>(arm_group_->plan(motion_plan));

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_controller"),
                   "Failed to plan movement to %s", target_name.c_str());
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("robot_controller"),
                "Executing movement to: %s", target_name.c_str());
    success = static_cast<bool>(arm_group_->execute(motion_plan));

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_controller"),
                   "Failed to execute movement to %s", target_name.c_str());
      return false;
    }

    return true;
  }

  bool controlGripper(const std::string &gripper_state) {
    RCLCPP_INFO(rclcpp::get_logger("robot_controller"),
                "Setting gripper to: %s", gripper_state.c_str());

    gripper_group_->setNamedTarget(gripper_state);

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = static_cast<bool>(gripper_group_->plan(gripper_plan));

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_controller"),
                   "Failed to plan gripper movement");
      return false;
    }

    success = static_cast<bool>(gripper_group_->execute(gripper_plan));

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_controller"),
                   "Failed to execute gripper movement");
      return false;
    }

    return true;
  }

  bool executeMovementSequence() {
    // Setup the planning scene first
    planning_scene_.setupInitialScene();
    rclcpp::sleep_for(
        std::chrono::milliseconds(1000)); // Wait for scene to update

    // Define the sequence of movements and gripper actions
    const std::vector<std::pair<std::string, std::string>> sequence = {
        {"move", "home"},       {"gripper", "gripper_open"},
        {"move", "pre_grasp2"}, {"move", "pre_grasp"},
        {"move", "grasp"},      {"gripper", "gripper_close"},
        {"move", "pre_grasp"},  {"move", "pre_grasp2"},
        {"move", "pre_grasp3"}};

    // Execute each movement in sequence
    for (const auto &action : sequence) {
      bool success = false;

      if (action.first == "move") {
        success = moveToNamedTarget(action.second);
      } else if (action.first == "gripper") {
        success = controlGripper(action.second);
      }

      if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_controller"),
                     "Failed to execute %s: %s", action.first.c_str(),
                     action.second.c_str());
        return false;
      }

      // Add delay between movements for safety
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(rclcpp::get_logger("robot_controller"),
                "Movement sequence completed successfully");
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_thread_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      gripper_group_;

  PlanningScene planning_scene_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("robot_motion_control_node");

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Initializing robot motion controller...");

  auto robot_controller = std::make_shared<RobotMotionController>(node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting movement sequence...");

  if (!robot_controller->executeMovementSequence()) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Movement sequence failed!");
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Movement sequence completed successfully");

  rclcpp::shutdown();
  return 0;
}