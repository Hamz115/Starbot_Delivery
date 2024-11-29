#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <tuple>
#include <vector>

class RobotController : public rclcpp::Node {
public:
  RobotController() : Node("robot_controller"), positions_received_(0) {
    move_group_node_ = std::make_shared<rclcpp::Node>(
        "move_group_interface_tutorial",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    executor_.add_node(move_group_node_);
    spinner_thread_ = std::thread([this]() { this->executor_.spin(); });

    setupMoveGroups(); // Modified to setup both groups
    setupPlanningScene();
    setupSubscriber();

    RCLCPP_INFO(this->get_logger(), "Robot Controller initialized");
  }

  ~RobotController() {
    if (spinner_thread_.joinable()) {
      executor_.cancel();
      spinner_thread_.join();
    }
  }

private:
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_thread_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      gripper_group_; // Added gripper group
  std::vector<std::tuple<double, double, double>> hole_positions_;
  int positions_received_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hole_subscriber_;

  void setupMoveGroups() {
    RCLCPP_INFO(this->get_logger(), "Setting up move group interfaces...");

    // Setup arm group
    arm_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "ur_manipulator");

    arm_group_->setMaxVelocityScalingFactor(0.1);
    arm_group_->setMaxAccelerationScalingFactor(0.1);
    arm_group_->setPlanningTime(20.0);
    arm_group_->setNumPlanningAttempts(10);
    arm_group_->allowReplanning(true);

    // Setup gripper group
    gripper_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "gripper");

    gripper_group_->setMaxVelocityScalingFactor(0.1);
    gripper_group_->setMaxAccelerationScalingFactor(0.1);
    gripper_group_->setPlanningTime(20.0);
    gripper_group_->setNumPlanningAttempts(10);

    RCLCPP_INFO(this->get_logger(), "Move group interfaces setup complete");
  }

  void setupPlanningScene() {
    RCLCPP_INFO(this->get_logger(), "Setting up planning scene...");

    std::vector<std::string> object_ids = {"wall", "table", "machine"};
    planning_scene_interface_.removeCollisionObjects(object_ids);
    rclcpp::sleep_for(std::chrono::seconds(1));

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    auto wall = createCollisionObject("wall", "base_link",
                                      createSolidPrimitiveBOX(2.0, 0.1, 2.0),
                                      createPose(-0.25, -0.4, 0, 1.0));

    auto table = createCollisionObject("table", "base_link",
                                       createSolidPrimitiveBOX(0.85, 1.8, 1.0),
                                       createPose(0.3, 0.35, -0.501, 1.0));

    auto machine = createCollisionObject(
        "machine", "base_link", createSolidPrimitiveBOX(0.6, 0.15, 0.4),
        createPose(0.2, 0.85, 0.2, 1.0));

    collision_objects.push_back(wall);
    collision_objects.push_back(table);
    collision_objects.push_back(machine);

    planning_scene_interface_.addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
  }

  void setupSubscriber() {
    hole_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/holes_detected", 10,
        std::bind(&RobotController::holeCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscriber setup complete");
  }

  void holeCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (positions_received_ < 4) {
      hole_positions_.push_back(std::make_tuple(msg->x, msg->y, msg->z));
      positions_received_++;

      RCLCPP_INFO(this->get_logger(),
                  "Received hole position %d: x=%f, y=%f, z=%f",
                  positions_received_, msg->x, msg->y, msg->z);

      if (positions_received_ == 2) {
        executeMovementSequence(msg->x, msg->y, msg->z);
      }

      if (positions_received_ == 4) {
        RCLCPP_INFO(this->get_logger(), "Received all 4 hole positions");
        hole_subscriber_.reset();
      }
    }
  }

  void executeMovementSequence(double hole_x, double hole_y, double hole_z) {
    // // 1. Move to home position
    // cmd_arm("home");
    // rclcpp::sleep_for(std::chrono::seconds(2));

    // // 2. Close the gripper using gripper group
    // cmd_gripper("gripper_close");
    // rclcpp::sleep_for(std::chrono::seconds(2));

    // 3. Move to pre-grasp position
    cmd_arm("pre_grasp3");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 4. Move to position with offset
    move_to_position_with_orientation(hole_x + 0.0017, // X with offset
                                      hole_y,          // Y unchanged
                                      hole_z + 0.37,   // Z with offset
                                      true // Use orientation constraints
    );
    move_to_position_with_orientation(hole_x, hole_y, hole_z + 0.09, true);
  }

  void cmd_gripper(const std::string &target_name) {
    RCLCPP_INFO(this->get_logger(), "Moving gripper to named target: %s",
                target_name.c_str());

    gripper_group_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (gripper_group_->plan(my_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing gripper move to %s",
                  target_name.c_str());
      gripper_group_->execute(my_plan);
      rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Gripper planning failed for %s",
                   target_name.c_str());
    }
  }

  void move_to_position_with_orientation(double x, double y, double z,
                                         bool use_constraints) {
    RCLCPP_INFO(this->get_logger(), "Moving to position: x=%f, y=%f, z=%f", x,
                y, z);

    geometry_msgs::msg::Pose current_pose = arm_group_->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    if (use_constraints) {
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = "hand_ee";
      ocm.header.frame_id = "base_link";
      ocm.orientation = current_pose.orientation;
      ocm.absolute_x_axis_tolerance = 0.1;
      ocm.absolute_y_axis_tolerance = 0.1;
      ocm.absolute_z_axis_tolerance = 0.1;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints constraints;
      constraints.orientation_constraints.push_back(ocm);
      arm_group_->setPathConstraints(constraints);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = arm_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory,
        use_constraints ? arm_group_->getPathConstraints()
                        : moveit_msgs::msg::Constraints());

    if (fraction > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Path computed successfully (%.2f%%)",
                  fraction * 100.0);
      bool success = static_cast<bool>(arm_group_->execute(trajectory));
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Motion executed successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Path computation failed!");
    }

    if (use_constraints) {
      arm_group_->clearPathConstraints();
    }
  }

  void cmd_arm(const std::string &target_name) {
    RCLCPP_INFO(this->get_logger(), "Moving arm to named target: %s",
                target_name.c_str());

    arm_group_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success =
        (arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing move to %s",
                  target_name.c_str());
      arm_group_->execute(my_plan);
      rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed for %s",
                   target_name.c_str());
    }
  }

  // Utility functions for collision objects
  moveit_msgs::msg::CollisionObject
  createCollisionObject(const std::string &id, const std::string &frame_id,
                        const shape_msgs::msg::SolidPrimitive &primitive,
                        const geometry_msgs::msg::Pose &pose) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = id;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    return object;
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}