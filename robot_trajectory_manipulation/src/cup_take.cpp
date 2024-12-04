#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <tuple>
#include <vector>

class ManipulatorSystem : public rclcpp::Node {
public:
  ManipulatorSystem() : Node("manipulator_system"), detected_points_(0) {
    movement_node_ = std::make_shared<rclcpp::Node>(
        "movement_interface_system",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    task_executor_.add_node(movement_node_);
    execution_thread_ = std::thread([this]() { this->task_executor_.spin(); });

    initializeMovementGroups();
    setupEnvironment();
    initializeDetector();

    RCLCPP_INFO(this->get_logger(), "Manipulator System initialized");
  }

  ~ManipulatorSystem() {
    if (execution_thread_.joinable()) {
      task_executor_.cancel();
      execution_thread_.join();
    }
  }

private:
  rclcpp::Node::SharedPtr movement_node_;
  rclcpp::executors::SingleThreadedExecutor task_executor_;
  std::thread execution_thread_;
  moveit::planning_interface::PlanningSceneInterface environment_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      manipulator_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      end_effector_group_;
  std::vector<std::tuple<double, double, double>> target_positions_;
  int detected_points_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_detector_;

  void initializeMovementGroups() {
    RCLCPP_INFO(this->get_logger(), "Initializing movement groups...");

    manipulator_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            movement_node_, "ur_manipulator");

    manipulator_group_->setMaxVelocityScalingFactor(0.05);
    manipulator_group_->setMaxAccelerationScalingFactor(0.05);
    manipulator_group_->setPlanningTime(20.0);
    manipulator_group_->setNumPlanningAttempts(10);
    manipulator_group_->allowReplanning(true);

    end_effector_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            movement_node_, "gripper");

    end_effector_group_->setMaxVelocityScalingFactor(0.05);
    end_effector_group_->setMaxAccelerationScalingFactor(0.05);
    end_effector_group_->setPlanningTime(20.0);
    end_effector_group_->setNumPlanningAttempts(10);

    RCLCPP_INFO(this->get_logger(), "Movement groups initialization complete");
  }

  void setupEnvironment() {
    RCLCPP_INFO(this->get_logger(), "Setting up environment...");

    std::vector<std::string> object_ids = {"barrier", "platform", "equipment"};
    environment_interface_.removeCollisionObjects(object_ids);
    rclcpp::sleep_for(std::chrono::seconds(1));

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    auto barrier = createCollisionObject("barrier", "base_link",
                                         createSolidPrimitiveBOX(2.0, 0.1, 2.0),
                                         createPose(-0.25, -0.4, 0, 1.0));

    auto platform = createCollisionObject(
        "platform", "base_link", createSolidPrimitiveBOX(0.85, 1.8, 1.0),
        createPose(0.3, 0.35, -0.501, 1.0));

    auto equipment = createCollisionObject(
        "equipment", "base_link", createSolidPrimitiveBOX(0.6, 0.15, 0.4),
        createPose(0.2, 0.85, 0.2, 1.0));

    auto cup = createCollisionObject("portable_cup_2", "base_link",
                                     createSolidPrimitiveBOX(0.04, 0.05, 0.04),
                                     createPose(14.16, -18.19, 1.025, 1.0));

    collision_objects.push_back(barrier);
    collision_objects.push_back(platform);
    collision_objects.push_back(equipment);
    collision_objects.push_back(cup);

    environment_interface_.addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Environment setup complete");
  }
  void initializeDetector() {
    point_detector_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/holes_detected", 10,
        std::bind(&ManipulatorSystem::pointDetectionCallback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Point detector initialized");
  }

  void pointDetectionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (detected_points_ < 4) {
      target_positions_.push_back(std::make_tuple(msg->x, msg->y, msg->z));
      detected_points_++;

      RCLCPP_INFO(this->get_logger(), "Detected point %d: x=%f, y=%f, z=%f",
                  detected_points_, msg->x, msg->y, msg->z);

      if (detected_points_ == 2) {
        executeTaskSequence(msg->x, msg->y, msg->z);
      }

      if (detected_points_ == 4) {
        RCLCPP_INFO(this->get_logger(), "All 4 points detected");
        point_detector_.reset();
      }
    }
  }

  void executeTaskSequence(double target_x, double target_y, double target_z) {

    control_gripper("gripper_open");
    rclcpp::sleep_for(std::chrono::seconds(2));

    execute_arm("pre_grasp3");
    rclcpp::sleep_for(std::chrono::seconds(2));

    execute_arm("after_grasp");
    rclcpp::sleep_for(std::chrono::seconds(2));

    navigate_to_position(target_x - 0.05, target_y + 0.02, 0.39, true);
    rclcpp::sleep_for(std::chrono::seconds(2));

    navigate_to_position(target_x - 0.05, target_y + 0.02, target_z + 0.50,
                         true);
    rclcpp::sleep_for(std::chrono::seconds(2));

    navigate_to_position(target_x - 0.05, target_y + 0.02, target_z + 0.40,
                         true);
    rclcpp::sleep_for(std::chrono::seconds(2));

    navigate_to_position(target_x - 0.05, target_y + 0.02, target_z + 0.27,
                         true);
    rclcpp::sleep_for(std::chrono::seconds(2));

    control_gripper("gripper_close");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Return to previous position after opening gripper
    navigate_to_position(target_x, target_y, 0.37, true);

    execute_arm("after_grasp");
    rclcpp::sleep_for(std::chrono::seconds(2));

    execute_arm("pre_grasp3");
    rclcpp::sleep_for(std::chrono::seconds(2));

  }

  void control_gripper(const std::string &target_name) {
    RCLCPP_INFO(this->get_logger(), "Controlling end effector: %s",
                target_name.c_str());

    end_effector_group_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;

    bool success = (end_effector_group_->plan(movement_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing end effector movement to %s",
                  target_name.c_str());
      end_effector_group_->execute(movement_plan);
      rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(this->get_logger(), "End effector planning failed for %s",
                   target_name.c_str());
    }
  }

  void navigate_to_position(double x, double y, double z,
                            bool use_constraints) {
    RCLCPP_INFO(this->get_logger(), "Navigating to position: x=%f, y=%f, z=%f",
                x, y, z);

    geometry_msgs::msg::Pose current_pose =
        manipulator_group_->getCurrentPose().pose;
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
      manipulator_group_->setPathConstraints(constraints);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory moveit_trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;

    double fraction = manipulator_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, moveit_trajectory,
        use_constraints ? manipulator_group_->getPathConstraints()
                        : moveit_msgs::msg::Constraints());

    if (fraction > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Navigation path computed (%.2f%%)",
                  fraction * 100.0);

      // Convert moveit_msgs::msg::RobotTrajectory to
      // robot_trajectory::RobotTrajectory
      robot_trajectory::RobotTrajectory robot_trajectory(
          manipulator_group_->getRobotModel(), manipulator_group_->getName());
      robot_trajectory.setRobotTrajectoryMsg(
          *manipulator_group_->getCurrentState(), moveit_trajectory);

      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      if (time_param.computeTimeStamps(robot_trajectory)) {
        RCLCPP_INFO(this->get_logger(), "Time parameterization successful");

        // Convert back to moveit_msgs::msg::RobotTrajectory for execution
        robot_trajectory.getRobotTrajectoryMsg(moveit_trajectory);
        bool success =
            static_cast<bool>(manipulator_group_->execute(moveit_trajectory));
        if (success) {
          RCLCPP_INFO(this->get_logger(), "Navigation executed successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Navigation execution failed");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Time parameterization failed");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation path computation failed!");
    }

    if (use_constraints) {
      manipulator_group_->clearPathConstraints();
    }
  }

  void execute_arm(const std::string &target_name) {
    RCLCPP_INFO(this->get_logger(), "Executing arm movement to: %s",
                target_name.c_str());

    manipulator_group_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;

    bool success = (manipulator_group_->plan(movement_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing movement to %s",
                  target_name.c_str());
      manipulator_group_->execute(movement_plan);
      rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Movement planning failed for %s",
                   target_name.c_str());
    }
  }

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
  auto node = std::make_shared<ManipulatorSystem>();
  rclcpp::spin(node);
  return 0;
}