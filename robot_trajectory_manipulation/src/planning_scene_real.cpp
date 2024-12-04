#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>

class PlanningSceneSetup : public rclcpp::Node {
public:
  PlanningSceneSetup() : Node("planning_scene_setup") {
    RCLCPP_INFO(this->get_logger(), "Setting up planning scene...");

    // Remove any existing objects
    removeCollisionObjects();

    // Add custom objects to the planning scene
    setupCollisionObjects();

    RCLCPP_INFO(this->get_logger(), "Planning scene setup complete.");
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  void removeCollisionObjects() {
    std::vector<std::string> object_ids = {"barrier", "platform", "equipment"};
    planning_scene_interface_.removeCollisionObjects(object_ids);
    rclcpp::sleep_for(
        std::chrono::seconds(1)); // Allow time for changes to take effect
    RCLCPP_INFO(this->get_logger(), "Removed existing collision objects.");
  }

  void setupCollisionObjects() {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Define "barrier" object
    collision_objects.push_back(createCollisionObject(
        "barrier", "base_link", createSolidPrimitiveBOX(2.0, 0.1, 2.0),
        createPose(-0.25, -0.4, 0, 1.0)));

    // Define "platform" object
    collision_objects.push_back(createCollisionObject(
        "platform", "base_link", createSolidPrimitiveBOX(0.85, 1.8, 1.0),
        createPose(0.3, 0.35, -0.501, 1.0)));

    // Define "equipment" object
    collision_objects.push_back(createCollisionObject(
        "equipment", "base_link", createSolidPrimitiveBOX(0.6, 0.15, 0.4),
        createPose(0.2, 0.85, 0.2, 1.0)));

    // Add the objects to the planning scene
    planning_scene_interface_.addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Added collision objects to the scene.");
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
  auto node = std::make_shared<PlanningSceneSetup>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}