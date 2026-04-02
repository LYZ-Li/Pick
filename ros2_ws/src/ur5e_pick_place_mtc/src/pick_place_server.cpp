#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class PickPlaceServer : public rclcpp::Node
{
public:
  PickPlaceServer()
  : Node("hardcoded_pick_place_server")
  {
    declare_parameter("move_group_name", "ur_manipulator");
    declare_parameter("base_frame", "base");
    declare_parameter("approach_distance", 0.10);
    declare_parameter("lift_distance", 0.12);
    declare_parameter("home_joint_positions", std::vector<double>{0.0, -1.57, -1.57, -1.57, 1.57, 0.0});
    declare_parameter("pick_pose.position", std::vector<double>{0.45, 0.0, 0.20});
    declare_parameter("pick_pose.orientation_xyzw", std::vector<double>{0.0, 1.0, 0.0, 0.0});
    declare_parameter("place_pose.position", std::vector<double>{0.45, -0.25, 0.20});
    declare_parameter("place_pose.orientation_xyzw", std::vector<double>{0.0, 1.0, 0.0, 0.0});
    declare_parameter("gripper_action_name", "/gripper/robotiq_gripper_controller/gripper_cmd");
    declare_parameter("gripper_open_position", 0.0);
    declare_parameter("gripper_closed_position", 0.8);
    declare_parameter("gripper_max_effort", 50.0);

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, get_parameter("gripper_action_name").as_string());

    srv_ = create_service<std_srvs::srv::Trigger>(
      "/execute_pick_place",
      std::bind(&PickPlaceServer::handle_execute, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  using GripperCommand = control_msgs::action::GripperCommand;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  geometry_msgs::msg::Pose read_pose(const std::string & key) const
  {
    const auto position = get_parameter(key + ".position").as_double_array();
    const auto orientation = get_parameter(key + ".orientation_xyzw").as_double_array();

    geometry_msgs::msg::Pose pose;
    pose.position.x = position.at(0);
    pose.position.y = position.at(1);
    pose.position.z = position.at(2);
    pose.orientation.x = orientation.at(0);
    pose.orientation.y = orientation.at(1);
    pose.orientation.z = orientation.at(2);
    pose.orientation.w = orientation.at(3);
    return pose;
  }

  bool send_gripper_goal(double position)
  {
    if (!gripper_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(get_logger(), "Gripper action server not available");
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = get_parameter("gripper_max_effort").as_double();

    auto send_goal_future = gripper_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to send gripper goal");
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle || !goal_handle->is_active()) {
      RCLCPP_ERROR(get_logger(), "Gripper goal was rejected");
      return false;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed waiting for gripper result");
      return false;
    }

    return true;
  }

  bool move_to_joint_target(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::vector<double> & joint_values)
  {
    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group.plan(plan));
    if (!planned) {
      return false;
    }
    return static_cast<bool>(move_group.execute(plan));
  }

  bool move_to_pose_target(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & pose)
  {
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group.plan(plan));
    if (!planned) {
      move_group.clearPoseTargets();
      return false;
    }
    const bool executed = static_cast<bool>(move_group.execute(plan));
    move_group.clearPoseTargets();
    return executed;
  }

  void handle_execute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    try {
      auto move_group = moveit::planning_interface::MoveGroupInterface(
        shared_from_this(), get_parameter("move_group_name").as_string());
      move_group.setPlanningTime(10.0);
      move_group.setPoseReferenceFrame(get_parameter("base_frame").as_string());
      move_group.setMaxVelocityScalingFactor(0.15);
      move_group.setMaxAccelerationScalingFactor(0.15);

      auto pick_pose = read_pose("pick_pose");
      auto place_pose = read_pose("place_pose");
      auto approach_pose = pick_pose;
      auto lift_pose = pick_pose;
      approach_pose.position.z += get_parameter("approach_distance").as_double();
      lift_pose.position.z += get_parameter("lift_distance").as_double();

      if (!send_gripper_goal(get_parameter("gripper_open_position").as_double())) {
        throw std::runtime_error("Failed to open gripper");
      }
      if (!move_to_joint_target(move_group, get_parameter("home_joint_positions").as_double_array())) {
        throw std::runtime_error("Failed to move to home joints");
      }
      if (!move_to_pose_target(move_group, approach_pose)) {
        throw std::runtime_error("Failed to move to approach pose");
      }
      if (!move_to_pose_target(move_group, pick_pose)) {
        throw std::runtime_error("Failed to move to pick pose");
      }
      if (!send_gripper_goal(get_parameter("gripper_closed_position").as_double())) {
        throw std::runtime_error("Failed to close gripper");
      }
      if (!move_to_pose_target(move_group, lift_pose)) {
        throw std::runtime_error("Failed to lift object");
      }
      if (!move_to_pose_target(move_group, place_pose)) {
        throw std::runtime_error("Failed to move to place pose");
      }
      if (!send_gripper_goal(get_parameter("gripper_open_position").as_double())) {
        throw std::runtime_error("Failed to release object");
      }
      if (!move_to_joint_target(move_group, get_parameter("home_joint_positions").as_double_array())) {
        throw std::runtime_error("Failed to return home");
      }

      res->success = true;
      res->message = "Hardcoded pick-place sequence executed";
    } catch (const std::exception & e) {
      res->success = false;
      res->message = e.what();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
