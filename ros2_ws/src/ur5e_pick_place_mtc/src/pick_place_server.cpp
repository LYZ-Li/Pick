#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

class PickPlaceServer : public rclcpp::Node {
public:
  PickPlaceServer() : Node("mtc_pick_place_server") {
    srv_ = create_service<std_srvs::srv::Trigger>(
      "/execute_pick_place",
      std::bind(&PickPlaceServer::handle_execute, this,
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  void handle_execute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    try {
      moveit::task_constructor::Task task;
      task.stages()->setName("ur5e_pick_place");

      // TODO:
      // - load robot model
      // - create CurrentState stage
      // - add open gripper stage
      // - connect to pregrasp
      // - Cartesian approach
      // - close gripper
      // - attach object
      // - lift
      // - connect to place
      // - lower
      // - open gripper
      // - detach object
      // - retreat

      task.init();
      if (!task.plan(5)) {
        res->success = false;
        res->message = "MTC planning failed";
        return;
      }

      auto result = task.execute(*task.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        res->success = false;
        res->message = "MTC execution failed";
        return;
      }

      res->success = true;
      res->message = "Pick-place executed successfully";
    } catch (const std::exception& e) {
      res->success = false;
      res->message = e.what();
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}