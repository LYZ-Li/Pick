#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <ur5e_pick_place_interfaces/srv/pick_place.hpp>

namespace mtc = moveit::task_constructor;

class PickPlaceServer : public rclcpp::Node {
public:
  PickPlaceServer(const rclcpp::NodeOptions& options)
    : Node("mtc_pick_place_server", options)
  {
    declare_parameter("arm_group", "ur_manipulator");
    declare_parameter("gripper_group", "gripper");
    declare_parameter("gripper_open_joint", "robotiq_85_left_knuckle_joint");
    declare_parameter("gripper_frame", "tool0");
    declare_parameter("world_frame", "base_link");
    declare_parameter("max_solutions", 5);

    arm_group_ = get_parameter("arm_group").as_string();
    gripper_group_ = get_parameter("gripper_group").as_string();
    gripper_open_joint_ = get_parameter("gripper_open_joint").as_string();
    gripper_frame_ = get_parameter("gripper_frame").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    max_solutions_ = get_parameter("max_solutions").as_int();

    srv_ = create_service<ur5e_pick_place_interfaces::srv::PickPlace>(
      "/execute_pick_place",
      std::bind(&PickPlaceServer::handle_execute, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "MTC PickPlaceServer ready on /execute_pick_place");
  }

private:
  using PickPlaceSrv = ur5e_pick_place_interfaces::srv::PickPlace;

  rclcpp::Service<PickPlaceSrv>::SharedPtr srv_;
  std::string arm_group_;
  std::string gripper_group_;
  std::string gripper_open_joint_;
  std::string gripper_frame_;
  std::string world_frame_;
  int max_solutions_;

  void handle_execute(
    const std::shared_ptr<PickPlaceSrv::Request> req,
    std::shared_ptr<PickPlaceSrv::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Received pick-place request");

    // Add a collision object for the target so MTC can attach/detach it
    add_target_object(req->grasp_pose);

    try {
      auto task = create_task(req);
      task.init();

      if (!task.plan(max_solutions_)) {
        res->success = false;
        res->message = "MTC planning failed — no valid solution found";
        remove_target_object();
        return;
      }

      auto result = task.execute(*task.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        res->success = false;
        res->message = "MTC execution failed (error code " +
                       std::to_string(result.val) + ")";
        remove_target_object();
        return;
      }

      res->success = true;
      res->message = "Pick-place executed successfully";
    } catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Exception: ") + e.what();
      remove_target_object();
    }
  }

  mtc::Task create_task(const std::shared_ptr<PickPlaceSrv::Request>& req) {
    mtc::Task task;
    task.stages()->setName("ur5e_pick_place");
    task.loadRobotModel(shared_from_this());

    // Planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(
      shared_from_this());
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.1);
    cartesian_planner->setMaxAccelerationScalingFactor(0.1);
    cartesian_planner->setStepSize(0.005);

    auto joint_interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // ---- Stage 1: Current State ----
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));

    // ---- Stage 2: Open Gripper ----
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interpolation);
      stage->setGroup(gripper_group_);
      stage->setGoal(gripper_joint_state(req->gripper_open_width));
      task.add(std::move(stage));
    }

    // ---- Stage 3: Move to Pre-Grasp ----
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pre-grasp",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // ---- Grasp Serial Container ----
    {
      auto grasp_container = std::make_unique<mtc::SerialContainer>("grasp");
      grasp_container->properties().configureInitFrom(
        mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // ---- Stage 4: Cartesian Approach ----
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach");
        stage->setGroup(arm_group_);
        stage->setMinMaxDistance(0.01, req->pregrasp_approach_distance);

        geometry_msgs::msg::Vector3Stamped approach_direction;
        approach_direction.header.frame_id = gripper_frame_;
        approach_direction.vector.z = 1.0;  // approach along gripper Z
        stage->setDirection(approach_direction);
        grasp_container->insert(std::move(stage));
      }

      // ---- Stage 5: Generate Grasp Pose ----
      {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setPreGraspPose(gripper_robot_state(req->gripper_open_width));
        stage->setObject("target_object");
        stage->setAngleDelta(0);  // single grasp pose from request, no sweep
        stage->setMonitoredStage(task.stages()->findChild("current state"));

        // Wrap in IK
        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(stage));
        ik_wrapper->setMaxIKSolutions(8);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(gripper_frame_);
        ik_wrapper->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(
          mtc::Stage::INTERFACE, {"target_pose"});

        // Override with the requested grasp pose
        ik_wrapper->properties().set("target_pose", req->grasp_pose);
        grasp_container->insert(std::move(ik_wrapper));
      }

      // ---- Stage 6: Allow Collision (gripper <-> object) ----
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (gripper, object)");
        stage->allowCollisions(
          "target_object",
          task.getRobotModel()->getJointModelGroup(gripper_group_)
            ->getLinkModelNamesWithCollisionGeometry(),
          true);
        grasp_container->insert(std::move(stage));
      }

      // ---- Stage 7: Close Gripper ----
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "close gripper", joint_interpolation);
        stage->setGroup(gripper_group_);
        stage->setGoal(gripper_joint_state(req->gripper_close_width));
        grasp_container->insert(std::move(stage));
      }

      // ---- Stage 8: Attach Object ----
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "attach object");
        stage->attachObject("target_object", gripper_frame_);
        grasp_container->insert(std::move(stage));
      }

      // ---- Stage 9: Lift ----
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "lift object", cartesian_planner);
        stage->properties().set("marker_ns", "lift");
        stage->setGroup(arm_group_);
        stage->setMinMaxDistance(0.01, req->lift_distance);

        geometry_msgs::msg::Vector3Stamped lift_direction;
        lift_direction.header.frame_id = world_frame_;
        lift_direction.vector.z = 1.0;  // lift up in world frame
        stage->setDirection(lift_direction);
        grasp_container->insert(std::move(stage));
      }

      task.add(std::move(grasp_container));
    }

    // ---- Stage 10: Move to Place ----
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // ---- Place Serial Container ----
    {
      auto place_container = std::make_unique<mtc::SerialContainer>("place");
      place_container->properties().configureInitFrom(
        mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // ---- Stage 11: Lower to Place ----
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "lower object", cartesian_planner);
        stage->properties().set("marker_ns", "lower");
        stage->setGroup(arm_group_);
        stage->setMinMaxDistance(0.01, req->lift_distance);

        geometry_msgs::msg::Vector3Stamped lower_direction;
        lower_direction.header.frame_id = world_frame_;
        lower_direction.vector.z = -1.0;
        stage->setDirection(lower_direction);
        place_container->insert(std::move(stage));
      }

      // ---- Stage 12: Generate Place Pose ----
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setObject("target_object");

        stage->setPose(req->place_pose);
        stage->setMonitoredStage(task.stages()->findChild("current state"));

        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "place pose IK", std::move(stage));
        ik_wrapper->setMaxIKSolutions(8);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(gripper_frame_);
        ik_wrapper->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(
          mtc::Stage::INTERFACE, {"target_pose"});

        place_container->insert(std::move(ik_wrapper));
      }

      // ---- Stage 13: Open Gripper ----
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "release object", joint_interpolation);
        stage->setGroup(gripper_group_);
        stage->setGoal(gripper_joint_state(req->gripper_open_width));
        place_container->insert(std::move(stage));
      }

      // ---- Stage 14: Forbid Collision (gripper <-> object) ----
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (gripper, object)");
        stage->allowCollisions(
          "target_object",
          task.getRobotModel()->getJointModelGroup(gripper_group_)
            ->getLinkModelNamesWithCollisionGeometry(),
          false);
        place_container->insert(std::move(stage));
      }

      // ---- Stage 15: Detach Object ----
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "detach object");
        stage->detachObject("target_object", gripper_frame_);
        place_container->insert(std::move(stage));
      }

      // ---- Stage 16: Retreat ----
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "retreat", cartesian_planner);
        stage->properties().set("marker_ns", "retreat");
        stage->setGroup(arm_group_);
        stage->setMinMaxDistance(0.01, req->pregrasp_approach_distance);

        geometry_msgs::msg::Vector3Stamped retreat_direction;
        retreat_direction.header.frame_id = gripper_frame_;
        retreat_direction.vector.z = -1.0;  // retreat along negative gripper Z
        stage->setDirection(retreat_direction);
        place_container->insert(std::move(stage));
      }

      task.add(std::move(place_container));
    }

    // Set task properties
    task.setProperty("group", arm_group_);
    task.setProperty("eef", gripper_group_);
    task.setProperty("ik_frame", gripper_frame_);

    return task;
  }

  // Map gripper width (m) to joint value for Robotiq 2F-85.
  // Robotiq knuckle joint: 0.0 = fully open (85mm), ~0.8 = fully closed.
  std::map<std::string, double> gripper_joint_state(double width_m) {
    double ratio = 1.0 - std::clamp(width_m / 0.085, 0.0, 1.0);
    double joint_value = ratio * 0.8;
    return {{gripper_open_joint_, joint_value}};
  }

  moveit_msgs::msg::RobotState gripper_robot_state(double width_m) {
    const auto joint_state = gripper_joint_state(width_m);

    moveit_msgs::msg::RobotState robot_state_msg;
    for (const auto& [joint_name, joint_value] : joint_state) {
      robot_state_msg.joint_state.name.push_back(joint_name);
      robot_state_msg.joint_state.position.push_back(joint_value);
    }

    return robot_state_msg;
  }

  void add_target_object(const geometry_msgs::msg::PoseStamped& grasp_pose) {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject obj;
    obj.id = "target_object";
    obj.header = grasp_pose.header;
    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = {0.04, 0.04, 0.04};  // 4cm cube placeholder
    obj.primitive_poses.resize(1);
    obj.primitive_poses[0] = grasp_pose.pose;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObject(obj);
  }

  void remove_target_object() {
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.removeCollisionObjects({"target_object"});
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PickPlaceServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
