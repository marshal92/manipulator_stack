#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manipulator_interfaces/action/manipulator_goal.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

class TacticalServerCpp : public rclcpp::Node {
public:
  using ManipulatorGoal = manipulator_interfaces::action::ManipulatorGoal;
  using GoalHandleManipulator = rclcpp_action::ServerGoalHandle<ManipulatorGoal>;
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GetCartesianPath = moveit_msgs::srv::GetCartesianPath;

  TacticalServerCpp() : Node("tactical_server") {
    move_client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action");
    cartesian_client_ = this->create_client<GetCartesianPath>("/compute_cartesian_path");
    execute_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/execute_trajectory");

    action_server_ = rclcpp_action::create_server<ManipulatorGoal>(
      this,
      "manipulator_action",
      std::bind(&TacticalServerCpp::handle_goal, this, _1, _2),
      std::bind(&TacticalServerCpp::handle_cancel, this, _1),
      std::bind(&TacticalServerCpp::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "C++ Tactical Server started.");
  }

private:
  rclcpp_action::Server<ManipulatorGoal>::SharedPtr action_server_;
  rclcpp_action::Client<MoveGroup>::SharedPtr move_client_;
  rclcpp::Client<GetCartesianPath>::SharedPtr cartesian_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_client_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ManipulatorGoal::Goal> goal)
  {
    if (goal->poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Pose array is empty!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleManipulator>)
  {
    RCLCPP_WARN(this->get_logger(), "Received cancel request for tactical action.");
    move_client_->async_cancel_all_goals();
    execute_client_->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
    std::thread{std::bind(&TacticalServerCpp::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleManipulator> goal_handle) {
    const auto req = goal_handle->get_goal();
    auto result = std::make_shared<ManipulatorGoal::Result>();

    double speed = (req->speed > 0.0) ? std::clamp(req->speed, 0.01, 1.0) : 0.1;
    double pos_tol = (req->pos_tolerance > 0.0) ? req->pos_tolerance : 0.03;
    double ori_tol = (req->ori_tolerance > 0.0) ? req->ori_tolerance : 0.15;

    RCLCPP_INFO(this->get_logger(),
      "Command: Mode=%s, Points=%zu, IgnoreOri=%s | Speed=%.2f, PosTol=%.3fm, OriTol=%.3frad",
      req->mode.c_str(), req->poses.size(), req->ignore_orientation ? "true" : "false",
      speed, pos_tol, ori_tol);

    // ==========================================
    // MODE 1: POSE (SINGLE TARGET MOTION)
    // ==========================================
    if (req->mode == "POSE") {
      if (!move_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "/move_action server is unavailable!");
        result->success = false;
        result->message = "/move_action server unavailable";
        goal_handle->abort(result);
        return;
      }

      MoveGroup::Goal goal_msg;
      moveit_msgs::msg::MotionPlanRequest mpr;
      mpr.group_name = "arm";
      mpr.num_planning_attempts = 10;
      mpr.allowed_planning_time = 5.0;
      mpr.max_velocity_scaling_factor = speed;
      mpr.max_acceleration_scaling_factor = speed;

      moveit_msgs::msg::Constraints constraints;

      moveit_msgs::msg::PositionConstraint pc;
      pc.header.frame_id = "arm_base_link";
      pc.link_name = "arm_tool0";
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions = {pos_tol};
      pc.constraint_region.primitives.push_back(primitive);
      pc.constraint_region.primitive_poses.push_back(req->poses[0]);
      pc.weight = 1.0;
      constraints.position_constraints.push_back(pc);

      if (!req->ignore_orientation) {
        moveit_msgs::msg::OrientationConstraint oc;
        oc.header.frame_id = "arm_base_link";
        oc.link_name = "arm_tool0";
        oc.orientation = req->poses[0].orientation;
        oc.absolute_x_axis_tolerance = ori_tol;
        oc.absolute_y_axis_tolerance = ori_tol;
        oc.absolute_z_axis_tolerance = ori_tol;
        oc.weight = 1.0;
        constraints.orientation_constraints.push_back(oc);
      }

      mpr.goal_constraints.push_back(constraints);
      goal_msg.request = mpr;
      goal_msg.planning_options.plan_only = false;
      goal_msg.planning_options.planning_scene_diff.is_diff = true;
      goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

      auto send_goal_future = move_client_->async_send_goal(goal_msg);
      if (send_goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        result->success = false;
        result->message = "Timeout sending goal to MoveIt";
        goal_handle->abort(result);
        return;
      }

      auto action_handle = send_goal_future.get();
      if (!action_handle) {
        result->success = false;
        result->message = "Goal rejected by MoveIt";
        goal_handle->abort(result);
        return;
      }

      auto res_future = move_client_->async_get_result(action_handle);
      if (res_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        result->success = false;
        result->message = "POSE motion execution timeout";
        goal_handle->abort(result);
        return;
      }

      auto wrapped_res = res_future.get();
      if (wrapped_res.result && wrapped_res.result->error_code.val == 1) {
        result->success = true;
        result->message = "Target reached";
        goal_handle->succeed(result);
      } else {
        result->success = false;
        result->message = "Path execution failed";
        goal_handle->abort(result);
      }
      return;
    }

    // ==========================================
    // MODE 2: CARTESIAN (LINEAR WAYPOINT PATH)
    // ==========================================
    if (req->mode == "CARTESIAN") {
      if (!cartesian_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "/compute_cartesian_path service is unavailable!");
        result->success = false;
        result->message = "/compute_cartesian_path service unavailable";
        goal_handle->abort(result);
        return;
      }

      auto req_cart = std::make_shared<GetCartesianPath::Request>();
      req_cart->header.frame_id = "arm_base_link";
      req_cart->group_name = "arm";
      req_cart->link_name = "arm_tool0";
      req_cart->waypoints = req->poses;
      req_cart->max_step = 0.01;
      req_cart->jump_threshold = 0.0;
      req_cart->avoid_collisions = true;
      req_cart->max_velocity_scaling_factor = speed;
      req_cart->max_acceleration_scaling_factor = speed;

      auto future_cart = cartesian_client_->async_send_request(req_cart);
      if (future_cart.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
        result->success = false;
        result->message = "Cartesian path computation timeout";
        goal_handle->abort(result);
        return;
      }

      auto res_cart = future_cart.get();
      if (res_cart->fraction < 0.9) {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path unreachable. Computed %.1f%%", res_cart->fraction * 100.0);
        result->success = false;
        result->message = "Cartesian path exceeds joint limits";
        goal_handle->abort(result);
        return;
      }

      if (!execute_client_->wait_for_action_server(std::chrono::seconds(5))) {
        result->success = false;
        result->message = "/execute_trajectory server unavailable";
        goal_handle->abort(result);
        return;
      }

      ExecuteTrajectory::Goal goal_exec;
      goal_exec.trajectory = res_cart->solution;

      auto future_exec = execute_client_->async_send_goal(goal_exec);
      if (future_exec.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        result->success = false;
        result->message = "Timeout sending trajectory for execution";
        goal_handle->abort(result);
        return;
      }

      auto action_exec_handle = future_exec.get();
      if (!action_exec_handle) {
        result->success = false;
        result->message = "Trajectory execution rejected";
        goal_handle->abort(result);
        return;
      }

      auto res_exec_future = execute_client_->async_get_result(action_exec_handle);
      if (res_exec_future.wait_for(std::chrono::seconds(90)) != std::future_status::ready) {
        result->success = false;
        result->message = "Cartesian trajectory execution timeout";
        goal_handle->abort(result);
        return;
      }

      auto wrapped_exec = res_exec_future.get();
      if (wrapped_exec.result && wrapped_exec.result->error_code.val == 1) {
        result->success = true;
        result->message = "Cartesian path completed";
        goal_handle->succeed(result);
      } else {
        result->success = false;
        result->message = "Cartesian execution error";
        goal_handle->abort(result);
      }
      return;
    }

    result->success = false;
    result->message = "Unknown execution mode";
    goal_handle->abort(result);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TacticalServerCpp>());
  rclcpp::shutdown();
  return 0;
}
