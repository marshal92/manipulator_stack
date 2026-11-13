#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;

class TrajectoryBridge : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  TrajectoryBridge()
  : Node("trajectory_bridge")
  {
    joint_map_ = {
      {"arm_joint_0", 0.0f},
      {"arm_joint_1", 1.0f},
      {"arm_joint_2", 2.0f},
      {"arm_joint_3", 3.0f},
      {"arm_joint_4", 4.0f},
      {"arm_joint_5", 5.0f},
      {"arm_clamp",   6.0f}
    };

    servo_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/arm/servo_cmd", 20);
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_states", 10);

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/arm/joint_states_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryBridge::joint_states_cb, this, std::placeholders::_1));

    // Topic subscription for direct joint trajectory commands (Web UI sliders, Unfold, moveit_servo)
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory",
      10,
      std::bind(&TrajectoryBridge::joint_trajectory_cb, this, std::placeholders::_1));

    // Action server for MoveIt trajectory execution (AUTO mode / templates)
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "arm_controller/follow_joint_trajectory",
      std::bind(&TrajectoryBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryBridge::handle_cancel, this, std::placeholders::_1),
      std::bind(&TrajectoryBridge::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ Manipulator Trajectory Bridge (Action + Topic) started.");
  }

private:
  void joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto out = *msg;
    out.header.stamp = this->get_clock()->now();
    js_pub_->publish(out);
  }

  void send_point_to_esp32(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & point,
    float duration)
  {
    const size_t count = std::min(joint_names.size(), point.positions.size());
    for (size_t i = 0; i < count; ++i) {
      auto it = joint_map_.find(joint_names[i]);
      if (it != joint_map_.end()) {
        std_msgs::msg::Float32MultiArray cmd_msg;
        cmd_msg.data = {
          it->second,
          static_cast<float>(point.positions[i]),
          0.0f,
          duration
        };
        servo_pub_->publish(cmd_msg);
      }
    }
  }

  void joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty() || msg->joint_names.empty()) {
      return;
    }

    // Single-point command (Web UI sliders, Unfold, moveit_servo) — dispatch immediately
    if (msg->points.size() == 1) {
      const auto & pt = msg->points[0];
      double t = static_cast<double>(pt.time_from_start.sec) +
                 static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
      float duration = static_cast<float>(t > 0.02 ? t : 0.05);
      send_point_to_esp32(msg->joint_names, pt, duration);
      return;
    }

    // Multi-point trajectory over topic
    std::thread([this, traj = *msg]() {
      double prev_time = 0.0;
      for (const auto & point : traj.points) {
        if (!rclcpp::ok()) return;
        double current_time = static_cast<double>(point.time_from_start.sec) +
                              static_cast<double>(point.time_from_start.nanosec) * 1e-9;
        double duration = current_time - prev_time;
        if (duration <= 0.01) {
          duration = 0.05;
        }
        send_point_to_esp32(traj.joint_names, point, static_cast<float>(duration));
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        prev_time = current_time;
      }
    }).detach();
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT>)
  {
    RCLCPP_WARN(this->get_logger(), "MoveIt trajectory canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    std::thread{std::bind(&TrajectoryBridge::execute_action, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_action(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing trajectory from MoveIt...");
    const auto & trajectory = goal_handle->get_goal()->trajectory;
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    double prev_time = 0.0;

    for (const auto & point : trajectory.points) {
      if (!rclcpp::ok()) {
        return;
      }
      if (goal_handle->is_canceling()) {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        goal_handle->canceled(result);
        return;
      }

      double current_time = static_cast<double>(point.time_from_start.sec) +
                            static_cast<double>(point.time_from_start.nanosec) * 1e-9;
      double duration = current_time - prev_time;
      if (duration <= 0.01) {
        duration = 0.05;
      }

      send_point_to_esp32(trajectory.joint_names, point, static_cast<float>(duration));

      // Slice sleep into 10 ms intervals for responsive cancellation
      double elapsed = 0.0;
      while (elapsed < duration && rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
          result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
          goal_handle->canceled(result);
          return;
        }
        double step = std::min(0.01, duration - elapsed);
        std::this_thread::sleep_for(std::chrono::duration<double>(step));
        elapsed += step;
      }

      prev_time = current_time;
    }

    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
  }

  std::unordered_map<std::string, float> joint_map_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryBridge>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
