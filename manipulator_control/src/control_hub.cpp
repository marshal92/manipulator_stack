#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <manipulator_interfaces/action/manipulator_goal.hpp>
#include <manipulator_interfaces/action/hub_command.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/msg/position_ik_request.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using json = nlohmann::json;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {

geometry_msgs::msg::Quaternion quaternion_multiply(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2)
{
  geometry_msgs::msg::Quaternion q;
  q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  return q;
}

geometry_msgs::msg::Quaternion make_z_quaternion(double angle)
{
  geometry_msgs::msg::Quaternion q_rot;
  q_rot.x = 0.0;
  q_rot.y = 0.0;
  q_rot.z = std::sin(angle / 2.0);
  q_rot.w = std::cos(angle / 2.0);
  return q_rot;
}

geometry_msgs::msg::Quaternion rotate_q_z(
  const geometry_msgs::msg::Quaternion & q_base,
  double angle)
{
  return quaternion_multiply(q_base, make_z_quaternion(angle));
}

geometry_msgs::msg::Quaternion rotate_world_z(
  const geometry_msgs::msg::Quaternion & q_base,
  double angle)
{
  return quaternion_multiply(make_z_quaternion(angle), q_base);
}

struct ScopeExitGuard {
  std::function<void()> fn;
  ~ScopeExitGuard() { if (fn) fn(); }
};

} // namespace

class ControlHubCpp : public rclcpp::Node {
public:
  using HubCommand = manipulator_interfaces::action::HubCommand;
  using GoalHandleHub = rclcpp_action::ServerGoalHandle<HubCommand>;
  using ManipulatorGoal = manipulator_interfaces::action::ManipulatorGoal;
  using GetPositionIK = moveit_msgs::srv::GetPositionIK;
  using GetPositionFK = moveit_msgs::srv::GetPositionFK;

  ControlHubCpp() : Node("control_hub") {
    web_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_teleop/hub_request", 10, std::bind(&ControlHubCpp::web_cmd_callback, this, _1));
    web_status_pub_ = this->create_publisher<std_msgs::msg::String>("/web_teleop/hub_status", 10);

    action_client_ = rclcpp_action::create_client<ManipulatorGoal>(this, "manipulator_action");
    ik_client_ = this->create_client<GetPositionIK>("/compute_ik");
    fk_client_ = this->create_client<GetPositionFK>("/compute_fk");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&ControlHubCpp::joint_states_callback, this, _1));

    action_server_ = rclcpp_action::create_server<HubCommand>(
      this,
      "hub_command",
      std::bind(&ControlHubCpp::handle_goal, this, _1, _2),
      std::bind(&ControlHubCpp::handle_cancel, this, _1),
      std::bind(&ControlHubCpp::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "C++ Control Hub started and waiting for commands.");
  }

private:
  std::atomic<bool> is_executing_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<uint64_t> task_seq_{0};
  std::mutex js_mutex_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;

  rclcpp_action::Server<HubCommand>::SharedPtr action_server_;
  rclcpp_action::Client<ManipulatorGoal>::SharedPtr action_client_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<GetPositionFK>::SharedPtr fk_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr web_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Only store messages that contain our arm joints
    for (const auto & name : msg->name) {
      if (name.rfind("arm_joint_", 0) == 0) {
        std::lock_guard<std::mutex> lock(js_mutex_);
        current_joint_state_ = msg;
        return;
      }
    }
  }

  sensor_msgs::msg::JointState::SharedPtr get_latest_joint_state() {
    std::lock_guard<std::mutex> lock(js_mutex_);
    return current_joint_state_;
  }

  void publish_web_status(const std::string & state, int progress, bool has_result = false, bool success = false, const std::string & message = "", const std::string & command = "") {
    json data;
    data["state"] = state;
    data["progress"] = progress;
    if (!command.empty()) {
      data["command"] = command;
    }
    if (has_result) {
      data["done"] = true;
      data["success"] = success;
      data["message"] = message;
    } else {
      data["done"] = false;
    }
    std_msgs::msg::String msg;
    msg.data = data.dump();
    web_status_pub_->publish(msg);
  }

  void web_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    try {
      auto req = json::parse(msg->data);
      std::string cmd = req.value("command", "");
      std::string file_path = req.value("file_path", "");
      double speed = req.value("speed", 0.3);

      if (cmd == "RESET" || cmd == "STOP" || cmd == "CANCEL") {
        cancel_requested_.store(true);
        task_seq_.fetch_add(1);
        is_executing_.store(false);
        action_client_->async_cancel_all_goals();
        publish_web_status("Ready (Reset)", 0, true, true, "Task reset", "RESET");
        return;
      }

      bool expected = false;
      if (!is_executing_.compare_exchange_strong(expected, true)) {
        publish_web_status("Error: Hub busy", 0, true, false, "Another task is already running", cmd);
        return;
      }

      cancel_requested_.store(false);
      const uint64_t seq = task_seq_.fetch_add(1) + 1;
      if (cmd == "EXECUTE_CSV" && !file_path.empty()) {
        std::thread(&ControlHubCpp::execute_csv_task_web, this, file_path, speed, seq).detach();
      } else if (cmd == "GOTO_XYZ") {
        double x = req.value("x", std::numeric_limits<double>::quiet_NaN());
        double y = req.value("y", std::numeric_limits<double>::quiet_NaN());
        double z = req.value("z", std::numeric_limits<double>::quiet_NaN());
        std::thread(&ControlHubCpp::execute_goto_xyz_task_web, this, x, y, z, speed, seq).detach();
      } else {
        is_executing_.store(false);
        publish_web_status("Error", 0, true, false, "Unknown command or empty file path", cmd);
      }
    } catch (const std::exception & e) {
      is_executing_.store(false);
      publish_web_status("Parse error", 0, true, false, e.what());
    }
  }

  void release_if_current(uint64_t seq) {
    if (task_seq_.load() == seq) {
      is_executing_.store(false);
    }
  }

  bool is_task_stale(uint64_t seq) const {
    return cancel_requested_.load() || task_seq_.load() != seq;
  }

  void execute_csv_task_web(std::string csv_path, double speed, uint64_t seq) {
    ScopeExitGuard guard{[this, seq]() { release_if_current(seq); }};
    auto update_status = [this, seq](const std::string & state, int prog) {
      if (!is_task_stale(seq)) {
        publish_web_status(state, prog, false, false, "", "EXECUTE_CSV");
      }
    };

    std::string out_msg;
    bool ok = run_csv_logic(csv_path, update_status, out_msg, speed);
    if (!is_task_stale(seq)) {
      publish_web_status(ok ? "Success" : "Error", 100, true, ok, out_msg, "EXECUTE_CSV");
    }
  }

  void execute_goto_xyz_task_web(double x, double y, double z, double speed, uint64_t seq) {
    ScopeExitGuard guard{[this, seq]() { release_if_current(seq); }};
    auto update_status = [this, seq](const std::string & state, int prog) {
      if (!is_task_stale(seq)) {
        publish_web_status(state, prog, false, false, "", "GOTO_XYZ");
      }
    };

    std::string out_msg;
    bool ok = run_goto_xyz_logic(x, y, z, update_status, out_msg, speed, seq);
    if (!is_task_stale(seq)) {
      publish_web_status(ok ? "Success" : "Error", 100, true, ok, out_msg, "GOTO_XYZ");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const HubCommand::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Hub Action received command: %s", goal->command.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHub>) {
    action_client_->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHub> goal_handle) {
    std::thread{std::bind(&ControlHubCpp::execute_action, this, _1), goal_handle}.detach();
  }

  void execute_action(const std::shared_ptr<GoalHandleHub> goal_handle) {
    auto result = std::make_shared<HubCommand::Result>();
    bool expected = false;
    if (!is_executing_.compare_exchange_strong(expected, true)) {
      result->success = false;
      result->message = "Hub busy";
      goal_handle->abort(result);
      return;
    }
    ScopeExitGuard guard{[this]() { is_executing_.store(false); }};

    const auto req = goal_handle->get_goal();
    if (req->command == "EXECUTE_CSV") {
      auto feedback = std::make_shared<HubCommand::Feedback>();
      auto update_status = [goal_handle, feedback](const std::string & state, int prog) {
        feedback->current_state = state;
        feedback->progress = prog;
        goal_handle->publish_feedback(feedback);
      };

      std::string out_msg;
      bool ok = run_csv_logic(req->file_path, update_status, out_msg);
      result->success = ok;
      result->message = out_msg;
      if (ok) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    } else {
      result->success = false;
      result->message = "Unknown command";
      goal_handle->abort(result);
    }
  }

  bool compute_fk_pose(
    const moveit_msgs::msg::RobotState & robot_state,
    geometry_msgs::msg::Pose & out_pose)
  {
    auto req_fk = std::make_shared<GetPositionFK::Request>();
    req_fk->header.frame_id = "arm_base_link";
    req_fk->fk_link_names = {"arm_tool0"};
    req_fk->robot_state = robot_state;

    auto future_fk = fk_client_->async_send_request(req_fk);
    if (future_fk.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      return false;
    }
    auto res_fk = future_fk.get();
    if (res_fk->error_code.val != 1 || res_fk->pose_stamped.empty()) {
      return false;
    }
    out_pose = res_fk->pose_stamped[0].pose;
    return true;
  }

  bool get_ik_for_point(
    double x, double y, double z,
    const geometry_msgs::msg::Quaternion & target_q,
    const moveit_msgs::msg::RobotState & seed_joints,
    geometry_msgs::msg::Pose & out_pose,
    moveit_msgs::msg::RobotState & out_solution)
  {
    auto req_ik = std::make_shared<GetPositionIK::Request>();
    req_ik->ik_request.group_name = "arm";
    req_ik->ik_request.avoid_collisions = true;
    req_ik->ik_request.robot_state = seed_joints;

    req_ik->ik_request.pose_stamped.header.frame_id = "arm_base_link";
    req_ik->ik_request.pose_stamped.pose.position.x = x;
    req_ik->ik_request.pose_stamped.pose.position.y = y;
    req_ik->ik_request.pose_stamped.pose.position.z = z;
    req_ik->ik_request.pose_stamped.pose.orientation = target_q;

    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "arm_base_link";
    oc.link_name = "arm_tool0";
    oc.orientation = target_q;
    oc.absolute_x_axis_tolerance = 0.2;
    oc.absolute_y_axis_tolerance = 0.2;
    oc.absolute_z_axis_tolerance = 0.2;
    oc.weight = 1.0;
    req_ik->ik_request.constraints.orientation_constraints.push_back(oc);

    auto future_ik = ik_client_->async_send_request(req_ik);
    if (future_ik.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      return false;
    }
    auto res_ik = future_ik.get();
    if (res_ik->error_code.val != 1) {
      return false;
    }

    if (!compute_fk_pose(res_ik->solution, out_pose)) {
      return false;
    }
    out_solution = res_ik->solution;
    return true;
  }

  bool send_goal_and_wait(
    const std::string & mode,
    const std::vector<geometry_msgs::msg::Pose> & poses,
    bool ignore_ori,
    double speed,
    double pos_tol,
    double ori_tol)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Tactical server (manipulator_action) is unavailable!");
      return false;
    }

    ManipulatorGoal::Goal goal_msg;
    goal_msg.mode = mode;
    goal_msg.poses = poses;
    goal_msg.ignore_orientation = ignore_ori;
    goal_msg.speed = speed;
    goal_msg.pos_tolerance = pos_tol;
    goal_msg.ori_tolerance = ori_tol;

    auto send_goal_future = action_client_->async_send_goal(goal_msg);
    if (send_goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      return false;
    }
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      return false;
    }

    auto res_future = action_client_->async_get_result(goal_handle);
    if (res_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      return false;
    }
    auto wrapped = res_future.get();
    return wrapped.result && wrapped.result->success;
  }

  bool prepare_moveit_context(
    moveit_msgs::msg::RobotState & start_robot_state,
    geometry_msgs::msg::Pose & current_pose,
    std::string & out_msg)
  {
    if (!ik_client_->wait_for_service(std::chrono::seconds(5)) ||
        !fk_client_->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "MoveIt services (/compute_ik, /compute_fk) are unavailable!");
      out_msg = "MoveIt (/compute_ik) service unavailable";
      return false;
    }

    // Wait up to 3s for joint states
    sensor_msgs::msg::JointState::SharedPtr js = get_latest_joint_state();
    for (int i = 0; i < 30 && !js && rclcpp::ok(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      js = get_latest_joint_state();
    }
    if (!js) {
      RCLCPP_ERROR(this->get_logger(), "No /joint_states received from manipulator!");
      out_msg = "No /joint_states received";
      return false;
    }

    start_robot_state.joint_state = *js;
    if (!compute_fk_pose(start_robot_state, current_pose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute current tool pose via FK!");
      out_msg = "Failed to compute FK pose";
      return false;
    }
    return true;
  }

  bool run_goto_xyz_logic(
    double x, double y, double z,
    const std::function<void(const std::string &, int)> & status_cb,
    std::string & out_msg,
    double speed,
    uint64_t seq)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      out_msg = "Invalid target coordinates";
      return false;
    }

    const double dist_shoulder = std::hypot(x, y, z - 0.09);
    if (dist_shoulder > 0.65 || z < -0.15) {
      out_msg = "Target out of arm workspace";
      return false;
    }

    status_cb("Checking target IK...", 20);
    moveit_msgs::msg::RobotState start_robot_state;
    geometry_msgs::msg::Pose current_pose;
    if (!prepare_moveit_context(start_robot_state, current_pose, out_msg)) {
      return false;
    }
    if (is_task_stale(seq)) {
      out_msg = "Cancelled";
      return false;
    }

    const double clamped_speed = std::clamp(speed, 0.05, 1.0);
    const double target_az = (std::hypot(x, y) > 0.03) ? std::atan2(y, x) : 0.0;
    const double curr_az = (std::hypot(current_pose.position.x, current_pose.position.y) > 0.03)
      ? std::atan2(current_pose.position.y, current_pose.position.x) : 0.0;
    const double d_yaw = std::atan2(std::sin(target_az - curr_az), std::cos(target_az - curr_az));
    const auto q_az = rotate_world_z(current_pose.orientation, d_yaw);

    std::vector<geometry_msgs::msg::Quaternion> q_candidates;
    q_candidates.push_back(q_az);
    if (std::abs(d_yaw) > 0.05) {
      q_candidates.push_back(current_pose.orientation);
    }
    for (double angle : {0.35, -0.35, 0.70, -0.70, 1.10, -1.10, 1.57, -1.57}) {
      q_candidates.push_back(rotate_q_z(q_az, angle));
    }

    geometry_msgs::msg::Pose solved_pose;
    moveit_msgs::msg::RobotState solved_state;
    bool found_ik = false;
    for (const auto & q_cand : q_candidates) {
      if (is_task_stale(seq)) {
        out_msg = "Cancelled";
        return false;
      }
      if (get_ik_for_point(x, y, z, q_cand, start_robot_state, solved_pose, solved_state)) {
        found_ik = true;
        break;
      }
    }

    bool ok = false;
    if (found_ik) {
      status_cb("Moving to target...", 55);
      ok = send_goal_and_wait("POSE", {solved_pose}, false, clamped_speed, 0.008, 0.15);
    } else {
      status_cb("Planning position-only path...", 45);
      geometry_msgs::msg::Pose fallback_pose;
      fallback_pose.position.x = x;
      fallback_pose.position.y = y;
      fallback_pose.position.z = z;
      fallback_pose.orientation = q_az;
      ok = send_goal_and_wait("POSE", {fallback_pose}, true, clamped_speed, 0.015, 0.25);
    }

    if (ok) {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "Reached (%.1f, %.1f, %.1f) mm", x * 1000.0, y * 1000.0, z * 1000.0);
      out_msg = buf;
      return true;
    }
    out_msg = is_task_stale(seq) ? "Cancelled" : "Target unreachable or planning failed";
    return false;
  }

  bool run_csv_logic(
    const std::string & csv_path,
    const std::function<void(const std::string &, int)> & status_cb,
    std::string & out_msg,
    double speed = 0.3)
  {
    status_cb("Reading CSV file...", 10);

    std::ifstream file(csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "File not found: %s", csv_path.c_str());
      out_msg = "File not found";
      return false;
    }

    struct Pt3D { double x; double y; double z; };
    std::vector<Pt3D> raw_points;
    std::string line;
    // Skip header
    std::getline(file, line);
    while (std::getline(file, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string cell;
      std::vector<double> vals;
      while (std::getline(ss, cell, ',')) {
        try {
          double v = std::stod(cell);
          if (!std::isfinite(v)) {
            out_msg = "Invalid number in CSV";
            return false;
          }
          vals.push_back(v);
        } catch (...) {
          out_msg = "Error parsing coordinates in CSV";
          return false;
        }
      }
      if (vals.size() >= 3) {
        raw_points.push_back({vals[0], vals[1], vals[2]});
      }
    }

    if (raw_points.empty()) {
      out_msg = "CSV file is empty";
      return false;
    }

    status_cb("Searching valid IK chain...", 20);

    moveit_msgs::msg::RobotState start_robot_state;
    geometry_msgs::msg::Pose current_pose;
    if (!prepare_moveit_context(start_robot_state, current_pose, out_msg)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
      "Current arm_tool0 pose: X=%.3f, Y=%.3f, Z=%.3f | First CSV point: X=%.3f, Y=%.3f, Z=%.3f",
      current_pose.position.x, current_pose.position.y, current_pose.position.z,
      raw_points[0].x, raw_points[0].y, raw_points[0].z);

    const std::vector<double> variations = {0.0, 0.3, -0.3, 0.7, -0.7, 1.57, -1.57, 2.3, -2.3, 3.14};
    std::vector<geometry_msgs::msg::Pose> valid_poses;
    bool found_chain = false;

    for (double angle : variations) {
      RCLCPP_INFO(this->get_logger(), "Trying IK chain (wrist rotation %.2f rad)...", angle);
      auto test_q = rotate_q_z(current_pose.orientation, angle);

      std::vector<geometry_msgs::msg::Pose> temp_poses;
      temp_poses.reserve(raw_points.size());
      auto chain_q = test_q;
      auto chain_joints = start_robot_state;

      bool success_ik = true;
      for (size_t i = 0; i < raw_points.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        moveit_msgs::msg::RobotState next_joints;
        if (!get_ik_for_point(raw_points[i].x, raw_points[i].y, raw_points[i].z, chain_q, chain_joints, pose, next_joints)) {
          RCLCPP_INFO(this->get_logger(),
            "   -> IK failed at point #%zu/%zu: (%.3f, %.3f, %.3f)",
            i + 1, raw_points.size(), raw_points[i].x, raw_points[i].y, raw_points[i].z);
          success_ik = false;
          break;
        }
        temp_poses.push_back(pose);
        chain_q = pose.orientation;
        chain_joints = next_joints;
      }

      if (success_ik) {
        valid_poses = std::move(temp_poses);
        found_chain = true;
        break;
      }
    }

    if (!found_chain || valid_poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Target shape is physically unreachable.");
      out_msg = "Target shape is physically unreachable from current pose";
      return false;
    }

    status_cb("Executing: Approaching start point...", 40);

    double approach_speed = std::min(1.0, std::max(0.05, speed * 1.5));
    if (!send_goal_and_wait("POSE", {valid_poses[0]}, false, approach_speed, 0.05, 0.2)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach start point!");
      out_msg = "Approach failed";
      return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    status_cb("Executing: Drawing path...", 60);

    if (send_goal_and_wait("CARTESIAN", valid_poses, false, speed, 0.05, 0.2)) {
      RCLCPP_INFO(this->get_logger(), "Shape path completed successfully!");
      out_msg = "Successfully completed";
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Cartesian path execution failed.");
      out_msg = "Cartesian path error";
      return false;
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlHubCpp>());
  rclcpp::shutdown();
  return 0;
}
