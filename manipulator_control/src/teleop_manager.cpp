#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using std::placeholders::_1;

namespace {

constexpr size_t kNumJoints = 6;
const std::array<std::string, kNumJoints> kArmJointNames = {
  "arm_joint_0", "arm_joint_1", "arm_joint_2",
  "arm_joint_3", "arm_joint_4", "arm_joint_5"
};

}  // namespace

class TeleopManagerCpp : public rclcpp::Node {
public:
  TeleopManagerCpp() : Node("teleop_manager") {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "arm_base_link");
    tip_frame_ = this->declare_parameter<std::string>("tip_frame", "arm_tool0");
    linear_scale_ = this->declare_parameter<double>("linear_scale", 0.25);
    rotational_scale_ = this->declare_parameter<double>("rotational_scale", 1.50);
    joint_scale_ = this->declare_parameter<double>("joint_scale", 3.14);
    max_joint_vel_ = this->declare_parameter<double>("max_joint_vel", 2.5);
    dls_damping_ = this->declare_parameter<double>("dls_damping", 0.020);
    rot_linear_weight_ = this->declare_parameter<double>("rot_linear_weight", 2.50);
    tool_trans_rot_weight_ = this->declare_parameter<double>("tool_trans_rot_weight", 0.04);
    soft_limit_margin_ = this->declare_parameter<double>("soft_limit_margin", 0.07);
    hard_limit_margin_ = this->declare_parameter<double>("hard_limit_margin", 0.025);
    kp_pos_ = this->declare_parameter<double>("kp_pos", 4.0);
    max_pos_lead_ = this->declare_parameter<double>("max_pos_lead", 0.035);
    line_cos_min_ = this->declare_parameter<double>("line_cos_min", 0.45);
    line_cos_full_ = this->declare_parameter<double>("line_cos_full", 0.70);
    rot_pivot_guard_lo_ = this->declare_parameter<double>("rot_pivot_guard_lo", 0.015);
    rot_pivot_guard_hi_ = this->declare_parameter<double>("rot_pivot_guard_hi", 0.035);

    auto wq_t = this->declare_parameter<std::vector<double>>(
      "w_q_trans", {1.0, 1.0, 1.0, 0.005, 0.35, 0.005});
    auto wq_r = this->declare_parameter<std::vector<double>>(
      "w_q_rot", {0.8, 0.6, 0.6, 0.25, 1.0, 1.0});
    for (size_t i = 0; i < kNumJoints; ++i) {
      w_q_trans_(static_cast<Eigen::Index>(i)) = (i < wq_t.size()) ? wq_t[i] : 1.0;
      w_q_rot_(static_cast<Eigen::Index>(i)) = (i < wq_r.size()) ? wq_r[i] : 1.0;
    }

    const std::string urdf_param = this->declare_parameter<std::string>("robot_description", "");
    if (!urdf_param.empty()) {
      init_kdl_from_urdf(urdf_param);
    }

    servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    servo_jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/web_teleop/tool_pose", 10);

    web_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/web_teleop/cmd_vel", 10, std::bind(&TeleopManagerCpp::cmd_vel_callback, this, _1));
    web_jog_sub_ = this->create_subscription<control_msgs::msg::JointJog>(
      "/web_teleop/cmd_jog", 10, std::bind(&TeleopManagerCpp::cmd_jog_callback, this, _1));
    unfold_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/web_teleop/unfold", 10, std::bind(&TeleopManagerCpp::unfold_callback, this, _1));
    fold_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/web_teleop/fold", 10, std::bind(&TeleopManagerCpp::fold_callback, this, _1));
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_teleop/mode", 10, std::bind(&TeleopManagerCpp::mode_callback, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&TeleopManagerCpp::joint_states_callback, this, _1));
    robot_desc_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&TeleopManagerCpp::robot_description_callback, this, _1));

    switch_srv_ = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // 50 Hz timer starts cancelled (Zero-Idle mode) and wakes up only during active teleop
    servo_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&TeleopManagerCpp::publish_current_command, this));
    servo_timer_->cancel();

    // 5 Hz telemetry timer (only queries TF when someone is subscribed to /web_teleop/tool_pose)
    telemetry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&TeleopManagerCpp::publish_telemetry, this));

    last_cmd_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "C++ Teleop Manager started (Zero-Idle + Weighted DLS drift mode).");
  }

private:
  enum class InputSource { NONE, TWIST_DLS, DIRECT_JOG };
  enum class RefMode { NONE, LINEAR, ROTATION };

  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  std::string base_frame_{"arm_base_link"};
  std::string tip_frame_{"arm_tool0"};
  double linear_scale_{0.25};
  double rotational_scale_{1.50};
  double joint_scale_{3.14};
  double max_joint_vel_{2.5};
  double dls_damping_{0.020};
  double rot_linear_weight_{2.50};
  double tool_trans_rot_weight_{0.04};
  double soft_limit_margin_{0.07};
  double hard_limit_margin_{0.025};
  double kp_pos_{4.0};
  double max_pos_lead_{0.035};
  double line_cos_min_{0.45};
  double line_cos_full_{0.70};
  double rot_pivot_guard_lo_{0.015};
  double rot_pivot_guard_hi_{0.035};
  Vec6 w_q_trans_{Vec6::Ones()};
  Vec6 w_q_rot_{Vec6::Ones()};

  bool has_p_ref_{false};
  RefMode ref_mode_{RefMode::NONE};
  Eigen::Vector3d p_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_cmd_dir_{Eigen::Vector3d::Zero()};
  std::string last_cmd_frame_{""};

  bool kdl_ready_{false};
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::vector<size_t> kdl_to_arm_idx_;
  std::array<double, kNumJoints> q_lower_{{-1.57, -0.05, -3.20, -1.57, -1.57, -1.57}};
  std::array<double, kNumJoints> q_upper_{{1.57, 3.14, 0.05, 1.57, 1.57, 1.57}};
  std::array<double, kNumJoints> q_current_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<bool, kNumJoints> joint_seen_{{false, false, false, false, false, false}};
  bool has_joint_state_{false};

  geometry_msgs::msg::Twist current_twist_;
  std::string current_frame_id_{"arm_base_link"};
  control_msgs::msg::JointJog current_jog_;
  InputSource input_source_{InputSource::NONE};
  std::string active_mode_{""};
  std::string target_mode_{""};
  std::string system_mode_{"MANUAL"};
  bool switch_in_progress_{false};
  int zero_msgs_sent_{10};
  std::chrono::steady_clock::time_point last_cmd_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_jog_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr web_twist_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr web_jog_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr unfold_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fold_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_srv_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  void wake_servo_timer() {
    if (servo_timer_->is_canceled()) {
      servo_timer_->reset();
    }
  }

  void publish_telemetry() {
    if (pose_pub_->get_subscription_count() == 0) {
      return; // Zero overhead when web panel is not listening
    }
    try {
      auto t = tf_buffer_->lookupTransform("arm_base_link", "arm_tool0", tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "arm_base_link";
      pose.pose.position.x = t.transform.translation.x;
      pose.pose.position.y = t.transform.translation.y;
      pose.pose.position.z = t.transform.translation.z;
      pose.pose.orientation = t.transform.rotation;
      pose_pub_->publish(pose);
    } catch (const tf2::TransformException &) {
      // Suppress warning if TF is not ready yet
    }
  }

  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!kdl_ready_ && !msg->data.empty()) {
      init_kdl_from_urdf(msg->data);
    }
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < kNumJoints; ++j) {
        if (msg->name[i] == kArmJointNames[j]) {
          q_current_[j] = msg->position[i];
          joint_seen_[j] = true;
          break;
        }
      }
    }
    if (!has_joint_state_) {
      has_joint_state_ = std::all_of(joint_seen_.begin(), joint_seen_.end(), [](bool v) { return v; });
      if (has_joint_state_) {
        RCLCPP_INFO(this->get_logger(), "Arm joint states synchronized (%s).",
                    kdl_ready_ ? "Weighted DLS active" : "waiting for URDF");
      }
    }
  }

  void init_kdl_from_urdf(const std::string & urdf_xml) {
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_xml)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF XML for KDL DLS teleop.");
      return;
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from URDF.");
      return;
    }
    KDL::Chain chain;
    if (!tree.getChain(base_frame_, tip_frame_, chain)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain '%s' -> '%s'.",
                   base_frame_.c_str(), tip_frame_.c_str());
      return;
    }
    if (chain.getNrOfJoints() != kNumJoints) {
      RCLCPP_ERROR(this->get_logger(), "Expected %zu arm joints in KDL chain, got %u.",
                   kNumJoints, chain.getNrOfJoints());
      return;
    }

    std::unordered_map<std::string, size_t> name_to_idx;
    for (size_t j = 0; j < kNumJoints; ++j) {
      name_to_idx[kArmJointNames[j]] = j;
    }

    std::vector<size_t> kdl_map;
    kdl_map.reserve(kNumJoints);
    for (unsigned int s = 0; s < chain.getNrOfSegments(); ++s) {
      const auto & joint = chain.getSegment(s).getJoint();
      if (joint.getType() == KDL::Joint::None) {
        continue;
      }
      auto it = name_to_idx.find(joint.getName());
      if (it == name_to_idx.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected joint '%s' in KDL chain.", joint.getName().c_str());
        return;
      }
      kdl_map.push_back(it->second);
      auto urdf_joint = urdf_model.getJoint(joint.getName());
      if (urdf_joint && urdf_joint->limits) {
        q_lower_[it->second] = urdf_joint->limits->lower;
        q_upper_[it->second] = urdf_joint->limits->upper;
      }
    }
    if (kdl_map.size() != kNumJoints) {
      return;
    }

    kdl_chain_ = chain;
    kdl_to_arm_idx_ = std::move(kdl_map);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
    kdl_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Initialized KDL chain '%s' -> '%s' for Weighted DLS + drift teleop.",
                base_frame_.c_str(), tip_frame_.c_str());
  }

  Vec6 solve_dls_step(const Mat6 & J_eval, const Vec6 & v_eval,
                      const Vec6 & wx, const Vec6 & wq) const
  {
    const Mat6 Jw = wx.asDiagonal() * J_eval;
    const Vec6 vw = wx.cwiseProduct(v_eval);
    const Mat6 A = Jw * wq.asDiagonal() * Jw.transpose() +
      (dls_damping_ * dls_damping_) * Mat6::Identity();
    const Vec6 y = A.ldlt().solve(vw);
    return wq.asDiagonal() * (Jw.transpose() * y);
  }

  control_msgs::msg::JointJog make_zero_jog() const {
    control_msgs::msg::JointJog msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = base_frame_;
    msg.joint_names.assign(kArmJointNames.begin(), kArmJointNames.end());
    msg.velocities.assign(kNumJoints, 0.0);
    return msg;
  }

  control_msgs::msg::JointJog compute_weighted_dls_jog() {
    KDL::JntArray q_kdl(kNumJoints);
    for (unsigned int k = 0; k < kNumJoints; ++k) {
      q_kdl(k) = q_current_[kdl_to_arm_idx_[k]];
    }

    KDL::Frame F_bt;
    KDL::Jacobian jac_kdl(kNumJoints);
    if (fk_solver_->JntToCart(q_kdl, F_bt) < 0 || jac_solver_->JntToJac(q_kdl, jac_kdl) < 0) {
      has_p_ref_ = false;
      ref_mode_ = RefMode::NONE;
      return make_zero_jog();
    }

    Mat6 J_base = jac_kdl.data;
    Vec6 v_eval;
    v_eval <<
      current_twist_.linear.x * linear_scale_,
      current_twist_.linear.y * linear_scale_,
      current_twist_.linear.z * linear_scale_,
      current_twist_.angular.x * rotational_scale_,
      current_twist_.angular.y * rotational_scale_,
      current_twist_.angular.z * rotational_scale_;

    Eigen::Matrix3d R_bt;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R_bt(r, c) = F_bt.M(r, c);
      }
    }
    const Eigen::Vector3d p_actual(F_bt.p.x(), F_bt.p.y(), F_bt.p.z());

    const bool is_tool_frame = (current_frame_id_ == tip_frame_ || current_frame_id_ == "tool0");
    Mat6 J_eval = J_base;
    if (is_tool_frame) {
      J_eval.topRows<3>() = R_bt.transpose() * J_base.topRows<3>();
      J_eval.bottomRows<3>() = R_bt.transpose() * J_base.bottomRows<3>();
    } else if (!current_frame_id_.empty() && current_frame_id_ != base_frame_) {
      try {
        auto tf = tf_buffer_->lookupTransform(base_frame_, current_frame_id_, tf2::TimePointZero);
        tf2::Quaternion q_tf(tf.transform.rotation.x, tf.transform.rotation.y,
                             tf.transform.rotation.z, tf.transform.rotation.w);
        tf2::Matrix3x3 m_tf(q_tf);
        Eigen::Matrix3d R_bf;
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            R_bf(r, c) = m_tf[r][c];
          }
        }
        v_eval.head<3>() = R_bf * v_eval.head<3>();
        v_eval.tail<3>() = R_bf * v_eval.tail<3>();
      } catch (const tf2::TransformException &) {
        // Keep base-frame interpretation if TF transform unavailable
      }
    }

    const double lin_norm = v_eval.head<3>().norm();
    const double rot_norm = v_eval.tail<3>().norm();
    if (lin_norm < 1e-6 && rot_norm < 1e-6) {
      has_p_ref_ = false;
      ref_mode_ = RefMode::NONE;
      return make_zero_jog();
    }

    constexpr double kDt = 0.02;
    const bool pure_linear = (lin_norm >= 1e-6 && rot_norm < 1e-6);
    const bool pure_rotation = (rot_norm >= 1e-6 && lin_norm < 1e-6);
    Eigen::Vector3d v_cmd_raw_eval = v_eval.head<3>();
    if (pure_linear) {
      const Eigen::Vector3d v_cmd_base = is_tool_frame ? (R_bt * v_cmd_raw_eval) : v_cmd_raw_eval;
      const Eigen::Vector3d cmd_dir_base = v_cmd_base.normalized();
      if (!has_p_ref_ || ref_mode_ != RefMode::LINEAR || last_cmd_frame_ != current_frame_id_ || cmd_dir_base.dot(last_cmd_dir_) < 0.98) {
        p_ref_ = p_actual;
        has_p_ref_ = true;
      }
      ref_mode_ = RefMode::LINEAR;
      last_cmd_frame_ = current_frame_id_;
      last_cmd_dir_ = cmd_dir_base;

      p_ref_ += v_cmd_base * kDt;
      Eigen::Vector3d pos_err = p_ref_ - p_actual;
      const double err_norm = pos_err.norm();
      if (err_norm > max_pos_lead_ && err_norm > 1e-9) {
        pos_err *= (max_pos_lead_ / err_norm);
        p_ref_ = p_actual + pos_err;
      }
      if (is_tool_frame) {
        v_eval.head<3>() += kp_pos_ * (R_bt.transpose() * pos_err);
      } else {
        v_eval.head<3>() += kp_pos_ * pos_err;
      }
    } else if (pure_rotation) {
      // Lock TCP pivot point in base frame to stop backward/sideways herringbone drift during Pitch/Roll/Yaw
      const Eigen::Vector3d rot_dir_eval = v_eval.tail<3>().normalized();
      if (!has_p_ref_ || ref_mode_ != RefMode::ROTATION || last_cmd_frame_ != current_frame_id_ || rot_dir_eval.dot(last_cmd_dir_) < 0.95) {
        p_ref_ = p_actual;
        has_p_ref_ = true;
      }
      ref_mode_ = RefMode::ROTATION;
      last_cmd_frame_ = current_frame_id_;
      last_cmd_dir_ = rot_dir_eval;

      Eigen::Vector3d pos_err = p_ref_ - p_actual;
      const double err_norm = pos_err.norm();
      if (err_norm > max_pos_lead_ && err_norm > 1e-9) {
        pos_err *= (max_pos_lead_ / err_norm);
      }
      v_eval.head<3>() = kp_pos_ * (is_tool_frame ? (R_bt.transpose() * pos_err) : pos_err);
    } else {
      has_p_ref_ = false;
      ref_mode_ = RefMode::NONE;
    }

    // True drift_dimensions during translation + strong pivot hold during angular jogs
    const double alpha_rot = rot_norm / (rot_norm + lin_norm + 1e-9);
    const double w_lin = (1.0 - alpha_rot) * 1.0 + alpha_rot * rot_linear_weight_;
    const double w_rot = (1.0 - alpha_rot) * (is_tool_frame ? tool_trans_rot_weight_ : 0.0) + alpha_rot * 1.0;

    Vec6 wx;
    wx << w_lin, w_lin, w_lin, w_rot, w_rot, w_rot;

    const Vec6 wq_arm = (1.0 - alpha_rot) * w_q_trans_ + alpha_rot * w_q_rot_;
    Vec6 wq_kdl;
    for (unsigned int k = 0; k < kNumJoints; ++k) {
      wq_kdl(k) = wq_arm(static_cast<Eigen::Index>(kdl_to_arm_idx_[k]));
    }

    Vec6 qdot_kdl = solve_dls_step(J_eval, v_eval, wx, wq_kdl);

    // Directional soft limit tapering before servo_node's joint_limit_margins halt all joints
    Vec6 wq_eff = wq_kdl;
    bool tapered = false;
    const double margin_span = std::max(1e-3, soft_limit_margin_ - hard_limit_margin_);
    for (unsigned int k = 0; k < kNumJoints; ++k) {
      const size_t idx = kdl_to_arm_idx_[k];
      const double q = q_current_[idx];
      if (qdot_kdl(k) < 0.0 && (q - q_lower_[idx]) < soft_limit_margin_) {
        const double frac = std::clamp((q - q_lower_[idx] - hard_limit_margin_) / margin_span, 0.0, 1.0);
        wq_eff(k) *= frac * frac;
        tapered = true;
      } else if (qdot_kdl(k) > 0.0 && (q_upper_[idx] - q) < soft_limit_margin_) {
        const double frac = std::clamp((q_upper_[idx] - q - hard_limit_margin_) / margin_span, 0.0, 1.0);
        wq_eff(k) *= frac * frac;
        tapered = true;
      }
    }
    if (tapered) {
      qdot_kdl = solve_dls_step(J_eval, v_eval, wx, wq_eff);
    }

    // Line-fidelity guard: attenuate motion rather than drifting sideways when hitting a workspace wall
    if (pure_linear) {
      const Eigen::Vector3d v_pred = J_eval.topRows<3>() * qdot_kdl;
      const double vp_norm = v_pred.norm();
      if (vp_norm > 1e-5) {
        const double cos_align = v_pred.dot(v_cmd_raw_eval.normalized()) / vp_norm;
        if (cos_align < line_cos_full_) {
          const double scale = std::clamp(
            (cos_align - line_cos_min_) / std::max(1e-3, line_cos_full_ - line_cos_min_), 0.0, 1.0);
          qdot_kdl *= scale;
        }
      }
    } else if (pure_rotation) {
      // Pivot guard: attenuate angular rate when hitting a joint limit would swing TCP away from p_ref_
      const Eigen::Vector3d v_lin_eval = J_eval.topRows<3>() * qdot_kdl;
      const Eigen::Vector3d v_lin_base = is_tool_frame ? (R_bt * v_lin_eval) : v_lin_eval;
      const Eigen::Vector3d err_b = p_ref_ - p_actual;
      const double err_norm = err_b.norm();
      if (err_norm > rot_pivot_guard_lo_) {
        const double outward = v_lin_base.dot(-err_b / err_norm);
        if (outward > 0.01) {
          const double span = std::max(1e-3, rot_pivot_guard_hi_ - rot_pivot_guard_lo_);
          const double rot_scale = std::clamp(1.0 - (err_norm - rot_pivot_guard_lo_) / span, 0.0, 1.0);
          v_eval.tail<3>() *= rot_scale;
          qdot_kdl = solve_dls_step(J_eval, v_eval, wx, wq_eff);
        }
      }
    }

    Vec6 qdot_arm;
    for (unsigned int k = 0; k < kNumJoints; ++k) {
      qdot_arm(static_cast<Eigen::Index>(kdl_to_arm_idx_[k])) = qdot_kdl(k);
    }

    const double peak = qdot_arm.cwiseAbs().maxCoeff();
    if (peak > max_joint_vel_ && peak > 1e-9) {
      qdot_arm *= (max_joint_vel_ / peak);
    }

    control_msgs::msg::JointJog jog = make_zero_jog();
    for (size_t i = 0; i < kNumJoints; ++i) {
      double qd = qdot_arm(static_cast<Eigen::Index>(i));
      const double q = q_current_[i];
      if (qd < 0.0 && (q + qd * kDt) < (q_lower_[i] + hard_limit_margin_)) {
        qd = (q <= q_lower_[i] + hard_limit_margin_) ? 0.0 : (q_lower_[i] + hard_limit_margin_ - q) / kDt;
      } else if (qd > 0.0 && (q + qd * kDt) > (q_upper_[i] - hard_limit_margin_)) {
        qd = (q >= q_upper_[i] - hard_limit_margin_) ? 0.0 : (q_upper_[i] - hard_limit_margin_ - q) / kDt;
      }
      jog.velocities[i] = qd / joint_scale_;
    }
    return jog;
  }

  void request_mode_switch(const std::string & mode) {
    if (switch_in_progress_ || active_mode_ == mode) {
      return;
    }
    if (!switch_srv_->service_is_ready()) {
      return;
    }

    target_mode_ = mode;
    switch_in_progress_ = true;

    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    if (mode == "TWIST") {
      req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
    } else if (mode == "JOINT_JOG") {
      req->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    }

    switch_srv_->async_send_request(
      req,
      [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
        switch_in_progress_ = false;
        try {
          auto res = future.get();
          if (res->success) {
            active_mode_ = target_mode_;
            RCLCPP_INFO(this->get_logger(), "Switched Servo to %s mode.", active_mode_.c_str());
          } else {
            RCLCPP_WARN(this->get_logger(), "Failed to switch Servo to %s mode.", target_mode_.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
      });
  }

  bool is_twist_zero() const {
    return (current_twist_.linear.x == 0.0 &&
            current_twist_.linear.y == 0.0 &&
            current_twist_.linear.z == 0.0 &&
            current_twist_.angular.x == 0.0 &&
            current_twist_.angular.y == 0.0 &&
            current_twist_.angular.z == 0.0);
  }

  bool is_jog_zero() const {
    if (current_jog_.velocities.empty()) return true;
    for (double v : current_jog_.velocities) {
      if (v != 0.0) return false;
    }
    return true;
  }

  void publish_stop_for_active_mode() {
    if (active_mode_ == "TWIST") {
      geometry_msgs::msg::TwistStamped stamped_msg;
      stamped_msg.header.stamp = this->now();
      stamped_msg.header.frame_id = current_frame_id_;
      servo_twist_pub_->publish(stamped_msg);
    } else if (active_mode_ == "JOINT_JOG") {
      servo_jog_pub_->publish(make_zero_jog());
    }
  }

  void publish_current_command() {
    // Safety watchdog: if button held > 10s without new message (e.g. lost connection), auto-stop
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_time_).count();
    if (elapsed > 10.0 && (!is_twist_zero() || !is_jog_zero())) {
      RCLCPP_WARN(this->get_logger(), "Teleop command timeout (10s). Stopping motion.");
      current_twist_ = geometry_msgs::msg::Twist();
      current_jog_ = control_msgs::msg::JointJog();
      zero_msgs_sent_ = 0;
    }

    if (system_mode_ == "AUTO") {
      has_p_ref_ = false;
      ref_mode_ = RefMode::NONE;
      if (zero_msgs_sent_ <= 5) {
        zero_msgs_sent_++;
        publish_stop_for_active_mode();
      } else {
        input_source_ = InputSource::NONE;
        servo_timer_->cancel();
      }
      return;
    }

    if (input_source_ == InputSource::TWIST_DLS) {
      if (is_twist_zero()) {
        has_p_ref_ = false;
        ref_mode_ = RefMode::NONE;
        zero_msgs_sent_++;
        if (zero_msgs_sent_ <= 5) {
          publish_stop_for_active_mode();
        } else {
          input_source_ = InputSource::NONE;
          servo_timer_->cancel();
        }
        return;
      }

      const bool use_dls = kdl_ready_ && has_joint_state_;
      const std::string desired_mode = use_dls ? "JOINT_JOG" : "TWIST";
      if (active_mode_ != desired_mode) {
        request_mode_switch(desired_mode);
        return;
      }

      zero_msgs_sent_ = 0;
      if (use_dls) {
        servo_jog_pub_->publish(compute_weighted_dls_jog());
      } else {
        geometry_msgs::msg::TwistStamped stamped_msg;
        stamped_msg.header.stamp = this->now();
        stamped_msg.header.frame_id = current_frame_id_;
        stamped_msg.twist = current_twist_;
        servo_twist_pub_->publish(stamped_msg);
      }
      return;
    }

    if (input_source_ == InputSource::DIRECT_JOG) {
      if (is_jog_zero()) {
        zero_msgs_sent_++;
        if (zero_msgs_sent_ <= 5) {
          publish_stop_for_active_mode();
        } else {
          input_source_ = InputSource::NONE;
          servo_timer_->cancel();
        }
        return;
      }

      if (active_mode_ != "JOINT_JOG") {
        request_mode_switch("JOINT_JOG");
        return;
      }
      zero_msgs_sent_ = 0;
      current_jog_.header.stamp = this->now();
      servo_jog_pub_->publish(current_jog_);
      return;
    }

    if (!switch_in_progress_) {
      servo_timer_->cancel();
    }
  }

  void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string new_mode = msg->data;
    std::transform(new_mode.begin(), new_mode.end(), new_mode.begin(),
                   [](unsigned char c) { return std::toupper(c); });
    if ((new_mode == "MANUAL" || new_mode == "AUTO") && system_mode_ != new_mode) {
      system_mode_ = new_mode;
      RCLCPP_INFO(this->get_logger(), "System mode switched to: %s", system_mode_.c_str());
      if (system_mode_ == "AUTO") {
        zero_msgs_sent_ = 0;
        current_twist_ = geometry_msgs::msg::Twist();
        current_jog_ = control_msgs::msg::JointJog();
        wake_servo_timer();
      }
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (system_mode_ == "AUTO") {
      return;
    }
    last_cmd_time_ = std::chrono::steady_clock::now();
    current_twist_ = msg->twist;
    current_jog_ = control_msgs::msg::JointJog();
    if (!msg->header.frame_id.empty()) {
      current_frame_id_ = msg->header.frame_id;
    }
    input_source_ = InputSource::TWIST_DLS;
    if (!is_twist_zero()) {
      request_mode_switch((kdl_ready_ && has_joint_state_) ? "JOINT_JOG" : "TWIST");
    }
    zero_msgs_sent_ = 0;
    wake_servo_timer();
  }

  void cmd_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg) {
    if (system_mode_ == "AUTO") {
      return;
    }
    last_cmd_time_ = std::chrono::steady_clock::now();
    current_jog_ = *msg;
    current_twist_ = geometry_msgs::msg::Twist();
    has_p_ref_ = false;
    ref_mode_ = RefMode::NONE;
    input_source_ = InputSource::DIRECT_JOG;
    if (!is_jog_zero()) {
      request_mode_switch("JOINT_JOG");
    }
    zero_msgs_sent_ = 0;
    wake_servo_timer();
  }

  void unfold_callback(const std_msgs::msg::Empty::SharedPtr) {
    if (system_mode_ == "AUTO") {
      RCLCPP_WARN(this->get_logger(), "Unfold ignored: system is in AUTO mode.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Unfolding robot to 'stand' pose (J4 = -45 deg)...");
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp.sec = 0;
    traj.header.stamp.nanosec = 0;
    traj.joint_names = {
      "arm_joint_0", "arm_joint_1", "arm_joint_2",
      "arm_joint_3", "arm_joint_4", "arm_joint_5"
    };
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 1.0, -1.0, 0.0, -0.785398, 0.0};
    point.time_from_start.sec = 3;
    point.time_from_start.nanosec = 0;
    traj.points.push_back(point);
    traj_pub_->publish(traj);
  }

  void fold_callback(const std_msgs::msg::Empty::SharedPtr) {
    if (system_mode_ == "AUTO") {
      RCLCPP_WARN(this->get_logger(), "Fold ignored: system is in AUTO mode.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Folding robot to 'home' pose (all 0)...");
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp.sec = 0;
    traj.header.stamp.nanosec = 0;
    traj.joint_names = {
      "arm_joint_0", "arm_joint_1", "arm_joint_2",
      "arm_joint_3", "arm_joint_4", "arm_joint_5"
    };
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start.sec = 3;
    point.time_from_start.nanosec = 0;
    traj.points.push_back(point);
    traj_pub_->publish(traj);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopManagerCpp>());
  rclcpp::shutdown();
  return 0;
}
