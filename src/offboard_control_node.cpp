#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <chrono>
#include <limits>

namespace px4_offboard_sim {

enum class FlightState {
  IDLE,
  STREAMING,
  SWITCH_TO_OFFBOARD,
  ARMING,
  TAKEOFF,
  HOVER,
  LANDING,
};

static const char* state_name(FlightState s) {
  switch (s) {
    case FlightState::IDLE:               return "IDLE";
    case FlightState::STREAMING:          return "STREAMING";
    case FlightState::SWITCH_TO_OFFBOARD: return "SWITCH_TO_OFFBOARD";
    case FlightState::ARMING:             return "ARMING";
    case FlightState::TAKEOFF:            return "TAKEOFF";
    case FlightState::HOVER:              return "HOVER";
    case FlightState::LANDING:            return "LANDING";
  }
  return "UNKNOWN";
}

class OffboardControlNode : public rclcpp::Node {
public:
  explicit OffboardControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("offboard_control", options)
  {
    // ── Declare parameters ──────────────────────────────────
    this->declare_parameter("takeoff_altitude", 3.0);
    this->declare_parameter("landing_descent_speed", 0.5);
    this->declare_parameter("position_switch_delay", 0.5);
    this->declare_parameter("velocity_threshold", 0.02);
    this->declare_parameter("min_safe_altitude", 1.5);
    this->declare_parameter("streaming_threshold", 50);
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("landing_timeout", 30.0);

    this->declare_parameter("topics.vehicle_status", "/fmu/out/vehicle_status_v1");
    this->declare_parameter("topics.vehicle_attitude", "/fmu/out/vehicle_attitude");
    this->declare_parameter("topics.vehicle_local_position", "/fmu/out/vehicle_local_position_v1");
    this->declare_parameter("topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
    this->declare_parameter("topics.trajectory_setpoint", "/fmu/in/trajectory_setpoint");
    this->declare_parameter("topics.vehicle_command", "/fmu/in/vehicle_command");
    this->declare_parameter("topics.cmd_vel", "/px4_offboard_sim/offboard_control/cmd_vel");
    this->declare_parameter("topics.arm", "/px4_offboard_sim/offboard_control/arm");
    this->declare_parameter("topics.target_pose", "/px4_offboard_sim/offboard_control/target_pose");
    this->declare_parameter("topics.mpc_setpoint", "/px4_offboard_sim/offboard_control/mpc_setpoint");
    this->declare_parameter("planner_timeout", 2.0);
    this->declare_parameter("planner_goal_threshold", 0.5);
    this->declare_parameter("planner_goal_hold_time", 3.0);

    // ── Read parameters ─────────────────────────────────────
    takeoff_altitude_ned_ = -std::abs(this->get_parameter("takeoff_altitude").as_double());
    landing_descent_speed_ = this->get_parameter("landing_descent_speed").as_double();
    position_switch_delay_ = this->get_parameter("position_switch_delay").as_double();
    velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
    min_safe_altitude_ned_ = -std::abs(this->get_parameter("min_safe_altitude").as_double());
    streaming_threshold_ = this->get_parameter("streaming_threshold").as_int();
    landing_timeout_ = this->get_parameter("landing_timeout").as_double();
    double control_rate = this->get_parameter("control_rate").as_double();
    control_dt_ = 1.0 / control_rate;

    auto t_status = this->get_parameter("topics.vehicle_status").as_string();
    auto t_attitude = this->get_parameter("topics.vehicle_attitude").as_string();
    auto t_local_pos = this->get_parameter("topics.vehicle_local_position").as_string();
    auto t_offboard = this->get_parameter("topics.offboard_control_mode").as_string();
    auto t_traj = this->get_parameter("topics.trajectory_setpoint").as_string();
    auto t_cmd = this->get_parameter("topics.vehicle_command").as_string();
    auto t_vel = this->get_parameter("topics.cmd_vel").as_string();
    auto t_arm = this->get_parameter("topics.arm").as_string();
    auto t_target_pose = this->get_parameter("topics.target_pose").as_string();
    auto t_mpc_setpoint = this->get_parameter("topics.mpc_setpoint").as_string();
    planner_timeout_ = this->get_parameter("planner_timeout").as_double();
    planner_goal_threshold_ = static_cast<float>(this->get_parameter("planner_goal_threshold").as_double());
    planner_goal_hold_time_ = this->get_parameter("planner_goal_hold_time").as_double();

    // ── QoS for PX4 (BEST_EFFORT, VOLATILE, depth 1) ───────
    auto px4_qos = rclcpp::QoS(1)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    // ── Subscriptions ───────────────────────────────────────
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      t_status, px4_qos,
      std::bind(&OffboardControlNode::vehicle_status_cb, this, std::placeholders::_1));

    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      t_attitude, px4_qos,
      std::bind(&OffboardControlNode::attitude_cb, this, std::placeholders::_1));

    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      t_local_pos, px4_qos,
      std::bind(&OffboardControlNode::local_position_cb, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      t_vel, rclcpp::QoS(10),
      std::bind(&OffboardControlNode::cmd_vel_cb, this, std::placeholders::_1));

    arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      t_arm, rclcpp::QoS(10),
      std::bind(&OffboardControlNode::arm_cb, this, std::placeholders::_1));

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      t_target_pose, rclcpp::QoS(10),
      std::bind(&OffboardControlNode::target_pose_cb, this, std::placeholders::_1));

    mpc_setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      t_mpc_setpoint, px4_qos,
      std::bind(&OffboardControlNode::mpc_setpoint_cb, this, std::placeholders::_1));

    // ── Publishers ──────────────────────────────────────────
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      t_offboard, px4_qos);
    trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      t_traj, px4_qos);
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      t_cmd, px4_qos);

    // ── Control loop timer ──────────────────────────────────
    int period_ms = static_cast<int>(1000.0 / control_rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&OffboardControlNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "Offboard control node initialized (%.0f Hz)", control_rate);
    RCLCPP_INFO(get_logger(), "Takeoff altitude: %.1f m, landing speed: %.1f m/s",
                -takeoff_altitude_ned_, landing_descent_speed_);
    RCLCPP_INFO(get_logger(), "Waiting for arm command on: %s", t_arm.c_str());
  }

private:
  // ── Subscription callbacks ──────────────────────────────────

  void vehicle_status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    if (msg->nav_state != nav_state_)
      RCLCPP_INFO(get_logger(), "NAV_STATE: %u", msg->nav_state);
    if (msg->arming_state != arm_state_)
      RCLCPP_INFO(get_logger(), "ARM_STATE: %u", msg->arming_state);
    if (msg->failsafe != failsafe_)
      RCLCPP_INFO(get_logger(), "FAILSAFE: %s", msg->failsafe ? "true" : "false");
    if (msg->pre_flight_checks_pass != flight_check_)
      RCLCPP_INFO(get_logger(), "FLIGHT_CHECK: %s", msg->pre_flight_checks_pass ? "PASS" : "FAIL");

    nav_state_ = msg->nav_state;
    arm_state_ = msg->arming_state;
    failsafe_ = msg->failsafe;
    flight_check_ = msg->pre_flight_checks_pass;
  }

  void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    const auto& q = msg->q;
    true_yaw_ = std::atan2(
      2.0f * (q[0] * q[3] + q[1] * q[2]),
      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  }

  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_pos_[0] = msg->x;
    current_pos_[1] = msg->y;
    current_pos_[2] = msg->z;
  }

  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    velocity_frd_[0] = msg->linear.x;
    velocity_frd_[1] = -msg->linear.y;
    velocity_frd_[2] = -msg->linear.z;
    yaw_rate_ = -static_cast<float>(msg->angular.z);
  }

  void arm_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    arm_command_ = msg->data;
    RCLCPP_INFO(get_logger(), "Arm command: %s", arm_command_ ? "ARM" : "DISARM");
  }

  /// Planner target pose callback (ENU frame from px4_super_bridge)
  /// Converts ENU position to NED for PX4 and activates TRAJECTORY mode.
  /// On first activation, calibrates frame offset between planner frame
  /// (FAST-LIO camera_init) and PX4 NED frame.
  void target_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (state_ != FlightState::HOVER) return;

    // MPC takes priority over bridge when active
    if (control_mode_ == ControlMode::MPC_TRAJECTORY) return;

    // ENU→NED: ned_x=enu_y, ned_y=enu_x, ned_z=-enu_z
    float raw_ned[3];
    raw_ned[0] = static_cast<float>(msg->pose.position.y);   // North
    raw_ned[1] = static_cast<float>(msg->pose.position.x);   // East
    raw_ned[2] = static_cast<float>(-msg->pose.position.z);  // Down

    // Calibrate frame offset on first planner command
    if (!planner_frame_calibrated_) {
      for (int i = 0; i < 3; i++)
        planner_offset_ned_[i] = current_pos_[i] - raw_ned[i];
      planner_frame_calibrated_ = true;
      RCLCPP_INFO(get_logger(),
        "Planner frame calibrated: offset NED = (%.2f, %.2f, %.2f)",
        planner_offset_ned_[0], planner_offset_ned_[1], planner_offset_ned_[2]);
    }

    // Apply frame offset
    float new_pos[3];
    new_pos[0] = raw_ned[0] + planner_offset_ned_[0];
    new_pos[1] = raw_ned[1] + planner_offset_ned_[1];
    new_pos[2] = raw_ned[2] + planner_offset_ned_[2];

    // If we already reached the goal, only re-enter TRAJECTORY if the
    // planner is commanding a significantly different position (new goal)
    if (planner_goal_reached_) {
      float dx = new_pos[0] - target_pos_[0];
      float dy = new_pos[1] - target_pos_[1];
      float dz = new_pos[2] - target_pos_[2];
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < planner_goal_threshold_ * 2.0f) {
        // Still near the reached goal, ignore
        planner_last_time_ = now_sec();
        return;
      }
      // New goal detected, reset
      planner_goal_reached_ = false;
      RCLCPP_INFO(get_logger(), "New planner goal detected (%.1fm from hold pos)", dist);
    }

    planner_pos_[0] = new_pos[0];
    planner_pos_[1] = new_pos[1];
    planner_pos_[2] = new_pos[2];

    // Extract yaw from quaternion (ENU) and convert to NED
    double siny = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                          msg->pose.orientation.x * msg->pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                                 msg->pose.orientation.z * msg->pose.orientation.z);
    float enu_yaw = static_cast<float>(std::atan2(siny, cosy));

    // ENU yaw (0=East, CCW+) → NED yaw (0=North, CW+): ned_yaw = pi/2 - enu_yaw
    float ned_yaw = static_cast<float>(M_PI_2) - enu_yaw;
    // Wrap to [-pi, pi]
    while (ned_yaw > M_PI) ned_yaw -= 2.0f * M_PI;
    while (ned_yaw < -M_PI) ned_yaw += 2.0f * M_PI;
    planner_yaw_ = ned_yaw;

    planner_last_time_ = now_sec();

    if (control_mode_ != ControlMode::TRAJECTORY) {
      control_mode_ = ControlMode::TRAJECTORY;
      planner_near_goal_start_ = 0.0;
      RCLCPP_INFO(get_logger(), "TRAJECTORY mode: planner active");
    }
  }

  /// MPC trajectory setpoint callback (already in NED frame from px4_mpc_controller)
  void mpc_setpoint_cb(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
  {
    if (state_ != FlightState::HOVER) return;

    // MPC takes priority — if bridge TRAJECTORY mode is active, override it
    mpc_pos_[0] = msg->position[0];
    mpc_pos_[1] = msg->position[1];
    mpc_pos_[2] = msg->position[2];
    mpc_vel_[0] = msg->velocity[0];
    mpc_vel_[1] = msg->velocity[1];
    mpc_vel_[2] = msg->velocity[2];
    mpc_acc_[0] = msg->acceleration[0];
    mpc_acc_[1] = msg->acceleration[1];
    mpc_acc_[2] = msg->acceleration[2];
    mpc_yaw_ = msg->yaw;
    mpc_yawspeed_ = msg->yawspeed;
    mpc_last_time_ = now_sec();

    if (control_mode_ != ControlMode::MPC_TRAJECTORY) {
      control_mode_ = ControlMode::MPC_TRAJECTORY;
      RCLCPP_INFO(get_logger(), "MPC_TRAJECTORY mode: MPC controller active");
    }
  }

  // ── Main control loop ───────────────────────────────────────

  void control_loop()
  {
    publish_offboard_mode();
    publish_trajectory_setpoint();

    FlightState prev = state_;

    switch (state_) {
      case FlightState::IDLE:
        handle_idle();
        break;
      case FlightState::STREAMING:
        handle_streaming();
        break;
      case FlightState::SWITCH_TO_OFFBOARD:
        handle_switch_to_offboard();
        break;
      case FlightState::ARMING:
        handle_arming();
        break;
      case FlightState::TAKEOFF:
        handle_takeoff();
        break;
      case FlightState::HOVER:
        handle_hover();
        break;
      case FlightState::LANDING:
        handle_landing();
        break;
    }

    if (state_ != prev)
      RCLCPP_INFO(get_logger(), "State: %s -> %s", state_name(prev), state_name(state_));
  }

  // ── State handlers ──────────────────────────────────────────

  void handle_idle()
  {
    setpoint_counter_ = 0;
    control_mode_ = ControlMode::POSITION;
    if (arm_command_ && flight_check_) {
      state_ = FlightState::STREAMING;
      RCLCPP_INFO(get_logger(), "Starting setpoint streaming...");
    }
  }

  void handle_streaming()
  {
    setpoint_counter_++;
    if (!flight_check_ || !arm_command_) {
      state_ = FlightState::IDLE;
    } else if (setpoint_counter_ >= streaming_threshold_) {
      state_ = FlightState::SWITCH_TO_OFFBOARD;
      RCLCPP_INFO(get_logger(), "Streamed %d setpoints, switching to offboard...", setpoint_counter_);
    }
  }

  void handle_switch_to_offboard()
  {
    if (!flight_check_ || !arm_command_) {
      state_ = FlightState::IDLE;
      return;
    }
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);

    if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      state_ = FlightState::ARMING;
      RCLCPP_INFO(get_logger(), "Offboard mode active, arming...");
    }
  }

  void handle_arming()
  {
    if (!flight_check_) {
      state_ = FlightState::IDLE;
      return;
    }
    if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      target_pos_[0] = current_pos_[0];
      target_pos_[1] = current_pos_[1];
      target_pos_[2] = takeoff_altitude_ned_;
      target_yaw_ = true_yaw_;
      takeoff_time_ = now_sec();
      state_ = FlightState::TAKEOFF;
      RCLCPP_INFO(get_logger(), "Armed! Taking off to %.1fm, yaw=%.1f deg",
                  -takeoff_altitude_ned_, true_yaw_ * 180.0f / M_PI);
    } else {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    }
  }

  void handle_takeoff()
  {
    float alt_error = std::abs(current_pos_[2] - takeoff_altitude_ned_);
    double elapsed = now_sec() - takeoff_time_;

    if (alt_error < 0.5f) {
      state_ = FlightState::HOVER;
      RCLCPP_INFO(get_logger(), "Reached target altitude (%.2fm)", -current_pos_[2]);
    } else if (elapsed > 10.0) {
      state_ = FlightState::HOVER;
      RCLCPP_WARN(get_logger(), "Takeoff timeout at %.2fm (target %.1fm)",
                  -current_pos_[2], -takeoff_altitude_ned_);
    }

    if (arm_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED || failsafe_) {
      state_ = FlightState::IDLE;
      arm_command_ = false;
    } else if (!arm_command_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    }
  }

  void handle_hover()
  {
    float vel_mag = std::sqrt(
      velocity_frd_[0] * velocity_frd_[0] +
      velocity_frd_[1] * velocity_frd_[1] +
      velocity_frd_[2] * velocity_frd_[2]) + std::abs(yaw_rate_);

    if (control_mode_ == ControlMode::MPC_TRAJECTORY) {
      // MPC controller is active - check for timeout
      if (now_sec() - mpc_last_time_ > planner_timeout_) {
        target_pos_[0] = current_pos_[0];
        target_pos_[1] = current_pos_[1];
        target_pos_[2] = current_pos_[2];
        target_yaw_ = true_yaw_;
        control_mode_ = ControlMode::POSITION;
        mpc_goal_reached_ = false;
        mpc_near_goal_start_ = 0.0;
        RCLCPP_WARN(get_logger(), "MPC timeout — holding position");
      }
      // MPC controller handles its own goal detection and will stop publishing
      // when trajectory is complete, which triggers the timeout above.
    } else if (control_mode_ == ControlMode::TRAJECTORY) {
      // Planner is active - check for timeout
      if (now_sec() - planner_last_time_ > planner_timeout_) {
        target_pos_[0] = current_pos_[0];
        target_pos_[1] = current_pos_[1];
        target_pos_[2] = current_pos_[2];
        target_yaw_ = true_yaw_;
        control_mode_ = ControlMode::POSITION;
        planner_frame_calibrated_ = false;
        planner_goal_reached_ = false;
        planner_near_goal_start_ = 0.0;
        RCLCPP_WARN(get_logger(), "Planner timeout — holding position");
      } else {
        // Check if drone is close to planner setpoint (goal reached)
        float dx = current_pos_[0] - planner_pos_[0];
        float dy = current_pos_[1] - planner_pos_[1];
        float dz = current_pos_[2] - planner_pos_[2];
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < planner_goal_threshold_) {
          if (planner_near_goal_start_ == 0.0)
            planner_near_goal_start_ = now_sec();
          if (now_sec() - planner_near_goal_start_ >= planner_goal_hold_time_) {
            target_pos_[0] = planner_pos_[0];
            target_pos_[1] = planner_pos_[1];
            target_pos_[2] = planner_pos_[2];
            target_yaw_ = planner_yaw_;
            control_mode_ = ControlMode::POSITION;
            planner_goal_reached_ = true;
            planner_near_goal_start_ = 0.0;
            RCLCPP_INFO(get_logger(), "Goal reached — holding position at (%.2f, %.2f, %.2f)",
                        target_pos_[0], target_pos_[1], -target_pos_[2]);
          }
        } else {
          planner_near_goal_start_ = 0.0;
        }
      }
    } else if (control_mode_ == ControlMode::POSITION) {
      if (vel_mag > velocity_threshold_) {
        control_mode_ = ControlMode::VELOCITY;
        velocity_idle_start_ = 0.0;
        RCLCPP_INFO(get_logger(), "Switching to VELOCITY mode");
      }
    } else {
      if (vel_mag > velocity_threshold_) {
        velocity_idle_start_ = 0.0;
      } else {
        if (velocity_idle_start_ == 0.0)
          velocity_idle_start_ = now_sec();

        if (now_sec() - velocity_idle_start_ >= position_switch_delay_) {
          target_pos_[0] = current_pos_[0];
          target_pos_[1] = current_pos_[1];
          target_pos_[2] = (current_pos_[2] > min_safe_altitude_ned_)
            ? min_safe_altitude_ned_ : current_pos_[2];
          target_yaw_ = true_yaw_;
          control_mode_ = ControlMode::POSITION;
          RCLCPP_INFO(get_logger(), "POSITION hold: x=%.1f y=%.1f alt=%.1fm yaw=%.0f deg",
                      target_pos_[0], target_pos_[1], -target_pos_[2],
                      target_yaw_ * 180.0f / M_PI);
        }
      }
    }

    if (arm_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED || failsafe_) {
      state_ = FlightState::IDLE;
      arm_command_ = false;
      control_mode_ = ControlMode::POSITION;
    } else if (!arm_command_) {
      state_ = FlightState::LANDING;
      control_mode_ = ControlMode::POSITION;
      landing_time_ = now_sec();
      RCLCPP_INFO(get_logger(), "Landing from %.1fm at (%.1f, %.1f) — switching to AUTO_LAND",
                  -current_pos_[2], current_pos_[0], current_pos_[1]);
    }
  }

  void handle_landing()
  {
    double elapsed = now_sec() - landing_time_;

    if (!land_mode_sent_ || (static_cast<int>(elapsed) % 2 == 0 && elapsed - last_land_cmd_ > 1.0)) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 6.0f);
      land_mode_sent_ = true;
      last_land_cmd_ = elapsed;
    }

    float altitude_agl = -current_pos_[2];
    if (altitude_agl < 0.3f && elapsed > 2.0) {
      // Force disarm (param2=21196 bypasses PX4 land detector)
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
    }

    if (arm_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      state_ = FlightState::IDLE;
      arm_command_ = false;
      control_mode_ = ControlMode::POSITION;
      land_mode_sent_ = false;
      RCLCPP_INFO(get_logger(), "Landed and disarmed — drone off");
    }

    if (elapsed > landing_timeout_) {
      // Force disarm (param2=21196 bypasses PX4 land detector)
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
      RCLCPP_WARN(get_logger(), "Landing timeout — forcing disarm");
    }

    if (arm_command_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      state_ = FlightState::HOVER;
      control_mode_ = ControlMode::POSITION;
      land_mode_sent_ = false;
      target_pos_[0] = current_pos_[0];
      target_pos_[1] = current_pos_[1];
      target_pos_[2] = current_pos_[2];
      target_yaw_ = true_yaw_;
      RCLCPP_INFO(get_logger(), "Landing cancelled, returning to HOVER");
    }
  }

  // ── Publishing ──────────────────────────────────────────────

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp_us();

    if (state_ == FlightState::LANDING) {
      msg.position = true;
      msg.velocity = false;
      msg.acceleration = false;
    } else if (control_mode_ == ControlMode::MPC_TRAJECTORY) {
      msg.position = true;
      msg.velocity = true;
      msg.acceleration = true;  // Enable acceleration feedforward in PX4
    } else if (control_mode_ == ControlMode::POSITION || control_mode_ == ControlMode::TRAJECTORY) {
      msg.position = true;
      msg.velocity = false;
      msg.acceleration = false;
    } else {
      msg.position = false;
      msg.velocity = true;
      msg.acceleration = false;
    }
    msg.attitude = false;
    msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = timestamp_us();

    if (state_ == FlightState::LANDING) {
      msg.position[0] = target_pos_[0];
      msg.position[1] = target_pos_[1];
      msg.position[2] = target_pos_[2];
      msg.velocity = {NAN_F, NAN_F, NAN_F};
      msg.yaw = target_yaw_;
      msg.yawspeed = NAN_F;
    } else if (control_mode_ == ControlMode::MPC_TRAJECTORY) {
      msg.position[0] = mpc_pos_[0];
      msg.position[1] = mpc_pos_[1];
      msg.position[2] = mpc_pos_[2];
      msg.velocity[0] = mpc_vel_[0];
      msg.velocity[1] = mpc_vel_[1];
      msg.velocity[2] = mpc_vel_[2];
      msg.acceleration[0] = mpc_acc_[0];
      msg.acceleration[1] = mpc_acc_[1];
      msg.acceleration[2] = mpc_acc_[2];
      msg.yaw = mpc_yaw_;
      msg.yawspeed = mpc_yawspeed_;
      trajectory_pub_->publish(msg);
      return;  // Skip the common acceleration = NaN below
    } else if (control_mode_ == ControlMode::TRAJECTORY) {
      msg.position[0] = planner_pos_[0];
      msg.position[1] = planner_pos_[1];
      msg.position[2] = planner_pos_[2];
      msg.velocity = {NAN_F, NAN_F, NAN_F};
      msg.yaw = planner_yaw_;
      msg.yawspeed = NAN_F;
    } else if (control_mode_ == ControlMode::POSITION) {
      msg.position[0] = target_pos_[0];
      msg.position[1] = target_pos_[1];
      msg.position[2] = target_pos_[2];
      msg.velocity = {NAN_F, NAN_F, NAN_F};
      msg.yaw = target_yaw_;
      msg.yawspeed = NAN_F;
    } else {
      float cos_y = std::cos(true_yaw_);
      float sin_y = std::sin(true_yaw_);
      msg.velocity[0] = velocity_frd_[0] * cos_y - velocity_frd_[1] * sin_y;
      msg.velocity[1] = velocity_frd_[0] * sin_y + velocity_frd_[1] * cos_y;
      msg.velocity[2] = velocity_frd_[2];
      msg.position = {NAN_F, NAN_F, NAN_F};
      msg.yaw = NAN_F;
      msg.yawspeed = yaw_rate_;
    }

    msg.acceleration = {NAN_F, NAN_F, NAN_F};
    trajectory_pub_->publish(msg);
  }

  void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
                              float param3 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_cmd_pub_->publish(msg);
  }

  // ── Helpers ─────────────────────────────────────────────────

  uint64_t timestamp_us() const
  {
    return this->get_clock()->now().nanoseconds() / 1000;
  }

  double now_sec() const
  {
    return static_cast<double>(this->get_clock()->now().nanoseconds()) * 1e-9;
  }

  // ── Types ───────────────────────────────────────────────────

  enum class ControlMode { POSITION, VELOCITY, TRAJECTORY, MPC_TRAJECTORY };

  // ── Subscriptions ───────────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr mpc_setpoint_sub_;

  // ── Publishers ──────────────────────────────────────────────
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  // ── Timer ───────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Parameters ──────────────────────────────────────────────
  float takeoff_altitude_ned_{-3.0f};
  double landing_descent_speed_{0.5};
  double position_switch_delay_{0.5};
  float velocity_threshold_{0.02f};
  float min_safe_altitude_ned_{-1.5f};
  int streaming_threshold_{50};
  double landing_timeout_{30.0};
  double control_dt_{0.02};

  // ── PX4 state ───────────────────────────────────────────────
  uint8_t nav_state_{255};
  uint8_t arm_state_{0};
  bool failsafe_{false};
  bool flight_check_{false};

  // ── Control state ───────────────────────────────────────────
  FlightState state_{FlightState::IDLE};
  ControlMode control_mode_{ControlMode::POSITION};
  bool arm_command_{false};
  int setpoint_counter_{0};

  // ── Position / velocity ─────────────────────────────────────
  float current_pos_[3]{0.0f, 0.0f, 0.0f};
  float target_pos_[3]{0.0f, 0.0f, -3.0f};
  float true_yaw_{0.0f};
  float target_yaw_{0.0f};
  float velocity_frd_[3]{0.0f, 0.0f, 0.0f};
  float yaw_rate_{0.0f};

  // ── Planner (TRAJECTORY mode) ────────────────────────────────
  float planner_pos_[3]{0.0f, 0.0f, 0.0f};
  float planner_yaw_{0.0f};
  double planner_last_time_{0.0};
  double planner_timeout_{2.0};
  float planner_offset_ned_[3]{0.0f, 0.0f, 0.0f};
  bool planner_frame_calibrated_{false};
  bool planner_goal_reached_{false};
  float planner_goal_threshold_{0.5f};
  double planner_goal_hold_time_{3.0};
  double planner_near_goal_start_{0.0};

  // ── MPC (MPC_TRAJECTORY mode) ────────────────────────────────
  float mpc_pos_[3]{0.0f, 0.0f, 0.0f};
  float mpc_vel_[3]{0.0f, 0.0f, 0.0f};
  float mpc_acc_[3]{0.0f, 0.0f, 0.0f};
  float mpc_yaw_{0.0f};
  float mpc_yawspeed_{0.0f};
  double mpc_last_time_{0.0};
  bool mpc_goal_reached_{false};
  double mpc_near_goal_start_{0.0};

  // ── Timing ──────────────────────────────────────────────────
  double takeoff_time_{0.0};
  double landing_time_{0.0};
  double velocity_idle_start_{0.0};
  bool land_mode_sent_{false};
  double last_land_cmd_{0.0};
};

}  // namespace px4_offboard_sim

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_sim::OffboardControlNode>());
  rclcpp::shutdown();
  return 0;
}
