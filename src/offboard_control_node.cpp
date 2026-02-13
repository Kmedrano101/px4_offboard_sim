#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

    if (control_mode_ == ControlMode::POSITION) {
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
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    }

    if (arm_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      state_ = FlightState::IDLE;
      arm_command_ = false;
      control_mode_ = ControlMode::POSITION;
      land_mode_sent_ = false;
      RCLCPP_INFO(get_logger(), "Landed and disarmed — drone off");
    }

    if (elapsed > landing_timeout_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
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
    } else if (control_mode_ == ControlMode::POSITION) {
      msg.position = true;
      msg.velocity = false;
    } else {
      msg.position = false;
      msg.velocity = true;
    }
    msg.acceleration = false;
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

  enum class ControlMode { POSITION, VELOCITY };

  // ── Subscriptions ───────────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;

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
