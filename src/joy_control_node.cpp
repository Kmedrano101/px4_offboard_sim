#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

namespace px4_offboard_sim {

// Global flag for clean shutdown from SIGINT
static std::atomic<bool> g_shutdown{false};
static struct termios g_original_settings;
static bool g_terminal_modified{false};

static void signal_handler(int /*sig*/)
{
  g_shutdown.store(true);
  if (g_terminal_modified) {
    tcsetattr(STDIN_FILENO, TCSADRAIN, &g_original_settings);
    g_terminal_modified = false;
  }
}

static constexpr const char* HELP_MSG =
  "\n"
  "Joy Control: keyboard + gamepad input for offboard velocity control\n"
  "Using Mode 2 RC layout:\n"
  "  W/S         : Throttle up/down\n"
  "  A/D         : Yaw left/right\n"
  "  Up/Down     : Pitch forward/backward\n"
  "  Left/Right  : Roll left/right\n"
  "  Q/E         : Increase/decrease speed\n"
  "  X           : Emergency stop\n"
  "  SPACE       : Toggle arm/disarm\n"
  "  Ctrl+C      : Exit\n"
  "\n";

class JoyControlNode : public rclcpp::Node {
public:
  explicit JoyControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("joy_control", options)
  {
    // ── Declare parameters ──────────────────────────────────
    this->declare_parameter("speed", 0.3);
    this->declare_parameter("turn", 0.2);
    this->declare_parameter("speed_step", 0.05);
    this->declare_parameter("turn_step", 0.025);
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("max_turn", 1.0);

    this->declare_parameter("ramp_rate", 0.15);
    this->declare_parameter("decay_rate", 0.92);
    this->declare_parameter("hold_duration", 0.3);
    this->declare_parameter("dead_zone", 0.005);

    this->declare_parameter("joy_axis_pitch", 4);
    this->declare_parameter("joy_axis_roll", 3);
    this->declare_parameter("joy_axis_throttle", 1);
    this->declare_parameter("joy_axis_yaw", 0);
    this->declare_parameter("joy_deadzone", 0.05);

    this->declare_parameter("joy_button_disarm", 0);
    this->declare_parameter("joy_button_arm", 2);

    this->declare_parameter("joy_axis_speed_preset", -1);
    this->declare_parameter("speed_slow", 0.15);
    this->declare_parameter("speed_normal", 0.3);
    this->declare_parameter("speed_fast", 0.6);
    this->declare_parameter("turn_slow", 0.1);
    this->declare_parameter("turn_normal", 0.2);
    this->declare_parameter("turn_fast", 0.4);

    this->declare_parameter("control_rate", 50.0);

    this->declare_parameter("topics.cmd_vel", "/px4_offboard_sim/offboard_control/cmd_vel");
    this->declare_parameter("topics.arm", "/px4_offboard_sim/offboard_control/arm");
    this->declare_parameter("topics.joy", "/px4_offboard_sim/joy");

    // ── Read parameters ─────────────────────────────────────
    speed_ = this->get_parameter("speed").as_double();
    turn_ = this->get_parameter("turn").as_double();
    speed_step_ = this->get_parameter("speed_step").as_double();
    turn_step_ = this->get_parameter("turn_step").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_turn_ = this->get_parameter("max_turn").as_double();

    ramp_rate_ = this->get_parameter("ramp_rate").as_double();
    decay_rate_ = this->get_parameter("decay_rate").as_double();
    hold_duration_ = this->get_parameter("hold_duration").as_double();
    dead_zone_ = this->get_parameter("dead_zone").as_double();

    joy_axis_pitch_ = this->get_parameter("joy_axis_pitch").as_int();
    joy_axis_roll_ = this->get_parameter("joy_axis_roll").as_int();
    joy_axis_throttle_ = this->get_parameter("joy_axis_throttle").as_int();
    joy_axis_yaw_ = this->get_parameter("joy_axis_yaw").as_int();
    joy_deadzone_ = this->get_parameter("joy_deadzone").as_double();

    joy_button_disarm_ = this->get_parameter("joy_button_disarm").as_int();
    joy_button_arm_ = this->get_parameter("joy_button_arm").as_int();

    joy_axis_speed_preset_ = this->get_parameter("joy_axis_speed_preset").as_int();
    speed_slow_ = this->get_parameter("speed_slow").as_double();
    speed_normal_ = this->get_parameter("speed_normal").as_double();
    speed_fast_ = this->get_parameter("speed_fast").as_double();
    turn_slow_ = this->get_parameter("turn_slow").as_double();
    turn_normal_ = this->get_parameter("turn_normal").as_double();
    turn_fast_ = this->get_parameter("turn_fast").as_double();

    double control_rate = this->get_parameter("control_rate").as_double();

    auto t_vel = this->get_parameter("topics.cmd_vel").as_string();
    auto t_arm = this->get_parameter("topics.arm").as_string();
    auto t_joy = this->get_parameter("topics.joy").as_string();

    // ── Publishers ──────────────────────────────────────────
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(t_vel, rclcpp::QoS(10));
    arm_pub_ = this->create_publisher<std_msgs::msg::Bool>(t_arm, rclcpp::QoS(10));

    // ── Joystick subscription ───────────────────────────────
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      t_joy, rclcpp::QoS(10),
      std::bind(&JoyControlNode::joy_callback, this, std::placeholders::_1));

    // ── Timer for continuous velocity publishing ────────────
    int period_ms = static_cast<int>(1000.0 / control_rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&JoyControlNode::timer_callback, this));

    // ── Save terminal settings ──────────────────────────────
    tcgetattr(STDIN_FILENO, &g_original_settings);

    RCLCPP_INFO(get_logger(), "Joy control node initialized (%.0f Hz)", control_rate);
    RCLCPP_INFO(get_logger(), "Velocity: %s | Arm: %s | Joy: %s",
                t_vel.c_str(), t_arm.c_str(), t_joy.c_str());
  }

  ~JoyControlNode()
  {
    restore_terminal();
  }

  bool has_terminal() const
  {
    return isatty(STDIN_FILENO) == 1;
  }

  void keyboard_loop()
  {
    if (!has_terminal()) {
      RCLCPP_INFO(get_logger(), "No interactive terminal detected — joystick-only mode");
      RCLCPP_INFO(get_logger(), "For keyboard control, run: ros2 run px4_offboard_sim joy_control_node "
                  "--ros-args --params-file "
                  "$(ros2 pkg prefix px4_offboard_sim)/share/px4_offboard_sim/config/joy_control.yaml");
      while (!g_shutdown.load() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      return;
    }

    printf("%s", HELP_MSG);
    printf("Speed: %.2f m/s | Turn: %.2f rad/s\n", speed_, turn_);

    set_raw_terminal();

    while (!g_shutdown.load() && rclcpp::ok()) {
      std::string key = read_key(50);

      if (key.empty()) continue;
      if (key == "\x03") break;

      if      (key == "w")       set_keyboard_target( 0,  0,  1,  0);
      else if (key == "s")       set_keyboard_target( 0,  0, -1,  0);
      else if (key == "a")       set_keyboard_target( 0,  0,  0,  1);
      else if (key == "d")       set_keyboard_target( 0,  0,  0, -1);
      else if (key == "\x1b[A")  set_keyboard_target( 1,  0,  0,  0);
      else if (key == "\x1b[B")  set_keyboard_target(-1,  0,  0,  0);
      else if (key == "\x1b[D")  set_keyboard_target( 0,  1,  0,  0);
      else if (key == "\x1b[C")  set_keyboard_target( 0, -1,  0,  0);
      else if (key == " ") {
        toggle_arm();
      }
      else if (key == "x") {
        stop_all();
      }
      else if (key == "q") {
        speed_ = std::min(speed_ + speed_step_, max_speed_);
        turn_ = std::min(turn_ + turn_step_, max_turn_);
        printf("\rSpeed: %.2f m/s | Turn: %.2f rad/s          \n", speed_, turn_);
      }
      else if (key == "e") {
        speed_ = std::max(speed_ - speed_step_, 0.05);
        turn_ = std::max(turn_ - turn_step_, 0.05);
        printf("\rSpeed: %.2f m/s | Turn: %.2f rad/s          \n", speed_, turn_);
      }
    }

    stop_all();
    publish_velocity();
    restore_terminal();
    printf("\nExiting...\n");
  }

private:
  // ── Terminal I/O ──────────────────────────────────────────

  void set_raw_terminal()
  {
    struct termios raw = g_original_settings;
    cfmakeraw(&raw);
    raw.c_oflag |= OPOST;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    g_terminal_modified = true;
  }

  void restore_terminal()
  {
    if (g_terminal_modified) {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &g_original_settings);
      g_terminal_modified = false;
    }
  }

  std::string read_key(int timeout_ms)
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) return "";

    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return "";

    if (c == '\x1b') {
      fd_set fds2;
      FD_ZERO(&fds2);
      FD_SET(STDIN_FILENO, &fds2);
      struct timeval tv2 = {0, 10000};

      if (select(STDIN_FILENO + 1, &fds2, nullptr, nullptr, &tv2) > 0) {
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) == 1 && read(STDIN_FILENO, &seq[1], 1) == 1) {
          return std::string(1, c) + std::string(1, seq[0]) + std::string(1, seq[1]);
        }
      }
      return std::string(1, c);
    }

    return std::string(1, c);
  }

  // ── Keyboard input ────────────────────────────────────────

  void set_keyboard_target(double x, double y, double z, double yaw)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    using_joystick_ = false;
    last_key_time_ = now_sec();
    target_x_ = x * speed_;
    target_y_ = y * speed_;
    target_z_ = z * speed_;
    target_yaw_ = yaw * turn_;

    printf("\rvel: x=%+.2f y=%+.2f z=%+.2f yaw=%+.2f | spd=%.2f  ",
           vel_x_, vel_y_, vel_z_, vel_yaw_, speed_);
    fflush(stdout);
  }

  void toggle_arm()
  {
    arm_toggle_ = !arm_toggle_;
    std_msgs::msg::Bool msg;
    msg.data = arm_toggle_;
    arm_pub_->publish(msg);
    printf("\r%s                    \n", arm_toggle_ ? "Armed" : "Disarmed");
  }

  void stop_all()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    target_x_ = target_y_ = target_z_ = target_yaw_ = 0.0;
    vel_x_ = vel_y_ = vel_z_ = vel_yaw_ = 0.0;
    printf("\rStopped                    \n");
  }

  // ── Joystick callback ─────────────────────────────────────

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    using_joystick_ = true;

    // Log joystick info on first message
    if (!joy_info_logged_) {
      joy_info_logged_ = true;
      RCLCPP_INFO(get_logger(),
        "Joystick connected: %zu axes, %zu buttons | speed_preset_axis=%d %s",
        msg->axes.size(), msg->buttons.size(),
        joy_axis_speed_preset_,
        (joy_axis_speed_preset_ >= 0 &&
         joy_axis_speed_preset_ < static_cast<int>(msg->axes.size()))
          ? "(ACTIVE)" : "(NOT AVAILABLE — check EdgeTX channel config)");
    }

    auto safe_axis = [&](int idx) -> double {
      if (idx < 0 || idx >= static_cast<int>(msg->axes.size())) return 0.0;
      double val = msg->axes[idx];
      return (std::abs(val) < joy_deadzone_) ? 0.0 : val;
    };

    target_x_   = -safe_axis(joy_axis_pitch_) * speed_;
    target_y_   = safe_axis(joy_axis_roll_) * speed_;
    target_z_   = -safe_axis(joy_axis_throttle_) * speed_;
    target_yaw_ = safe_axis(joy_axis_yaw_) * turn_;

    auto btn = [&](int idx) -> bool {
      return idx >= 0 && idx < static_cast<int>(msg->buttons.size()) && msg->buttons[idx] == 1;
    };

    bool combo_pressed = btn(joy_button_arm_) && btn(joy_button_disarm_);

    if (combo_pressed && !last_combo_btn_) {
      arm_toggle_ = !arm_toggle_;
      std_msgs::msg::Bool arm_msg;
      arm_msg.data = arm_toggle_;
      arm_pub_->publish(arm_msg);
      RCLCPP_INFO(get_logger(), "Joystick: %s", arm_toggle_ ? "ARMED" : "DISARMED");
    }

    last_combo_btn_ = combo_pressed;

    if (joy_axis_speed_preset_ >= 0 &&
        joy_axis_speed_preset_ < static_cast<int>(msg->axes.size())) {
      double sw = msg->axes[joy_axis_speed_preset_];
      double new_speed, new_turn;
      const char* label;
      if (sw > 0.5) {
        new_speed = speed_fast_; new_turn = turn_fast_; label = "FAST";
      } else if (sw < -0.5) {
        new_speed = speed_slow_; new_turn = turn_slow_; label = "SLOW";
      } else {
        new_speed = speed_normal_; new_turn = turn_normal_; label = "NORMAL";
      }
      if (std::abs(speed_ - new_speed) > 0.001) {
        speed_ = new_speed;
        turn_ = new_turn;
        RCLCPP_WARN(get_logger(), ">>> SPEED: %s — %.2f m/s, %.2f rad/s <<<",
                    label, speed_, turn_);
      }
    }
  }

  // ── Timer callback ────────────────────────────────────────

  void timer_callback()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);

      double time_since_key = now_sec() - last_key_time_;

      if (!using_joystick_ && time_since_key > hold_duration_) {
        target_x_ *= decay_rate_;
        target_y_ *= decay_rate_;
        target_z_ *= decay_rate_;
        target_yaw_ *= decay_rate_;

        if (std::abs(target_x_) < 0.01) target_x_ = 0.0;
        if (std::abs(target_y_) < 0.01) target_y_ = 0.0;
        if (std::abs(target_z_) < 0.01) target_z_ = 0.0;
        if (std::abs(target_yaw_) < 0.01) target_yaw_ = 0.0;
      }

      vel_x_ += (target_x_ - vel_x_) * ramp_rate_;
      vel_y_ += (target_y_ - vel_y_) * ramp_rate_;
      vel_z_ += (target_z_ - vel_z_) * ramp_rate_;
      vel_yaw_ += (target_yaw_ - vel_yaw_) * ramp_rate_;

      if (std::abs(vel_x_) < dead_zone_) vel_x_ = 0.0;
      if (std::abs(vel_y_) < dead_zone_) vel_y_ = 0.0;
      if (std::abs(vel_z_) < dead_zone_) vel_z_ = 0.0;
      if (std::abs(vel_yaw_) < dead_zone_) vel_yaw_ = 0.0;
    }

    publish_velocity();
  }

  void publish_velocity()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vel_x_;
    msg.linear.y = vel_y_;
    msg.linear.z = vel_z_;
    msg.angular.z = vel_yaw_;
    vel_pub_->publish(msg);
  }

  // ── Helpers ───────────────────────────────────────────────

  double now_sec() const
  {
    return static_cast<double>(
      std::chrono::steady_clock::now().time_since_epoch().count()) * 1e-9;
  }

  // ── Members ───────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double speed_{0.3};
  double turn_{0.2};
  double speed_step_{0.05};
  double turn_step_{0.025};
  double max_speed_{2.0};
  double max_turn_{1.0};

  double ramp_rate_{0.15};
  double decay_rate_{0.92};
  double hold_duration_{0.3};
  double dead_zone_{0.005};

  int joy_axis_pitch_{4};
  int joy_axis_roll_{3};
  int joy_axis_throttle_{1};
  int joy_axis_yaw_{0};
  double joy_deadzone_{0.05};
  int joy_button_disarm_{0};
  int joy_button_arm_{2};
  int joy_axis_speed_preset_{-1};
  double speed_slow_{0.15};
  double speed_normal_{0.3};
  double speed_fast_{0.6};
  double turn_slow_{0.1};
  double turn_normal_{0.2};
  double turn_fast_{0.4};

  std::mutex mtx_;
  bool using_joystick_{false};
  bool arm_toggle_{false};
  bool last_combo_btn_{false};
  bool joy_info_logged_{false};
  double last_key_time_{0.0};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_{0.0};

  double vel_x_{0.0};
  double vel_y_{0.0};
  double vel_z_{0.0};
  double vel_yaw_{0.0};
};

}  // namespace px4_offboard_sim

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, px4_offboard_sim::signal_handler);

  auto node = std::make_shared<px4_offboard_sim::JoyControlNode>();

  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });

  node->keyboard_loop();

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
