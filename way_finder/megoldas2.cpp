#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
source ~/ros2_ws/install/setup.bash

using namespace std::chrono_literals;

class megoldas2 : public rclcpp::Node
{
public:
  megoldas2()
  : Node("megoldas2")
  {
    this->declare_parameter<std::string>("Csapatnev", "Racsecar");
    this->declare_parameter<std::string>("Azonosito", "1");
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<double>("safety_radius", 0.63);
    this->declare_parameter<double>("max_throttle", 0.32);
    this->declare_parameter<double>("steering_sensitivity", 2.1);
    this->declare_parameter<double>("max_steering_angle", 0.7);
    this->declare_parameter<bool>("is_running", true);

    Csapatnev_ = this->get_parameter("Csapatnev").as_string();
    Azonosito_ = this->get_parameter("Azonosito").as_string();
    debug_ = this->get_parameter("debug").as_bool();
    safety_radius_ = this->get_parameter("safety_radius").as_double();
    max_throttle_ = this->get_parameter("max_throttle").as_double();
    steering_sensitivity_ = this->get_parameter("steering_sensitivity").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    is_running_ = this->get_parameter("is_running").as_bool();

    // --- dynamic parameter callback ---
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&megoldas2::on_param_change, this, std::placeholders::_1)
    );

    // --- publishers & subscribers ---
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&megoldas2::scan_callback, this, std::placeholders::_1)
    );

    debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug_marker", 1);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    pubst1_ = this->create_publisher<std_msgs::msg::String>("control_state", 10);
    pubst2_ = this->create_publisher<std_msgs::msg::String>("kozepiskola", 10);

    // a timer (same as Python, empty callback)
    timer_ = this->create_wall_timer(100ms, std::bind(&megoldas2::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Follow the gap node has been started");
    RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_ ? "true" : "false");
  }

  ~megoldas2() override = default;

private:
  std::string Csapatnev_;
  std::string Azonosito_;
  bool debug_;
  double safety_radius_;
  double max_throttle_;
  double steering_sensitivity_;
  double max_steering_angle_;
  bool is_running_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubst1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubst2_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult on_param_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    for (const auto & p : params) {
      const std::string & name = p.get_name();
      if (name == "steering_sensitivity" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        steering_sensitivity_ = p.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated steering_sensitivity -> %f", steering_sensitivity_);
      } else if (name == "safety_radius" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        safety_radius_ = p.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated safety_radius -> %f", safety_radius_);
      } else if (name == "max_throttle" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        max_throttle_ = p.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated max_throttle -> %f", max_throttle_);
      } else if (name == "max_steering_angle" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        max_steering_angle_ = p.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated max_steering_angle -> %f", max_steering_angle_);
      } else if (name == "debug" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        debug_ = p.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated debug -> %s", debug_ ? "true" : "false");
      } else if (name == "is_running" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        is_running_ = p.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated is_running -> %s", is_running_ ? "true" : "false");
      }
    }
    return result;
  }
-
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    std::vector<double> ranges;
    ranges.reserve(scan_msg->ranges.size());
    double max_range = scan_msg->range_max;
    for (auto r : scan_msg->ranges) {
      if (!std::isfinite(r)) r = max_range;
      ranges.push_back(static_cast<double>(r));
    }

    std::vector<double> safe_ranges = ranges;
    for (auto & v : safe_ranges) {
      if (v <= safety_radius_) v = 0.0;
    }

    double best_direction = find_best_gap(safe_ranges, scan_msg->angle_min, scan_msg->angle_increment);
    publish_drive_command(best_direction);
  }
-
  double find_best_gap(const std::vector<double> & ranges, double angle_min, double angle_increment)
  {
    const double PI = M_PI;
    auto normalize_angle = [&](double angle) {
      double a = std::fmod(angle + PI, 2.0 * PI);
      if (a < 0) a += 2.0 * PI;
      return a - PI;
    };

    std::vector<int> safe_indices;
    safe_indices.reserve(ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (ranges[i] > safety_radius_) safe_indices.push_back(static_cast<int>(i));
    }

    if (safe_indices.empty()) {
      return 0.0;
    }

    std::vector<int> filtered;
    filtered.reserve(safe_indices.size());
    for (int idx : safe_indices) {
      double angle = angle_min + idx * angle_increment;
      double na = normalize_angle(angle);
      if (na >= -PI/2.0 && na <= PI/2.0) filtered.push_back(idx);
    }

    if (filtered.empty()) {
      return 0.0;
    }

    std::vector<std::pair<int,int>> gaps;
    int gap_start = filtered.front();
    int prev = filtered.front();
    for (size_t k = 1; k < filtered.size(); ++k) {
      int cur = filtered[k];
      if (cur - prev > 1) {
        gaps.emplace_back(gap_start, prev);
        gap_start = cur;
      }
      prev = cur;
    }
    gaps.emplace_back(gap_start, filtered.back());


    auto largest_gap = *std::max_element(
      gaps.begin(), gaps.end(),
      [](const std::pair<int,int> & a, const std::pair<int,int> & b){
        return (a.second - a.first) < (b.second - b.first);
      }
    );

    int mid_index = (largest_gap.first + largest_gap.second) / 2;
    double best_angle = normalize_angle(angle_min + mid_index * angle_increment);
    best_angle = normalize_angle(best_angle);

    if (debug_) {
      RCLCPP_INFO(this->get_logger(),
                  "Largest gap: Start %d, End %d, Best Angle: %.1f deg",
                  largest_gap.first, largest_gap.second, best_angle * 180.0 / M_PI);
    }

    visualization_msgs::msg::Marker marker_center;
    marker_center.header.frame_id = "odom_combined";
    marker_center.header.stamp = this->now();
    marker_center.ns = "megoldas2";
    marker_center.id = 0;
    marker_center.type = visualization_msgs::msg::Marker::SPHERE;
    marker_center.action = visualization_msgs::msg::Marker::ADD;
    double r = ranges[mid_index];
    marker_center.pose.position.x = r * std::cos(best_angle);
    marker_center.pose.position.y = r * std::sin(best_angle);
    marker_center.pose.position.z = 0.0;
    marker_center.pose.orientation.w = 1.0;
    marker_center.scale.x = 0.6;
    marker_center.scale.y = 0.6;
    marker_center.scale.z = 0.6;
    marker_center.color.a = 1.0;
    marker_center.color.r = 0.2f;
    marker_center.color.g = 0.6f;
    marker_center.color.b = 0.4f;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker_center);
    debug_marker_pub_->publish(marker_array);

    std_msgs::msg::String s1;
    std::ostringstream oss;
    oss << "megoldas2\nmid_index: " << mid_index
        << "\nbest_angle (deg): " << (best_angle * 180.0 / M_PI)
        << "\nsteering_sensitivity: " << steering_sensitivity_;
    s1.data = oss.str();
    pubst1_->publish(s1);

    return best_angle;
  }

  void publish_drive_command(double best_angle)
  {
    double throttle_value = max_throttle_;
    double steering_value = best_angle * steering_sensitivity_;

    if (std::fabs(steering_value) > max_steering_angle_) {
      steering_value = (steering_value > 0 ? 1.0 : -1.0) * max_steering_angle_;
    }

    geometry_msgs::msg::Twist twist_cmd;
    twist_cmd.linear.x = throttle_value;
    twist_cmd.angular.z = steering_value;

    if (is_running_) {
      cmd_pub_->publish(twist_cmd);
    }

    std_msgs::msg::String s2;
    s2.data = Csapatnev_ + " (" + Azonosito_ + ")";
    pubst2_->publish(s2);
  }

  void timer_callback()
  {
  }

public:
  void shutdown_node()
  {
    RCLCPP_INFO(this->get_logger(), "Follow the gap shutdown procedure has been started");
    is_running_ = false;
    try {
      geometry_msgs::msg::Twist twist_cmd;
      twist_cmd.linear.x = 0.0;
      twist_cmd.angular.z = 0.0;
      cmd_pub_->publish(twist_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      cmd_pub_->publish(twist_cmd);
      RCLCPP_INFO(this->get_logger(), "Follow the gap node has been stopped");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "An error occurred: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<megoldas2>();
  try {
    rclcpp::spin(node);
  } catch (...) {
  }
  try {
    node->shutdown_node();
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error during shutdown: %s", e.what());
  }
  return 0;
}
