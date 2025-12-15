#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class megoldas2 : public rclcpp::Node
{
public:
    megoldas2() : Node("megoldas2")
    {
        this->declare_parameter<bool>("debug", false);
        this->declare_parameter<double>("safety_radius", 0.63);
        this->declare_parameter<double>("max_throttle", 0.32);
        this->declare_parameter<double>("steering_sensitivity", 2.1);
        this->declare_parameter<bool>("is_running", true);

        debug_ = this->get_parameter("debug").as_bool();
        safety_radius_ = this->get_parameter("safety_radius").as_double();
        max_throttle_ = this->get_parameter("max_throttle").as_double();
        steering_sensitivity_ = this->get_parameter("steering_sensitivity").as_double();
        is_running_ = this->get_parameter("is_running").as_bool();

        param_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(&megoldas2::onParamChange, this, _1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&megoldas2::scanCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        debug_marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug_marker", 1);
        state_pub_ = this->create_publisher<std_msgs::msg::String>("control_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&megoldas2::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Az algoritmus elindult");
    }

private:
    rcl_interfaces::msg::SetParametersResult
    onParamChange(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            if (param.get_name() == "steering_sensitivity")
                steering_sensitivity_ = param.as_double();
            else if (param.get_name() == "safety_radius")
                safety_radius_ = param.as_double();
            else if (param.get_name() == "max_throttle")
                max_throttle_ = param.as_double();
            else if (param.get_name() == "debug")
                debug_ = param.as_bool();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<float> ranges = msg->ranges;

        for (auto &r : ranges)
        {
            if (std::isinf(r))
                r = msg->range_max;
        }

        double best_angle = findBestGap(
            ranges, msg->angle_min, msg->angle_increment);

        publishDriveCommand(best_angle);
    }

    double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double findBestGap(const std::vector<float> &ranges,
                       double angle_min,
                       double angle_increment)
    {
        std::vector<int> safe_indices;

        for (size_t i = 0; i < ranges.size(); i++)
        {
            double angle = normalizeAngle(angle_min + i * angle_increment);
            if (ranges[i] > safety_radius_ &&
                angle >= -M_PI / 2 && angle <= M_PI / 2)
            {
                safe_indices.push_back(i);
            }
        }

        if (safe_indices.empty())
            return 0.0;

        std::vector<std::pair<int, int>> gaps;
        int start = safe_indices.front();

        for (size_t i = 1; i < safe_indices.size(); i++)
        {
            if (safe_indices[i] - safe_indices[i - 1] > 1)
            {
                gaps.emplace_back(start, safe_indices[i - 1]);
                start = safe_indices[i];
            }
        }
        gaps.emplace_back(start, safe_indices.back());

        auto largest_gap = *std::max_element(
            gaps.begin(), gaps.end(),
            [](auto &a, auto &b)
            { return (a.second - a.first) < (b.second - b.first); });

        int mid_index = (largest_gap.first + largest_gap.second) / 2;
        double best_angle = normalizeAngle(angle_min + mid_index * angle_increment);

        publishDebugMarker(ranges[mid_index], best_angle);
        publishDebugText(mid_index, best_angle);

        return best_angle;
    }

    void publishDebugMarker(double distance, double angle)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom_combined";
        marker.header.stamp = this->now();
        marker.ns = "megoldas2";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = distance * std::cos(angle);
        marker.pose.position.y = distance * std::sin(angle);
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;

        marker.color.a = 1.0;
        marker.color.r = 0.2;
        marker.color.g = 0.6;
        marker.color.b = 0.4;

        visualization_msgs::msg::MarkerArray array;
        array.markers.push_back(marker);
        debug_marker_pub_->publish(array);
    }

    void publishDebugText(int mid_index, double angle)
    {
        std_msgs::msg::String msg;
        msg.data =
            "megoldas2\nmid_index: " + std::to_string(mid_index) +
            "\nbest_angle (deg): " + std::to_string(angle * 180.0 / M_PI) +
            "\nsteering_sensitivity: " + std::to_string(steering_sensitivity_);
        state_pub_->publish(msg);
    }

    void publishDriveCommand(double angle)
    {
        if (!is_running_)
            return;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = max_throttle_;
        cmd.angular.z = angle * steering_sensitivity_;
        cmd_pub_->publish(cmd);
    }

    void timerCallback() {}

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    bool debug_;
    bool is_running_;
    double safety_radius_;
    double max_throttle_;
    double steering_sensitivity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<megoldas2>());
    rclcpp::shutdown();
    return 0;
}
