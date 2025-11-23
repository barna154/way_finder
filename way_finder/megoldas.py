
import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time

Csapatnev = "Racsecar"
Azonosito = "1"

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('megoldas')

        # --- Declare ROS2 parameters ---
        self.declare_parameter('Csapatnev', "Racsecar")
        self.declare_parameter('Azonosito', "1")
        self.declare_parameter('debug', False)
        self.declare_parameter('safety_radius', 0.63)
        self.declare_parameter('max_throttle', 0.32)
        self.declare_parameter('steering_sensitivity', 2.1)
        self.declare_parameter('max_steering_angle', 0.7)
        self.declare_parameter('is_running', True)

        # --- Read parameter values ---
        self.Csapatnev = self.get_parameter('Csapatnev').get_parameter_value().string_value
        self.Azonosito = self.get_parameter('Azonosito').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.safety_radius = self.get_parameter('safety_radius').get_parameter_value().double_value
        self.max_throttle = self.get_parameter('max_throttle').get_parameter_value().double_value
        self.steering_sensitivity = self.get_parameter('steering_sensitivity').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.is_running = self.get_parameter('is_running').get_parameter_value().bool_value

        # --- Dynamic parameter callback ---
        self.add_on_set_parameters_callback(self.on_param_change)

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_marker', 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pubst1 = self.create_publisher(String, 'control_state', 10)
        self.pubst2 = self.create_publisher(String, 'kozepiskola', 10)
        self.timer1 = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Follow the gap node has been started', once=True)
        self.get_logger().info(f'Debug mode: {self.debug}', once=True)

    # --- Dynamic ROS2 parameter handler ---
    def on_param_change(self, params):
        for param in params:
            if param.name == "steering_sensitivity" and param.type_ == Parameter.Type.DOUBLE:
                self.steering_sensitivity = param.value
                self.get_logger().info(f"Updated steering_sensitivity -> {self.steering_sensitivity}")
            elif param.name == "safety_radius" and param.type_ == Parameter.Type.DOUBLE:
                self.safety_radius = param.value
                self.get_logger().info(f"Updated safety_radius -> {self.safety_radius}")
            elif param.name == "max_throttle" and param.type_ == Parameter.Type.DOUBLE:
                self.max_throttle = param.value
                self.get_logger().info(f"Updated max_throttle -> {self.max_throttle}")
            elif param.name == "max_steering_angle" and param.type_ == Parameter.Type.DOUBLE:
                self.max_steering_angle = param.value
                self.get_logger().info(f"Updated max_steering_angle -> {self.max_steering_angle}")
            elif param.name == "debug" and param.type_ == Parameter.Type.BOOL:
                self.debug = param.value
                self.get_logger().info(f"Updated debug -> {self.debug}")
        return SetParametersResult(successful=True)

    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        # Replace 'inf' values with max range
        max_range = scan_data.range_max
        ranges[np.isinf(ranges)] = max_range

        # Only keep ranges beyond the safety radius
        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)

        # Find the largest safe gap and its midpoint angle
        best_direction = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)

        # Publish throttle and steering commands
        self.publish_drive_command(best_direction)

    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > self.safety_radius)[0]

        if len(safe_indices) == 0:
            return 0.0  # Keep driving straight

        def normalize_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        safe_indices = [idx for idx in safe_indices if -np.pi/2 <= normalize_angle(angle_min + idx * angle_increment) <= np.pi/2]
        if len(safe_indices) == 0:
            return 0.0

        gaps = []
        gap_start = safe_indices[0]
        for i in range(1, len(safe_indices)):
            if safe_indices[i] - safe_indices[i - 1] > 1:
                gaps.append((gap_start, safe_indices[i - 1]))
                gap_start = safe_indices[i]
        gaps.append((gap_start, safe_indices[-1]))

        largest_gap = max(gaps, key=lambda gap: gap[1] - gap[0])
        mid_index = (largest_gap[0] + largest_gap[1]) // 2
        best_angle = normalize_angle(angle_min + mid_index * angle_increment)
        best_angle = (best_angle + np.pi) % (2 * np.pi) - np.pi

        if self.debug:
            self.get_logger().info(f"Largest gap: Start {largest_gap[0]}, End {largest_gap[1]}, Best Angle: {np.degrees(best_angle):.1f}°")

        marker_center = Marker()
        marker_center.header.frame_id = "odom_combined"
        marker_center.ns = "megoldas"
        marker_center.id = 0
        marker_center.type = Marker.SPHERE
        marker_center.action = Marker.ADD
        marker_center.pose.position.x = ranges[mid_index] * np.cos(best_angle)
        marker_center.pose.position.y = ranges[mid_index] * np.sin(best_angle)
        marker_center.pose.position.z = 0.0
        marker_center.pose.orientation.w = 1.0
        marker_center.scale.x = 0.6
        marker_center.scale.y = 0.6
        marker_center.scale.z = 0.6
        marker_center.color.a = 1.0
        marker_center.color.r = 0.2
        marker_center.color.g = 0.6
        marker_center.color.b = 0.4
        marker_array = MarkerArray()
        marker_array.markers.append(marker_center)
        self.debug_marker_pub.publish(marker_array)

        messageS1 = String()
        messageS1.data = "megoldas"
        messageS1.data += f"\nmid_index: {mid_index:.0f}"
        messageS1.data += f"\nbest_angle (deg): {np.degrees(best_angle):.1f}"
        messageS1.data += f"\nsteering_sensitivity: {self.steering_sensitivity:.1f}"
        self.pubst1.publish(messageS1)
        return best_angle

    def publish_drive_command(self, best_angle):
        throttle_value = self.max_throttle
        steering_value = best_angle * self.steering_sensitivity
        twist_cmd = Twist()
        twist_cmd.linear.x = throttle_value
        twist_cmd.angular.z = steering_value
        if self.is_running:
            self.cmd_pub.publish(twist_cmd)
        self.pubst2.publish(String(data=f"{self.Csapatnev} ({self.Azonosito})"))

    def timer_callback(self):
        pass

    def shutdown_node(self):
        self.get_logger().info('Follow the gap shutdown procedure has been started')
        self.is_running = False
        try:
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.0
            self.cmd_pub.publish(twist_cmd)
            time.sleep(0.5)
            self.cmd_pub.publish(twist_cmd)
            self.get_logger().info('Follow the gap node has been stopped')
        except Exception as e:
            print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions(2))
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_node()
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception as e:
            print(f"An error occurred: {e}")
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()