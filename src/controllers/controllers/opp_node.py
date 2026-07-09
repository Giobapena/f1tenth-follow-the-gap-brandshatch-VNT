import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class OppFollowGap(Node):
    def __init__(self):
        super().__init__('opp_reactive_node')
        self.max_range = 8.0
        self.bubble_radius = 0.5
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
        self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
        self.start_x = None
        self.start_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.has_left = False
        self.get_logger().info("Oponente iniciado")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.lap_start_time = time.time()
            self.get_logger().info(f"Salida registrada: x={x:.2f}, y={y:.2f}")
            return
        dist = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
        if not self.has_left and dist > 3.0:
            self.has_left = True
        if self.has_left and dist < 1.5:
            lap_time = time.time() - self.lap_start_time
            self.lap_count += 1
            self.get_logger().info(f"OPP VUELTA {self.lap_count} - Tiempo: {lap_time:.2f} seg")
            self.lap_start_time = time.time()
            self.has_left = False

    def preprocess_lidar(self, ranges):
        proc = np.array(ranges)
        proc = np.nan_to_num(proc, nan=0.0, posinf=self.max_range, neginf=0.0)
        proc = np.clip(proc, 0, self.max_range)
        return proc

    def extend_disparities(self, ranges, angle_increment):
        ranges = ranges.copy()
        car_half_width = 0.25
        disparity_threshold = 0.25
        n = len(ranges)
        for i in range(n - 1):
            diff = ranges[i + 1] - ranges[i]
            if abs(diff) > disparity_threshold:
                near_value = min(ranges[i], ranges[i + 1])
                if near_value <= 0:
                    continue
                angle_to_cover = np.arctan2(car_half_width, near_value)
                num_indices = int(angle_to_cover / (angle_increment + 1e-6))
                if diff > 0:
                    end = min(n, i + 1 + num_indices)
                    for j in range(i + 1, end):
                        ranges[j] = min(ranges[j], near_value)
                else:
                    start = max(0, i - num_indices)
                    for j in range(start, i + 1):
                        ranges[j] = min(ranges[j], near_value)
        return ranges

    def limit_fov(self, ranges, angle_min, angle_increment, fov_deg=85):
        fov_limit = np.deg2rad(fov_deg)
        min_index = int((-fov_limit - angle_min) / angle_increment)
        max_index = int((fov_limit - angle_min) / angle_increment)
        min_index = max(0, min_index)
        max_index = min(len(ranges) - 1, max_index)
        ranges[:min_index] = 0
        ranges[max_index:] = 0
        return ranges

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        if slices is None:
            return 0, len(free_space_ranges) - 1
        if not isinstance(slices, list):
            slices = [slices]
        max_len = 0
        best_slice = slices[0]
        for s in slices:
            if s.stop - s.start > max_len:
                max_len = s.stop - s.start
                best_slice = s
        return best_slice.start, best_slice.stop - 1

    def find_best_point(self, start_i, end_i, ranges):
        gap_ranges = ranges[start_i:end_i + 1]
        if len(gap_ranges) == 0:
            return start_i
        farthest_index = np.argmax(gap_ranges) + start_i
        center_index = (start_i + end_i) // 2
        best_index = int(0.5 * center_index + 0.5 * farthest_index)
        return max(start_i, min(end_i, best_index))

    def lidar_callback(self, data):
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        closest_index = np.argmin(proc_ranges)
        radius_indices = int(self.bubble_radius / (data.angle_increment + 1e-6))
        min_idx = max(0, closest_index - radius_indices)
        max_idx = min(len(proc_ranges) - 1, closest_index + radius_indices)
        proc_ranges[min_idx:max_idx + 1] = 0
        proc_ranges = self.extend_disparities(proc_ranges, data.angle_increment)
        proc_ranges = self.limit_fov(proc_ranges, data.angle_min, data.angle_increment, fov_deg=85)
        start_i, end_i = self.find_max_gap(proc_ranges)
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)
        angle = data.angle_min + best_point_index * data.angle_increment
        angle = np.clip(angle, -np.deg2rad(34), np.deg2rad(34))
        gap_width_deg = np.rad2deg((end_i - start_i) * data.angle_increment)
        gap_depth = proc_ranges[best_point_index]
        is_straight = (gap_width_deg > 50.0) and (gap_depth > 4.0) and (abs(angle) < np.deg2rad(5))
        # Oponente mas lento que el principal (7.0)
        if is_straight:
            speed = 3.5
        else:
            angle_factor = max(0.0, 1.0 - (abs(angle) / 0.7))
            speed = 0.8 + angle_factor * (2.5 - 0.8)
        speed = np.clip(speed, 0.8, 3.5)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Oponente iniciado")
    opp_node = OppFollowGap()
    rclpy.spin(opp_node)
    opp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
