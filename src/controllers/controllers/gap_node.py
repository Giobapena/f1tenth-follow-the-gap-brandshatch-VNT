import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """
    Follow The Gap con:
    - Disparity extender, FOV limitado, velocidad adaptativa (controlador de manejo)
    - Contador de vueltas y cronometro por vuelta usando /ego_racecar/odom
    """

    def __init__(self):
        super().__init__('reactive_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        self.bubble_radius = 0.5
        self.max_range = 8.0
        self.safety_distance = 0.4

        self.scan_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)

        # --- Contador de vueltas y cronometro ---
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.start_x = None
        self.start_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.has_left_start_zone = False

        self.start_zone_radius = 1.5   # radio (metros) para considerar "en la linea de salida"
        self.leave_zone_radius = 3.0   # distancia minima que debe alejarse antes de contar vuelta nueva

        self.get_logger().info("Contador de vueltas y cronometro inicializados")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # La primera vez que recibimos odometria, guardamos la posicion de salida
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.lap_start_time = time.time()
            self.get_logger().info(
                f"Linea de salida registrada en x={x:.2f}, y={y:.2f}")
            return

        distance_to_start = math.sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)

        # Si el carro se alejo lo suficiente, marcamos que ya "salio" a dar la vuelta
        if not self.has_left_start_zone and distance_to_start > self.leave_zone_radius:
            self.has_left_start_zone = True

        # Si ya se habia alejado y ahora vuelve a estar cerca de la salida, contamos vuelta
        if self.has_left_start_zone and distance_to_start < self.start_zone_radius:
            lap_time = time.time() - self.lap_start_time
            self.lap_count += 1
            self.get_logger().info(
                f"VUELTA {self.lap_count} completada - Tiempo: {lap_time:.2f} segundos")

            # Reiniciamos el cronometro y el estado para la siguiente vuelta
            self.lap_start_time = time.time()
            self.has_left_start_zone = False

    def preprocess_lidar(self, ranges):
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0, self.max_range)
        return proc_ranges

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
            length = s.stop - s.start
            if length > max_len:
                max_len = length
                best_slice = s

        return best_slice.start, best_slice.stop - 1

    def find_best_point(self, start_i, end_i, ranges):
        gap_ranges = ranges[start_i:end_i + 1]
        if len(gap_ranges) == 0:
            return start_i

        farthest_index = np.argmax(gap_ranges) + start_i
        center_index = (start_i + end_i) // 2

        weight_center = 0.45
        weight_farthest = 0.55
        best_index = int(weight_center * center_index + weight_farthest * farthest_index)

        best_index = max(start_i, min(end_i, best_index))
        return best_index

    def lidar_callback(self, data):
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        proc_ranges = self.extend_disparities(proc_ranges, data.angle_increment)
        proc_ranges = self.limit_fov(proc_ranges, data.angle_min, data.angle_increment, fov_deg=85)

        closest_index = np.argmin(proc_ranges)

        radius_indices = int(self.bubble_radius / (data.angle_increment + 1e-6))
        min_idx = max(0, closest_index - radius_indices)
        max_idx = min(len(proc_ranges) - 1, closest_index + radius_indices)
        proc_ranges[min_idx:max_idx + 1] = 0

        start_i, end_i = self.find_max_gap(proc_ranges)
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)

        angle = data.angle_min + best_point_index * data.angle_increment

        max_steering_angle = np.deg2rad(34)
        angle = np.clip(angle, -max_steering_angle, max_steering_angle)

        gap_width_deg = np.rad2deg((end_i - start_i) * data.angle_increment)
        gap_depth = proc_ranges[best_point_index]

        is_straight = (gap_width_deg > 50.0) and (gap_depth > 4.0) and (abs(angle) < np.deg2rad(5))

        max_speed = 7.0
        cruise_speed = 4.5
        min_speed = 1.5

        if is_straight:
            speed = max_speed
        else:
            angle_abs = abs(angle)
            max_angle_for_scaling = 0.7
            angle_factor = max(0.0, 1.0 - (angle_abs / max_angle_for_scaling))
            speed = min_speed + angle_factor * (cruise_speed - min_speed)

        speed = np.clip(speed, min_speed, max_speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Follow The Gap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
