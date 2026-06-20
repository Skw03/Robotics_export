#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rmf_fleet_msgs.msg import RobotState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OfficeSyntheticLidar(Node):
    def __init__(self):
        super().__init__("office_synthetic_lidar")

        # Declare parameters (used by launch.xml <param> tags)
        self.declare_parameter("robot_name", "tinyRobot1")
        self.declare_parameter("rate", 10.0)
        # use_sim_time is a built-in ROS 2 parameter — do NOT re-declare it

        use_sim = False
        try:
            use_sim = self.get_parameter("use_sim_time").value
        except Exception:
            pass
        if use_sim:
            self.use_sim_time()

        self.robot_name = self.get_parameter("robot_name").value
        rate = self.get_parameter("rate").value

        self.pose: Optional[RobotState] = None
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.tf = TransformBroadcaster(self)

        state_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=50,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(RobotState, "robot_state",
                                 self._robot_state_cb, state_qos)

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(2.0)
        self.range_min = 0.08
        self.range_max = 12.0
        self.office_walls = [
            (-1.0, -13.0, 24.0, -13.0), (-1.0, 1.0, 24.0, 1.0),
            (-1.0, -13.0, -1.0, 1.0), (24.0, -13.0, 24.0, 1.0),
            (6.5, -13.0, 6.5, -2.0), (13.0, -13.0, 13.0, -2.0),
            (18.0, -13.0, 18.0, -2.0), (6.5, -2.0, 18.0, -2.0),
        ]
        self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f"Synthetic lidar ready: robot={self.robot_name}, "
            f"rate={rate}Hz, sim_time={use_sim}")

    def _robot_state_cb(self, msg: RobotState):
        if msg.name == self.robot_name:
            self.pose = msg

    def _ray_segment(self, ox, oy, dx, dy, x1, y1, x2, y2):
        sx = x2 - x1
        sy = y2 - y1
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-9:
            return None
        qpx = x1 - ox
        qpy = y1 - oy
        t = (qpx * sy - qpy * sx) / denom
        u = (qpx * dy - qpy * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _range_at(self, x, y, yaw, angle):
        ray_yaw = yaw + angle
        dx = math.cos(ray_yaw)
        dy = math.sin(ray_yaw)
        best = self.range_max
        for wall in self.office_walls:
            hit = self._ray_segment(x, y, dx, dy, *wall)
            if hit is not None and self.range_min <= hit < best:
                best = hit
        return best

    def _publish_tf(self, stamp, x, y, yaw):
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_footprint"
        odom_to_base.transform.translation.x = float(x)
        odom_to_base.transform.translation.y = float(y)
        odom_to_base.transform.rotation.z = math.sin(yaw / 2.0)
        odom_to_base.transform.rotation.w = math.cos(yaw / 2.0)

        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = stamp
        base_to_lidar.header.frame_id = "base_footprint"
        base_to_lidar.child_frame_id = "lidar_link"
        base_to_lidar.transform.translation.z = 0.25
        base_to_lidar.transform.rotation.w = 1.0
        self.tf.sendTransform([map_to_odom, odom_to_base, base_to_lidar])

    def _publish(self):
        if self.pose is None:
            return
        loc = self.pose.location
        now = self.get_clock().now().to_msg()
        self._publish_tf(now, loc.x, loc.y, loc.yaw)

        count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "lidar_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [self._range_at(loc.x, loc.y, loc.yaw, self.angle_min + i * self.angle_increment) for i in range(count)]
        self.scan_pub.publish(scan)


def main():
    try:
        rclpy.init()
        node = OfficeSyntheticLidar()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[FATAL] Synthetic lidar crashed: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
