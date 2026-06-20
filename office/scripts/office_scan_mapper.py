#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rmf_fleet_msgs.msg import RobotState
from sensor_msgs.msg import LaserScan


class OfficeScanMapper(Node):
    def __init__(self):
        super().__init__("office_scan_mapper")
        self.robot_name = self.declare_parameter("robot_name", "tinyRobot1").value
        self.resolution = 0.1
        self.origin_x = -2.0
        self.origin_y = -14.0
        self.width = 280
        self.height = 170
        self.grid = [-1] * (self.width * self.height)
        self.pose: Optional[RobotState] = None
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.create_subscription(RobotState, "robot_state", self._robot_state_cb, 50)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

    def _robot_state_cb(self, msg: RobotState):
        if msg.name == self.robot_name:
            self.pose = msg

    def _index(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return my * self.width + mx
        return None

    def _mark_free_line(self, x0, y0, x1, y1):
        steps = max(int(math.hypot(x1 - x0, y1 - y0) / self.resolution), 1)
        for i in range(steps):
            ratio = i / steps
            idx = self._index(x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio)
            if idx is not None and self.grid[idx] < 100:
                self.grid[idx] = 0

    def _scan_cb(self, scan: LaserScan):
        if self.pose is None:
            return
        loc = self.pose.location
        robot_x = float(loc.x)
        robot_y = float(loc.y)
        robot_yaw = float(loc.yaw)
        angle = scan.angle_min
        for rng in scan.ranges:
            if math.isfinite(rng) and scan.range_min <= rng <= scan.range_max:
                world_angle = robot_yaw + angle
                hit_x = robot_x + rng * math.cos(world_angle)
                hit_y = robot_y + rng * math.sin(world_angle)
                self._mark_free_line(robot_x, robot_y, hit_x, hit_y)
                idx = self._index(hit_x, hit_y)
                if idx is not None:
                    self.grid[idx] = 100
            angle += scan.angle_increment
        self._publish(scan.header.stamp)

    def _publish(self, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = OfficeScanMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
