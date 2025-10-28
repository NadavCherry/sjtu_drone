#!/usr/bin/env python3
"""
map_position_viewer.py
----------------------
Loads a static occupancy grid map (hospital_map.pgm + hospital_map.yaml)
once at startup, listens to /simple_drone/gt_pose for the drone position,
and shows where the drone is on the map in real-time.
Also includes functions to check if nearby cells are free or occupied.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


class MapPositionViewer(Node):
    def __init__(self):
        super().__init__('map_position_viewer')

        # === Load map once ===
        map_yaml_path = "/root/drone_workspace/sjtu_drone/maps/hospital_map_downscaled.yaml"
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = info['resolution']
        self.origin = info['origin']  # [x, y, yaw]
        map_image_path = info['image']

        if not map_image_path.startswith('/'):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        # Load occupancy grid (0-255 grayscale)
        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        # Convert to occupancy (0=free, 1=occupied)
        self.map_data = np.zeros_like(img, dtype=np.uint8)
        self.map_data[img < 50] = 1  # dark pixels = occupied
        self.map_data = np.flipud(self.map_data)  # flip y-axis
        self.get_logger().info(f"Map loaded: {map_image_path}, size={self.map_data.shape}")

        # === Subscribe to ground truth pose ===
        self.pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)
        self.drone_pose = None

        # === Setup display ===
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.im = self.ax.imshow(self.map_data, cmap='gray', origin='lower')
        self.point, = self.ax.plot([], [], 'ro', markersize=5)
        self.ax.set_title("Drone position on occupancy map")
        plt.show(block=False)

    # === Coordinate conversions ===
    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        x_map = int((x_world - ox) / self.resolution)
        y_map = int((y_world - oy) / self.resolution)
        return x_map, y_map

    def map_to_world(self, x_map, y_map):
        ox, oy, _ = self.origin
        x_world = x_map * self.resolution + ox
        y_world = y_map * self.resolution + oy
        return x_world, y_world

    # === Pose callback ===
    def pose_callback(self, msg: Pose):
        self.drone_pose = (msg.position.x, msg.position.y)

    # === Occupancy checking ===
    def is_free(self, x_map, y_map):
        """Return True if the map cell is free (0)"""
        if 0 <= y_map < self.map_data.shape[0] and 0 <= x_map < self.map_data.shape[1]:
            return self.map_data[y_map, x_map] == 0
        return False

    # === Visualization update ===
    def update_display(self):
        if self.drone_pose is None:
            return

        x_map, y_map = self.world_to_map(*self.drone_pose)
        self.point.set_data([x_map], [y_map])
        self.ax.set_title(f"Drone Position (map coords): ({x_map}, {y_map})")
        plt.pause(0.05)

        # # Print occupancy info
        # if self.is_free(x_map, y_map):
        #     self.get_logger().info("Drone is in a FREE cell.")
        # else:
        #     self.get_logger().warn("Drone is in an OCCUPIED cell!")


    # === Main loop ===
    def spin(self):
        self.get_logger().info("Waiting for /simple_drone/gt_pose messages...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_display()


def main():
    rclpy.init()
    node = MapPositionViewer()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
