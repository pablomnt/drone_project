#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker



class WaypointMarkers(Node):
    def __init__(self):
        super().__init__('waypoint_markers')
        self.publisher = self.create_publisher(Marker, '/waypoint_markers', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)

        self.waypoints = [
            (1.0, 0.0, -1.0),
            (2.0, 1.0, -1.0),
            (3.0, 0.0, -1.0),
        ]

    def publish_markers(self):
        for i, (x, y, z) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.publisher.publish(marker)

def main():
    rclpy.init()
    node = WaypointMarkers()
    rclpy.spin(node)
    rclpy.shutdown()
