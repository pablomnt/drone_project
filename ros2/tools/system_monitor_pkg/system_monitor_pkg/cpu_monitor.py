#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import psutil

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Create publishers
        self.cpu_pub = self.create_publisher(Float32, '/telemetry/cpu_usage_total', 10)
        self.ram_pub = self.create_publisher(Float32, '/telemetry/ram_usage_total', 10)
        
        # Initialize the CPU monitor
        psutil.cpu_percent()
        
        # Publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_metrics)
        self.get_logger().info("System Monitor Node Started! Publishing to /telemetry/...")

    def publish_metrics(self):
        cpu_usage = psutil.cpu_percent()
        ram_usage = psutil.virtual_memory().percent
        
        self.cpu_pub.publish(Float32(data=float(cpu_usage)))
        self.ram_pub.publish(Float32(data=float(ram_usage)))

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()