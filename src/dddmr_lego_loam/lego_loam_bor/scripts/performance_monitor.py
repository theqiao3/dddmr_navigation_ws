#!/usr/bin/env python3
"""
Cloud Preprocessor 性能分析脚本
用于实时监控点云滤波的性能指标
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from collections import deque
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        self.input_topic = self.declare_parameter('input_topic', '/livox/lidar/pointcloud').value
        self.output_topic = self.declare_parameter('output_topic', '/livox/lidar/pointcloud_filtered').value
        
        # Statistics
        self.window_size = 100
        self.input_sizes = deque(maxlen=self.window_size)
        self.output_sizes = deque(maxlen=self.window_size)
        self.timestamps = deque(maxlen=self.window_size)
        self.latencies = deque(maxlen=self.window_size)
        
        self.last_timestamp = None
        self.frame_count = 0
        
        # Subscriptions
        self.sub_input = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.input_callback,
            1
        )
        
        self.sub_output = self.create_subscription(
            PointCloud2,
            self.output_topic,
            self.output_callback,
            1
        )
        
        # Timer for periodic reporting
        self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info(f'Performance Monitor started')
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
    
    def input_callback(self, msg: PointCloud2):
        """Callback for input point cloud"""
        current_time = time.time()
        self.timestamps.append(current_time)
        self.input_sizes.append(msg.width * msg.height)
    
    def output_callback(self, msg: PointCloud2):
        """Callback for output point cloud"""
        current_time = time.time()
        self.output_sizes.append(msg.width * msg.height)
        
        # Calculate latency (approximation)
        if len(self.timestamps) > 0:
            latency = (current_time - self.timestamps[-1]) * 1000  # Convert to ms
            self.latencies.append(latency)
    
    def report_statistics(self):
        """Report performance statistics"""
        if len(self.input_sizes) < 10:
            return
        
        import statistics
        
        avg_input = statistics.mean(self.input_sizes)
        avg_output = statistics.mean(self.output_sizes)
        avg_latency = statistics.mean(self.latencies) if self.latencies else 0
        
        compression_ratio = (1 - avg_output / avg_input) * 100 if avg_input > 0 else 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('PERFORMANCE STATISTICS (last 100 frames):')
        self.get_logger().info(f'  Avg Input Points:   {avg_input:>10.0f}')
        self.get_logger().info(f'  Avg Output Points:  {avg_output:>10.0f}')
        self.get_logger().info(f'  Compression Ratio:  {compression_ratio:>10.1f}%')
        self.get_logger().info(f'  Avg Latency:        {avg_latency:>10.2f} ms')
        
        if self.latencies:
            min_latency = min(self.latencies)
            max_latency = max(self.latencies)
            self.get_logger().info(f'  Min Latency:        {min_latency:>10.2f} ms')
            self.get_logger().info(f'  Max Latency:        {max_latency:>10.2f} ms')
        
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
