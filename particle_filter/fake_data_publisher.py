#!/usr/bin/env python3
"""
Mock sensor data generator for ROS2 particle filter benchmarking.
Creates realistic odometry and range sensor streams.
"""
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, Pose, Twist, Point, Vector3
from std_msgs.msg import Header
import tf_transformations
from particle_filter.fake_data import FakeWorld


class MockSensorPublisher(Node):
    """Generates and transmits simulated robot sensor measurements."""
    
    def __init__(self):
        super().__init__('mock_sensor_publisher')
        
        # Setup parameters with defaults
        self.declare_parameter('odometry_channel', '/fake_odom')
        self.declare_parameter('lidar_channel', '/scan')  
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('total_steps', 1000)
        
        # Read parameters
        odom_topic = self.get_parameter('odometry_channel').get_parameter_value().string_value
        scan_topic = self.get_parameter('lidar_channel').get_parameter_value().string_value
        frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.max_steps = self.get_parameter('total_steps').get_parameter_value().integer_value
        
        # Create message publishers
        self.odometry_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.scan_publisher = self.create_publisher(LaserScan, scan_topic, 10)
        
        # Initialize world simulator
        self.world_simulator = FakeWorld()
        
        # Motion state
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.iteration = 0
        
        # Robot motion configuration
        self.velocity_linear = 0.2  # m/s
        self.velocity_angular = 0.05  # rad/s
        self.timestep = 1.0 / frequency
        
        # Noise configuration for realistic odometry
        self.sigma_x = 0.02
        self.sigma_y = 0.02
        self.sigma_theta = 0.01
        
        # Start periodic publishing
        self.timer = self.create_timer(self.timestep, self.publish_callback)
        
        self.get_logger().info(f'Mock sensor node started: {odom_topic}, {scan_topic} @ {frequency}Hz')
    
    def compute_next_pose(self):
        """Calculate robot's next position using motion model."""
        current_x = self.current_pose['x']
        current_y = self.current_pose['y']
        current_theta = self.current_pose['theta']
        
        # Compute ideal motion deltas
        delta_x_ideal = self.velocity_linear * np.cos(current_theta) * self.timestep
        delta_y_ideal = self.velocity_linear * np.sin(current_theta) * self.timestep
        delta_theta_ideal = self.velocity_angular * self.timestep
        
        # Add Gaussian noise to simulate real odometry
        noise_x = np.random.normal(0, self.sigma_x)
        noise_y = np.random.normal(0, self.sigma_y)
        noise_theta = np.random.normal(0, self.sigma_theta)
        
        delta_x_noisy = delta_x_ideal + noise_x
        delta_y_noisy = delta_y_ideal + noise_y
        delta_theta_noisy = delta_theta_ideal + noise_theta
        
        # Update pose
        self.current_pose['x'] += delta_x_noisy
        self.current_pose['y'] += delta_y_noisy
        self.current_pose['theta'] += delta_theta_noisy
        
        return delta_x_noisy, delta_y_noisy, delta_theta_noisy
    
    def build_quaternion_from_yaw(self, yaw):
        """Create quaternion message from yaw angle."""
        q_array = tf_transformations.quaternion_from_euler(0, 0, yaw)
        q_msg = Quaternion()
        q_msg.x = q_array[0]
        q_msg.y = q_array[1]
        q_msg.z = q_array[2]
        q_msg.w = q_array[3]
        return q_msg
    
    def publish_callback(self):
        """Timer callback to generate and publish sensor data."""
        # Update motion if not past step limit
        if self.iteration < self.max_steps:
            dx, dy, dtheta = self.compute_next_pose()
        else:
            # Stop moving after max steps but keep publishing
            dx = dy = dtheta = 0.0
        
        # Get current timestamp
        now = self.get_clock().now().to_msg()
        
        # Build and publish odometry
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        pose_msg = Pose()
        pose_msg.position = Point()
        pose_msg.position.x = self.current_pose['x']
        pose_msg.position.y = self.current_pose['y']
        pose_msg.position.z = 0.0
        pose_msg.orientation = self.build_quaternion_from_yaw(self.current_pose['theta'])
        odom_msg.pose.pose = pose_msg
        
        # Velocity
        twist_msg = Twist()
        twist_msg.linear = Vector3()
        twist_msg.linear.x = dx / self.timestep
        twist_msg.linear.y = dy / self.timestep
        twist_msg.linear.z = 0.0
        twist_msg.angular = Vector3()
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = dtheta / self.timestep
        odom_msg.twist.twist = twist_msg
        
        self.odometry_publisher.publish(odom_msg)
        
        # Generate laser measurements from world
        pose_tuple = (self.current_pose['x'], self.current_pose['y'], self.current_pose['theta'])
        measurements = self.world_simulator._sense(pose_tuple)
        
        # Build and publish laser scan
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = 'base_link'
        
        # Scan configuration
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        num_beams = max(len(measurements), 1)
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / num_beams
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.timestep
        scan_msg.range_min = 0.0
        scan_msg.range_max = 30.0
        
        # Extract range data
        if measurements:
            scan_msg.ranges = [float(meas[0]) for meas in measurements]
        else:
            scan_msg.ranges = [float('inf')]
        
        self.scan_publisher.publish(scan_msg)
        
        self.iteration += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockSensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
