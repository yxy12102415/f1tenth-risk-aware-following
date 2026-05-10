#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class AutonomousDrivePublisher(Node):
    def __init__(self):
        super().__init__('autonomous_drive_publisher')
        # Set up Subscription
        self.AEB_active = False
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom', # this is more reliable than /pf/pose/odom
            self.odom_callback,
            1000
        )
        self.AEB_subscription = self.create_subscription(
            Bool,
            '/emergency_breaking', # this is more reliable than /pf/pose/odom
            self.AEB_callback,
            1000
        )
        self.x = 0.0
        #set up publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 100)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.publish_drive_command)
        
    def odom_callback(self, odom_msg):
        # Update current pose on x
        self.x = odom_msg.pose.pose.position.x
        
    # Listen to Safety node 
    def AEB_callback(self, msg): 
        # Update current pose on x
        self.AEB_active = msg.data
        if self.AEB_active:
        	self.get_logger().info('AEB Actived')

    def publish_drive_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Assuming "base_link" as the frame_id
        if self.AEB_active:
        	return
        
        if self.x < 100.0: # number can be replaced as any distance you want
        	msg.drive.speed = 1.0  # Setting speed
        else:
        	msg.drive.speed = 0.0  # Setting speed
        #msg.drive.steering_angle = 0.5  # Setting steering angle
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    autonomous_drive_publisher = AutonomousDrivePublisher()
    rclpy.spin(autonomous_drive_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autonomous_drive_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
