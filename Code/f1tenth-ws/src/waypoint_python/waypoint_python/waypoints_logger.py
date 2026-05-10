import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from numpy.linalg import norm
import os
from time import gmtime, strftime, time

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(Odometry, '/odom', self.save_waypoint, 10)
        
        
        home = os.path.expanduser('~')
        self.file = open(strftime(f'{home}/f1tenth-ws/src/waypoint_python/wp-%Y-%m-%d-%H-%M-%S.csv', gmtime()), 'w')
        self.last_write_time=time()
        self.write_interval = 0.05
        self.get_logger().info('Saving waypoints...')

    # Save a CSV file for a track, include x, y, v, theta
    def save_waypoint(self, msg):
        current_time = time()
        
        if current_time - self.last_write_time >= self.write_interval:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            speed = norm(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y]), 2)
            q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
            quat = R.from_quat(q)
            euler = quat.as_euler("zxy",degrees = False)
            yaw = euler[0] + 2*math.pi

            
            self.file.write(f'{x}, {y}, {speed}, {yaw}\n')
            self.last_write_time = current_time




    def shutdown(self):
        self.file.close()
        self.get_logger().info('Goodbye')

def main(args=None):
    rclpy.init(args=args)
    waypoints_logger = WaypointsLogger()
    rclpy.spin(waypoints_logger)

    waypoints_logger.shutdown()
    waypoints_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
