import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class Sensors(Node):
    '''
    Obtains LIDAR data and publishes the closest object and the angle to that objects
    '''
    def __init__(self):
        super().__init__('sensor')

        self.fov = math.pi/6
        self.fov_wide = math.pi/3

        scan_qos_profile = QoSProfile(depth=5)
        scan_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        scan_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        scan_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self._lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._lidar_callback,
            scan_qos_profile)
        
        self._dist_pub = self.create_publisher(
            Pose2D,
            '/object_dist',
            10)
        
    def _lidar_callback(self, scan:LaserScan):
        '''
        Takes laser scan data, filters it, then publishes the closest point and the angle to that point 
        relative to the robots current heading
        
        Args:
            scan (LaserScan): LIDAR data
        '''
        pose_dist = Pose2D()
        n = len(scan.ranges)
        # Convert laser scan data into a readable, wrapped array
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        ### WIDE SKIRT ###
        # Mask for field of view
        mask_wide = np.abs(angles) <= (self.fov_wide/2)
        angles_wide = angles[mask_wide]
        ranges_wide = np.array(scan.ranges)[mask_wide]

        # Mask for nan and infinite
        filt_wide = np.isfinite(ranges_wide)
        angles_wide = angles_wide[filt_wide]
        ranges_wide = ranges_wide[filt_wide]

        idx_wide = int(np.argmin(ranges_wide))
        min_angle_wide = angles_wide[idx_wide]

        ### REG SKIRT ###
        # Mask for field of view
        mask = np.abs(angles) <= (self.fov/2)
        angles = angles[mask]
        ranges = np.array(scan.ranges)[mask]

        # Mask for nan and infinite
        filt = np.isfinite(ranges)
        angles = angles[filt]
        ranges = ranges[filt]

        # If no ranges can be obtained, publish a debug message to inform other nodes
        if not np.any(ranges):
            pose_dist.x = 1.0
            pose_dist.y = -1.0
            pose_dist.theta = 1.0
            self._dist_pub.publish(pose_dist)
            
        # Determine shortest distance and angle
        idx = int(np.argmin(ranges))
        min_dist = ranges[idx]

        pose_dist.x = float(min_dist)
        pose_dist.y = 0.0
        pose_dist.theta = float(min_angle_wide)
        
        self._dist_pub.publish(pose_dist)

def main():
    rclpy.init()
    rclpy.spin(Sensors())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
