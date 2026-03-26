#!/usr/bin/env python3
"""
Tello Odometry Node
Estimates drone position using velocity integration from /tello/status
and publishes nav_msgs/Odometry for RViz visualization.

The node integrates velocity over time to estimate position, with drift reduction
to improve accuracy. It also maintains a path history for trajectory visualization.
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from tello_driver.msg import TelloStatus


class TelloOdometryNode:
    """
    A ROS node that estimates Tello drone odometry from velocity data.
    
    This node subscribes to /tello/status and estimates the drone's position
    by integrating velocity over time. It publishes odometry data and a path
    message for visualization in RViz.
    """
    
    def __init__(self):
        """Initialize the odometry node and ROS publishers/subscribers."""
        rospy.init_node('tello_odom_node', anonymous=False)
        
        # Node parameters
        self.rate = rospy.Rate(20)  # 20 Hz
        self.velocity_threshold = 0.02  # m/s - threshold for drift reduction
        self.decay_factor = 0.999  # Position decay for drift reduction
        self.message_timeout = 1.0  # seconds - stop integration if no message
        self.max_path_length = 1000  # Maximum number of poses in path
        
        # State variables
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z in meters
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz in m/s
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # qx, qy, qz, qw
        self.path_poses = []  # List of PoseStamped for path message
        
        # Timing
        self.last_time = None
        self.last_message_time = None
        
        # Publishers
        self.odom_pub = rospy.Publisher(
            '/tello/odom',
            Odometry,
            queue_size=10
        )
        self.path_pub = rospy.Publisher(
            '/tello/path',
            Path,
            queue_size=10
        )
        
        # Subscriber
        self.status_sub = rospy.Subscriber(
            '/tello/status',
            TelloStatus,
            self.status_callback,
            queue_size=10
        )
        
        rospy.loginfo("Tello Odometry Node initialized")
        rospy.loginfo("Publishing to /tello/odom and /tello/path")
    
    def status_callback(self, msg):
        """
        Callback for /tello/status messages.
        
        Extracts velocity and height from the status message and triggers
        position integration.
        
        Args:
            msg (TelloStatus): Status message from tello_driver
        """
        current_time = rospy.Time.now()
        self.last_message_time = current_time
        
        # Initialize timing on first message
        if self.last_time is None:
            self.last_time = current_time
            rospy.loginfo("First status message received, starting odometry integration")
            return
        
        # Compute time delta
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return
        
        self.last_time = current_time
        
        # Extract velocity components from status message
        # Convert NED (North-East-Down) convention to local frame:
        # North -> y axis, East -> x axis
        vx = msg.speed_easting_mps      # East velocity (x-axis)
        vy = msg.speed_northing_mps     # North velocity (y-axis)
        height = msg.height_m           # Height (z-axis)
        
        # Apply drift reduction: zero out small velocities
        if abs(vx) < self.velocity_threshold:
            vx = 0.0
        if abs(vy) < self.velocity_threshold:
            vy = 0.0
        
        # Store velocity for odometry message
        self.velocity[0] = vx
        self.velocity[1] = vy
        self.velocity[2] = 0.0
        
        # Integrate velocity to estimate position
        # Position update: p += v * dt
        self.position[0] += vx * dt
        self.position[1] += vy * dt
        self.position[2] = height  # Use absolute height measurement
        
        # Apply position decay for drift reduction
        # Slowly decay position estimate to reduce accumulated error
        self.position[0] *= self.decay_factor
        self.position[1] *= self.decay_factor
        # z (height) is set directly from sensor, no decay
    
    def check_message_timeout(self):
        """
        Check if the status message has timed out.
        
        If no message has been received for longer than message_timeout,
        stop velocity integration by zeroing velocities.
        """
        if self.last_message_time is None:
            return
        
        time_since_message = (rospy.Time.now() - self.last_message_time).to_sec()
        
        if time_since_message > self.message_timeout:
            # Timeout occurred - stop integration
            self.velocity = np.array([0.0, 0.0, 0.0])
            if time_since_message > self.message_timeout + 0.5:
                rospy.logwarn(
                    f"Tello status timeout: {time_since_message:.2f}s, "
                    "stopping odometry integration"
                )
    
    def publish_odometry(self, current_time):
        """
        Publish odometry message.
        
        Creates and publishes a nav_msgs/Odometry message with current
        position and velocity estimates.
        
        Args:
            current_time (rospy.Time): Current ROS time stamp
        """
        odom_msg = Odometry()
        
        # Header
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        
        # Position (from integration)
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        
        # Orientation (fixed identity quaternion)
        # qx, qy, qz, qw = 0, 0, 0, 1 (no rotation)
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        
        # Pose covariance (assume some uncertainty)
        # Diagonal elements: [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.1   # x variance
        odom_msg.pose.covariance[7] = 0.1   # y variance
        odom_msg.pose.covariance[14] = 0.1  # z variance
        odom_msg.pose.covariance[21] = 0.1  # roll variance
        odom_msg.pose.covariance[28] = 0.1  # pitch variance
        odom_msg.pose.covariance[35] = 0.1  # yaw variance
        
        # Twist (velocity)
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        
        # Angular velocity (not estimated)
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Twist covariance
        odom_msg.twist.covariance[0] = 0.05   # vx variance
        odom_msg.twist.covariance[7] = 0.05   # vy variance
        odom_msg.twist.covariance[14] = 0.05  # vz variance
        odom_msg.twist.covariance[21] = 0.1   # angular x variance
        odom_msg.twist.covariance[28] = 0.1   # angular y variance
        odom_msg.twist.covariance[35] = 0.1   # angular z variance
        
        self.odom_pub.publish(odom_msg)
    
    def publish_path(self, current_time):
        """
        Publish path message.
        
        Creates and publishes a nav_msgs/Path message containing the history
        of drone poses. Limits the path to max_path_length points.
        
        Args:
            current_time (rospy.Time): Current ROS time stamp
        """
        # Create a pose stamped for the current position
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = "map"
        
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]
        
        # Add to path history
        self.path_poses.append(pose)
        
        # Limit path length to prevent memory issues
        if len(self.path_poses) > self.max_path_length:
            # Remove oldest poses
            excess = len(self.path_poses) - self.max_path_length
            self.path_poses = self.path_poses[excess:]
        
        # Create and publish path message
        path_msg = Path()
        path_msg.header.stamp = current_time
        path_msg.header.frame_id = "map"
        path_msg.poses = self.path_poses
        
        self.path_pub.publish(path_msg)
    
    def run(self):
        """
        Main loop of the odometry node.
        
        Runs at 20 Hz, checking for message timeouts and publishing
        odometry and path messages.
        """
        rospy.loginfo("Tello Odometry Node running at 20 Hz")
        
        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                
                # Check for message timeout
                self.check_message_timeout()
                
                # Publish messages
                self.publish_odometry(current_time)
                self.publish_path(current_time)
                
                # Sleep to maintain 20 Hz rate
                self.rate.sleep()
                
            except KeyboardInterrupt:
                rospy.loginfo("Tello Odometry Node interrupted by user")
                break
            except Exception as e:
                rospy.logerr(f"Error in odometry loop: {e}")
                continue
        
        rospy.loginfo("Tello Odometry Node shutting down")


def main():
    """Entry point for the tello_odom_node."""
    try:
        node = TelloOdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tello Odometry Node terminated")


if __name__ == '__main__':
    main()
