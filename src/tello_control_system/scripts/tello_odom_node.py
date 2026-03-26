#!/usr/bin/env python3
"""Fake odometry node for Tello based on velocity integration.

This node reads /tello/status (TelloStatus) and integrates the reported velocities
into a simple `nav_msgs/Odometry` + `nav_msgs/Path` stream for RViz visualization.
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tello_driver.msg import TelloStatus


class TelloOdomNode:
    """Estimates odometry from velocity information."""

    def __init__(self):
        rospy.init_node('tello_odom_node', anonymous=False)

        # Parameters
        self.rate_hz = 20
        self.path_max_length = 500

        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.path_poses = []

        # Timing
        self.last_time = None
        self.last_msg_time = None

        # Publishers
        self.odom_pub = rospy.Publisher('/tello/odom', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('/tello/path', Path, queue_size=10)

        # Subscriber
        self.status_sub = rospy.Subscriber('/tello/status', TelloStatus, self._status_cb, queue_size=10)

        rospy.loginfo('Tello odom node initialized')

    def _status_cb(self, msg):
        now = rospy.Time.now()
        self.last_msg_time = now

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).to_sec()
        if dt <= 0:
            return

        self.last_time = now

        vx = msg.speed_easting_mps
        vy = msg.speed_northing_mps
        height = msg.height_m

        if abs(vx) < 0.02:
            vx = 0.0
        if abs(vy) < 0.02:
            vy = 0.0

        self.velocity = [vx, vy, 0.0]

        # Integrate position
        self.position[0] += vx * dt
        self.position[1] += vy * dt
        self.position[2] = height

        # Drift reduction
        self.position[0] *= 0.999
        self.position[1] *= 0.999

    def _check_timeout(self):
        if self.last_msg_time is None:
            return

        elapsed = (rospy.Time.now() - self.last_msg_time).to_sec()
        if elapsed > 1.0:
            self.velocity = [0.0, 0.0, 0.0]

    def _publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]

        odom.pose.pose.orientation.x = self.orientation[0]
        odom.pose.pose.orientation.y = self.orientation[1]
        odom.pose.pose.orientation.z = self.orientation[2]
        odom.pose.pose.orientation.w = self.orientation[3]

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]

        self.odom_pub.publish(odom)

    def _publish_path(self, now):
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'

        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]

        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]

        self.path_poses.append(pose)
        if len(self.path_poses) > self.path_max_length:
            excess = len(self.path_poses) - self.path_max_length
            self.path_poses = self.path_poses[excess:]

        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'map'
        path.poses = self.path_poses

        self.path_pub.publish(path)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo('Tello odometry node running at %d Hz', self.rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._check_timeout()
            self._publish_odom(now)
            self._publish_path(now)
            rate.sleep()

        rospy.loginfo('Tello odometry node shutting down')


def main():
    try:
        node = TelloOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Tello odometry node terminated')


if __name__ == '__main__':
    main()
