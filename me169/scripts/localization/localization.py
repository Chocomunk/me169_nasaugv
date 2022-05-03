#!/usr/bin/env python3
#
#   simpledriver.py
#
#   Drive controller node.  This
#   (a) converts a comparison between current pose and target pose into body velocity commands.
#
#   Node:       /local_driver
#   Publish:    /vel_cmd                geometry_msgs/Twist
#   Subscribe:  /odom                   geometry_msgs/TransJointState
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#               /scan                   sensor_msgs/LaserScan
#
import math
import rospy
import numpy as np

from geometry_msgs.msg  import PoseStamped
from nav_msgs.msg       import Odometry, OccupancyGrid
from sensor_msgs.msg    import LaserScan

from PlanarTransform import PlanarTransform


# CONSTANTS

#
#   Localization Algorithm
#
class FixedTransform:
    # Initialize.
    def __init__(self):
        self.pose_transfom = PlanarTransform.basic(0, 0, 0)

    def set_init_pose(self, init_pose: PoseStamped):
        self.pose_transfom = PlanarTransform.fromPose(init_pose.pose)
    
    def update(last_odom, last_scan):
        # TODO


#
#   Localization Object
#
class Localization:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_odom = Odometry()
        self.init_pose = PoseStamped()
        self.last_scan = LaserScan()

        self.localization_algo = FixedTransform()

        # Create a publisher to map space pose.
        self.pub_pose = rospy.Publisher('/pose', PoseStamped,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/odom', Odometry, self.cb_odom)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/initialpose', PoseStamped, self.cb_init_pose)

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

        # TODO: Add map
        # Wait 30sec for a map.
        # rospy.loginfo("Waiting for a map...")
        # self.mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

    def cb_init_pose(self, msg: PoseStamped):
        self.init_pose = msg
        self.localization_algo.set_init_pose(msg)

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

        # NOTE: params subject to change
        ...OUTPUT VAL... = self.localization_algo.update(msg, self.last_scan)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.pose.position = ...SET POINT...
        pose.pose.orientation = ...SET QUATERNION...
        self.pub_pose.publish(pose)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localization')

    # Instantiate the Odometry object
    localization = Localization()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Localization running...")
    rospy.spin()
    rospy.loginfo("Localization stopped.")
