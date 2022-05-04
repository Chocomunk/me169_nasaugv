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
import tf2_ros
import numpy as np

from geometry_msgs.msg  import PoseStamped, Pose, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg       import Odometry, OccupancyGrid
from sensor_msgs.msg    import LaserScan

from PlanarTransform import PlanarTransform


# CONSTANTS

#
#   Localization Correction
#
class IdentityCorrection:
    # Initialize.
    def __init__(self, map: OccupancyGrid):
        self.map = map      # Not needed, but store for reference
        self.identity_tf = PlanarTransform.basic(0, 0, 0)

    def get_tf(self):
        return self.identity_tf
    
    def update(self, last_odom: Odometry, last_scan: LaserScan):
        # Do nothing for now, keep returning the identity transform
        pass


#
#   Localization Object
#
class Localization:
    # Initialize.
    def __init__(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

        # State Variables
        self.last_odom = Odometry()
        self.init_pose = PoseWithCovarianceStamped()
        self.last_scan = LaserScan()

        # Transforms
        self.map2base = PlanarTransform.basic(0, 0, 0)
        self.map2odom = PlanarTransform.basic(0, 0, 0)
        self.odom2base = PlanarTransform.basic(0, 0, 0)

        self.correction = IdentityCorrection(self.mapmsg)

        # -------------------- Publishers/Subscribers --------------------

        # Create a publisher to map space pose.
        self.pub_pose = rospy.Publisher('/pose', PoseStamped,
                                        queue_size=10)

        # TF broadcaster for map2odom
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/odom', Odometry, self.cb_odom)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.cb_init_pose)

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        assert (msg.header.frame_id == "map")
        self.init_pose = msg

        # Update map transforms to reflect the new map pose
        self.map2base = PlanarTransform.fromPose(msg.pose.pose)
        self.map2odom = self.map2base * self.odom2base.inv()

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

        # NOTE: this is subject to change
        self.correction.update(self.last_odom, msg)

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

        # Create the transform msg and broadcast (reuse the time stamp).
        # NOTE: we broadcast for every /odom, but only update for each /initialpose
        msg = TransformStamped()
        msg.header.stamp            = msg.header.stamp
        msg.header.frame_id         = 'map'
        msg.child_frame_id          = 'odom'
        msg.transform               = self.map2odom.toTransform()
        self.brd_tf.sendTransform(msg)

        # Compute map2base pose corresponding to the odom2base reading
        self.odom2base = PlanarTransform.fromPose(msg.pose.pose)
        pose_tf = self.correction.get_tf() * self.map2odom * self.odom2base

        # Publish new map2base pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.pose = pose_tf.toPose()
        self.pub_pose.publish(pose_stamped)


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
