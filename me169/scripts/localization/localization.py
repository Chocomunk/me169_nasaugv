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

from geometry_msgs.msg  import PoseStamped, TransformStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg       import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan

from planar_transform import PlanarTransform
from corrections import IdentityCorrection, BasicLeastSquaresCorrection


MAX_DIST_FROM_ROBOT = 3


def laser2cart(scan: LaserScan):
    r = np.array(scan.ranges) / 1.14
    t = np.linspace(scan.angle_min, scan.angle_max, len(r))
    idxs = np.where(r < MAX_DIST_FROM_ROBOT)
    r = r[idxs, None]
    t = t[idxs]
    return r * np.array((np.cos(t), np.sin(t))).T

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
        self.odom2laser = PlanarTransform.basic(0, 0, 0)

        self.correction = BasicLeastSquaresCorrection(self.mapmsg)
        # self.correction = IdentityCorrection(self.mapmsg)

        # -------------------- TF Broadcasters/Listeners --------------------

        # First create a TF2 listener. This implicily fills a local
        # buffer, so we can always retrive the transforms we want.
        self.tfBuffer = tf2_ros.Buffer()
        self.tflisten = tf2_ros.TransformListener(self.tfBuffer)

        # TF broadcaster for map2odom
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Give the broadcaster time to connect, then send the initial transform.
        rospy.sleep(0.25)
        tfmsg = TransformStamped()
        tfmsg.header.stamp = rospy.Time.now()
        tfmsg.header.frame_id = "map"
        tfmsg.child_frame_id = "odom"
        tfmsg.transform = PlanarTransform.unity().toTransform()
        self.brd_tf.sendTransform(tfmsg)

        # -------------------- Publishers/Subscribers --------------------

        # Create a publisher to map space pose.
        self.pub_pose = rospy.Publisher('/pose', PoseStamped,
                                        queue_size=10)
        self.pub_lasmap = rospy.Publisher('/lasermap', Marker,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/odom', Odometry, self.cb_odom)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.cb_init_pose)

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        assert (msg.header.frame_id == "map"), "Init pose not in map frame"
        self.init_pose = msg

        # Update map transforms to reflect the new map pose
        self.map2base = PlanarTransform.fromPose(msg.pose.pose)
        self.map2odom = self.map2base * self.odom2base.inv()

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

        # Get odom2laser
        # Assume we received a scan message (scanmsg). Use TF to
        # look up the matching transform. Give it up to 100ms, in
        # case Linux has (temporarily) swapped out the odometry node.
        tfmsg = self.tfBuffer.lookup_transform("odom",
            self.last_scan.header.frame_id,
            self.last_scan.header.stamp,
            # "laser",
            # rospy.Time(0),
            rospy.Duration(0.1))
        self.odom2laser = PlanarTransform.fromTransform(tfmsg.transform)

        map2laser = self.map2odom * self.odom2laser
        laserpts = laser2cart(self.last_scan)
        laser_map = map2laser.apply(laserpts)

        laser_inrange = self.correction.update(laser_map)
        self.pub_laser_map(laser_inrange, msg)

        self.map2odom = self.correction.get_tf() * self.map2odom

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

        # Create the transform msg and broadcast (reuse the time stamp).
        # NOTE: we broadcast for every /odom, but only update for each /initialpose
        tf_msg = TransformStamped()
        tf_msg.header.stamp         = msg.header.stamp
        tf_msg.header.frame_id      = 'map'
        tf_msg.child_frame_id       = 'odom'
        tf_msg.transform            = self.map2odom.toTransform()
        self.brd_tf.sendTransform(tf_msg)

        # Compute map2base pose corresponding to the odom2base reading
        self.odom2base = PlanarTransform.fromPose(msg.pose.pose)
        pose_tf = self.map2odom * self.odom2base

        # Publish new map2base pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.pose = pose_tf.toPose()
        self.pub_pose.publish(pose_stamped)

    def pub_laser_map(self, laser_map, scan):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = scan.header.stamp
        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        s = 0.05
        msg.scale.x = s
        msg.scale.y = s
        msg.scale.z = s

        msg.color.a = 1.0
        msg.color.r = 0
        msg.color.g = 0
        msg.color.b = 1

        pts = []
        for i in range(len(laser_map)):
            pt = Point()
            pt.x = laser_map[i,0]
            pt.y = laser_map[i,1]
            pts.append(pt)
        
        msg.points = pts
        self.pub_lasmap.publish(msg)


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
