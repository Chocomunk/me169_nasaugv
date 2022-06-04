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
import rospy
import tf2_ros

from geometry_msgs.msg  import PoseStamped, TransformStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg       import Odometry, OccupancyGrid

from planar_transform import PlanarTransform


#
#   Localization Object
#
class MapPose:
    # Initialize.
    def __init__(self):
        # -------------------- TF Broadcasters/Listeners --------------------

        # First create a TF2 listener. This implicily fills a local
        # buffer, so we can always retrive the transforms we want.
        self.tfBuffer = tf2_ros.Buffer()
        self.tflisten = tf2_ros.TransformListener(self.tfBuffer)

        # -------------------- Publishers/Subscribers --------------------

        # Create a publisher to map space pose.
        self.pub_pose = rospy.Publisher('/pose', PoseStamped,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/odom', Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg: Odometry):
        # Get map2odom
        tfmsg = self.tfBuffer.lookup_transform("map",
            msg.header.frame_id,
            msg.header.stamp,
            rospy.Duration(0.1))
        map2odom = PlanarTransform.fromTransform(tfmsg.transform)

        # Compute map2base pose corresponding to the odom2base reading
        odom2base = PlanarTransform.fromPose(msg.pose.pose)
        pose_tf = map2odom * odom2base

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
    rospy.init_node('map_pose')

    # Instantiate the Odometry object
    map_pose = MapPose()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("MapPose running...")
    rospy.spin()
    rospy.loginfo("MapPose stopped.")
