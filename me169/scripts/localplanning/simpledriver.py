#!/usr/bin/env python3
#
#   odometry.py
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import rospy
import tf2_ros

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


# /move_base_simple/goal


#
#   Odometry Object
#
class LocalDriver:
    # Initialize.
    def __init__(self):
        # Create a publisher to send velocity commands.
        self.pub_vcmd = rospy.Publisher('/vel_cmd', Twist,
                                        queue_size=10)

        # TODO: finish setting up communication
        # Create a publisher to send odometry information.
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create a TF2 transform broadcaster.
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to twist commands.
        rospy.Subscriber('/vel_cmd', Twist, self.cb_vel_cmd)

        # Create a subscriber to listen to wheel state.
        rospy.Subscriber('/wheel_state', JointState, self.cb_wheel_state)

    def cb_nav_goal(self, msg: PoseStamped):
        pass

    def cb_odom(self, msg: Odometry):
        # Initialize the (repeating) message data.
        # TODO: move this into another callback for an external timer.
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        self.pub_vcmd.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('local_driver')

    # Instantiate the Odometry object
    local_driver = LocalDriver()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Local Driver spinning...")
    rospy.spin()
    rospy.loginfo("Local Driver stopped.")
