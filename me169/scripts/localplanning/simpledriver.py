#!/usr/bin/env python3
#
#   simpledriver.py
#
#   Local planning node.  This
#   (a) converts a comparison between current pose and target pose into body velocity commands.
#
#   Node:       /local_driver
#   Publish:    /vel_cmd                geometry_msgs/Twist
#   Subscribe:  /odom                   geometry_msgs/TransJointState
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#
import math
import rospy
import tf2_ros

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


LAM_TURN = 1 / 0.7      # time constant multiplier for the angular speed (hz)
LAM_FORW = 1 / 0.5      # timst contant multiplier for forward speed (hz)
POS_TOL = 0.05              # (meters)
VEL_TOL = 0.4               # (m/s)
THETA_TOL = math.pi/12.     # (radians)
OMEGA_LIM = math.pi/3.      # (rad/s)


def angle_diff(angle1, angle2):
    """This function finds the smallest angle difference between
    angle1 and angle2"""
    return (angle1-angle2) - 2.0*math.pi * round(0.5*(angle1-angle2)/math.pi)


#
#   LocalDriver Object
#
class LocalDriver:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_odom = Odometry()
        self.nav_goal = PoseStamped()
        self.reached_goal = True

        # Create a publisher to send velocity commands.
        self.pub_vcmd = rospy.Publisher('/vel_cmd', Twist,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/odom', Odometry, self.cb_odom)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_nav_goal)

    def cb_nav_goal(self, msg: PoseStamped):
        self.nav_goal = msg
        self.reached_goal = False

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_timer(self, event):
        if not self.reached_goal:
            # Find current angle
            cur_q = self.last_odom.pose.pose.orientation
            cur_th = 2*math.atan2(cur_q.z, cur_q.w)

            # Find heading angle to goal
            cur_px = self.last_odom.pose.pose.position.x
            cur_py = self.last_odom.pose.pose.position.y
            goal_px = self.nav_goal.pose.position.x
            goal_py = self.nav_goal.pose.position.y
            dx = goal_px - cur_px
            dy = goal_py - cur_py

            # Initialize command vars
            vx = 0
            wz = 0
            adiff = 0

            dist = math.sqrt(dy*dy + dx*dx)
            if abs(dist) < POS_TOL:
                # Find desired heading angle
                goal_q = self.nav_goal.pose.orientation
                nav_th = 2*math.atan2(goal_q.z, goal_q.w)
                adiff = angle_diff(nav_th, cur_th)
                
                if abs(adiff) < THETA_TOL:
                    adiff = 0
                    self.reached_goal = True
            else:
                # Find heading to goal position
                goal_th = math.atan2(dy, dx)
                adiff = angle_diff(goal_th, cur_th)

                # Compute desired velocity and omega
                vd = LAM_FORW * dist
                vd = min(VEL_TOL, max(-VEL_TOL, vd))    # Clamp
                vx = vd * math.cos(adiff)

            # Compute rotating speed
            wz = LAM_TURN * adiff
            wz = min(OMEGA_LIM, max(-OMEGA_LIM, wz))    # Clamp

            # Publish velocity commands
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

    # Define durations
    duration = rospy.Duration(1. / 10)       # 10 Hz
    dt       = duration.to_sec()

    # Instantiate the Odometry object
    local_driver = LocalDriver()

    # Create the timer.
    timer = rospy.Timer(duration, local_driver.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Local Driver running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Local Driver stopped.")
