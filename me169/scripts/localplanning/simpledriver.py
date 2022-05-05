#!/usr/bin/env python3
#
#   simpledriver.py
#
#   Drive controller node.  This
#   (a) converts a comparison between current pose and target pose into body velocity commands.
#
#   Node:       /local_driver
#   Publish:    /vel_cmd                geometry_msgs/Twist
#   Subscribe:  /pose                   geometry_msgs/PoseStamped
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#               /scan                   sensor_msgs/LaserScan
#
import math
import rospy
import numpy as np

from geometry_msgs.msg  import PoseStamped, Twist
from sensor_msgs.msg    import LaserScan


LAM_TURN = 1 / 0.7      # time constant multiplier for the angular speed (hz)
LAM_FORW = 1 / 0.5      # timst contant multiplier for forward speed (hz)
POS_TOL = 0.05              # (meters)
VEL_TOL = 0.4               # (m/s)
THETA_TOL = math.pi/12.     # (radians)
OMEGA_LIM = math.pi/3.      # (rad/s)
TURN_DELAY = 0.25           # (sec)
SCAN_ANGLE = math.pi/3      # (radians)
WALL_THRESH = 0.3           # (meters)


def angle_diff(angle1, angle2):
    """This function finds the smallest angle difference between
    angle1 and angle2"""
    return (angle1-angle2) - 2.0*math.pi * round(0.5*(angle1-angle2)/math.pi)


def closest_scan(scan: LaserScan, angle_range=SCAN_ANGLE):
    """This function returns the float value of the minimum laser scan in the specified angle range."""
    offset_l = -int((scan.angle_min + angle_range/2) / scan.angle_increment)
    offset_r =  int((scan.angle_max - angle_range/2) / scan.angle_increment)
    start_index = max(0, offset_l)
    end_index = len(scan.ranges) - max(0, offset_r)
    clamped_range = np.array(scan.ranges[start_index:end_index+1])
    nonzero = clamped_range[clamped_range > 0]
    return 0 if len(nonzero) == 0 else np.min(nonzero)


class DriveTurn:
    """ Drive and turn simultaneously, then turn into the correct heading """

    def __init__(self):
        self.last_facing_time = rospy.Time.now()
        self.finished = True

    def reset(self):
        self.finished = False

    def update(self, pose: PoseStamped, nav_goal: PoseStamped, scan: LaserScan):
        # Find current angle
        cur_q = pose.pose.orientation
        cur_th = 2*math.atan2(cur_q.z, cur_q.w)

        # Find heading angle to goal
        cur_px = pose.pose.position.x
        cur_py = pose.pose.position.y
        goal_px = nav_goal.pose.position.x
        goal_py = nav_goal.pose.position.y
        dx = goal_px - cur_px
        dy = goal_py - cur_py

        # Initialize command vars
        vx = 0
        wz = 0
        adiff = 0

        dist = math.sqrt(dy*dy + dx*dx)
        if abs(dist) < POS_TOL:
            # Find desired heading angle
            goal_q = nav_goal.pose.orientation
            nav_th = 2*math.atan2(goal_q.z, goal_q.w)
            adiff = angle_diff(nav_th, cur_th)
            
            if abs(adiff) < THETA_TOL:
                adiff = 0
                self.finished = True
        else:
            # Find heading to goal position
            goal_th = math.atan2(dy, dx)
            adiff = angle_diff(goal_th, cur_th)

            # Compute desired velocity and omega
            if closest_scan(scan) > WALL_THRESH:
                vd = LAM_FORW * dist
                vd = min(VEL_TOL, max(-VEL_TOL, vd))    # Clamp
                vx = vd * max(0, math.cos(adiff))

        # Compute rotating speed
        wz = LAM_TURN * adiff
        wz = min(OMEGA_LIM, max(-OMEGA_LIM, wz))    # Clamp

        return vx, wz


class TurnDriveTurn:
    """ Turn toward the target, drive, then turn into the correct heading """

    def __init__(self):
        self.last_facing_time = rospy.Time.now()
        self.facing_goal = False
        self.at_goal = False
        self.finished = True

    def reset(self):
        self.facing_goal = False
        self.at_goal = False
        self.finished = False

    def update(self, pose: PoseStamped, nav_goal: PoseStamped):
        # Find current angle
        cur_q = pose.pose.orientation
        cur_th = 2*math.atan2(cur_q.z, cur_q.w)

        # Find heading angle to goal
        cur_px = pose.pose.position.x
        cur_py = pose.pose.position.y
        goal_px = nav_goal.pose.position.x
        goal_py = nav_goal.pose.position.y
        dx = goal_px - cur_px
        dy = goal_py - cur_py
        goal_th = math.atan2(dy, dx)

        # Initialize command vars
        vx = 0
        wz = 0
        d_angle = 0

        # Angle-to-goal and Dist-to-goal metrics
        d_angle_goal = angle_diff(goal_th, cur_th)
        dist = math.sqrt(dy*dy + dx*dx)

        if not self.facing_goal and abs(d_angle_goal) < THETA_TOL:
            self.facing_goal = True
            self.last_facing_time = rospy.Time.now()
        if not self.at_goal:
            self.at_goal = abs(dist) < POS_TOL

        # Intially, try to rotate to face the goal position
        d_angle = d_angle_goal

        t_since = (rospy.Time.now() - self.last_facing_time).to_sec()
        if t_since > TURN_DELAY and (self.facing_goal or self.at_goal):
            if self.at_goal:                     # Reached goal position
                # Find desired heading angle
                goal_q = nav_goal.pose.orientation
                nav_th = 2*math.atan2(goal_q.z, goal_q.w)

                # Turn towards desired heading
                d_angle = angle_diff(nav_th, cur_th)
                if abs(d_angle) < THETA_TOL:
                    d_angle = 0
                    self.finished = True
            else:                               # Drive towards goal position
                # Compute desired velocity and omega
                vd = LAM_FORW * dist
                vd = min(VEL_TOL, max(-VEL_TOL, vd))    # Clamp
                vx = vd * math.cos(d_angle_goal)    

        # Compute rotating speed from angle_diff
        wz = LAM_TURN * d_angle
        wz = min(OMEGA_LIM, max(-OMEGA_LIM, wz))        # Clamp

        return vx, wz


#
#   LocalDriver Object
#
class LocalDriver:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.controller = DriveTurn()
        self.last_scan = LaserScan()

        # Create a publisher to send velocity commands.
        self.pub_vcmd = rospy.Publisher('/vel_cmd', Twist,
                                        queue_size=10)

        # Create a subscriber to listen to map pose.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_nav_goal)

        # Create a subscriber to listen to the lase scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

    def cb_nav_goal(self, msg: PoseStamped):
        assert (msg.header.frame_id == "map"), "Nav goal not in map frame"
        self.nav_goal = msg
        self.controller.reset()

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_timer(self, event):
        if not self.controller.finished:
            vx, wz = self.controller.update(self.last_pose, self.nav_goal, self.last_scan)

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

    # Instantiate the Local Driver object
    local_driver = LocalDriver()

    # Create the timer.
    timer = rospy.Timer(duration, local_driver.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Local Driver running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Local Driver stopped.")
