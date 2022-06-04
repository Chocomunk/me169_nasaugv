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
import rospy

from geometry_msgs.msg  import PoseStamped, Twist
from sensor_msgs.msg    import LaserScan
from std_msgs.msg       import Bool, Empty

from drive_planners import DriveTurn, TurnDriveTurn


#
#   LocalDriver Object
#
class LocalDriver:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.last_scan = LaserScan()

        self.controller = DriveTurn(self.pub_finish, self.pub_obstructed)

        # Create a publisher to send velocity commands.
        self.pub_vcmd = rospy.Publisher('/vel_cmd', Twist,
                                        queue_size=10)
        self.pub_drive_state = rospy.Publisher('/drive_finish', Empty,
                                        queue_size=10)
        self.pub_obs = rospy.Publisher('/obstructed', Bool,
                                        queue_size=10)

        # Create a subscriber to listen to map pose.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/waypoint_intermediate', PoseStamped, self.cb_nav_goal, (False,))
        rospy.Subscriber('/waypoint_final', PoseStamped, self.cb_nav_goal, (True,))

        # Create a subscriber to listen to the lase scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

    def cb_nav_goal(self, msg: PoseStamped, align=True):
        assert (msg.header.frame_id == "map"), "Nav goal not in map frame"
        self.nav_goal = msg
        self.controller.reset(align)

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

    def pub_finish(self):
        self.pub_drive_state.publish(Empty())

    # Consider publishing this slower on a different timer
    def pub_obstructed(self, obstructed):
        msg = Bool()
        msg.data = obstructed
        self.pub_obs.publish(msg)


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
