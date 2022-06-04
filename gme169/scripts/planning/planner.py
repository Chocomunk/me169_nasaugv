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
from inspect import trace
import rospy
import traceback

from nav_msgs.msg       import OccupancyGrid
from geometry_msgs.msg  import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg       import Bool

from planning_algorithms import AStarPlan


POS_TOL = 0.15              # (meters)
OBSTRUCTED_TIMEOUT = 3      # (sec)


#
#   Planner Object
#
class Planner:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.obstructed = False
        self.last_map = None

        # The last timestamp when we were not obstructed
        self.last_free_time = rospy.Time.now()

        self.planner = AStarPlan()
        self.reached_goal = True
        self.waypts = []

        # Create a publisher to send waypoints.
        self.pub_wpinter = rospy.Publisher('/waypoint_intermediate', PoseStamped,
                                        queue_size=10)
        self.pub_wpfinal = rospy.Publisher('/waypoint_final', PoseStamped,
                                        queue_size=10)
        self.pub_waypmap = rospy.Publisher('/waypmap', Marker,
                                        queue_size=10)

        # Create a subscriber to listen to robot pose in map.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to navigation goal.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_nav_goal)

        # Create a subscriber to listen to the map update.
        rospy.Subscriber('/map', OccupancyGrid, self.cb_map)

        # Create a subscriber to listen to the obstruction state.
        rospy.Subscriber('/obstructed', Bool, self.cb_obstructed)

    def plan_search(self, start: PoseStamped, end: PoseStamped):
        s_x = start.pose.position.x
        s_y = start.pose.position.y

        e_x = end.pose.position.x
        e_y = end.pose.position.y

        try:
            waypts = self.planner.search((s_x, s_y), (e_x, e_y))
            self.waypts = waypts.tolist()[::3]
        except Exception:
            traceback.print_exc()
            waypts = []

        if self.waypts:
            self.pub_path_pts(waypts)
            self.pub_wayp(self.waypts[0])
            self.reached_goal = False
        else:
            self.reached_goal = True

    def cb_nav_goal(self, msg: PoseStamped):
        assert (msg.header.frame_id == "map"), "Nav goal not in map frame"
        self.nav_goal = msg
        self.plan_search(self.last_pose, msg)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_map(self, msg: OccupancyGrid):
        self.last_map = msg
        self.planner.update_map(msg)

    def cb_obstructed(self, msg: Bool):
        self.obstructed = msg.data
        print(self.obstructed,
            (rospy.Time.now() - self.last_free_time).to_sec())
        if not msg.data:
            self.last_free_time = rospy.Time.now()

    def cb_timer(self, event):
        if not self.reached_goal:
            if self.obstructed and \
                (rospy.Time.now() - self.last_free_time).to_sec() > OBSTRUCTED_TIMEOUT:
                self.plan_search(self.last_pose, self.nav_goal)
                return

            px = self.last_pose.pose.position.x
            py = self.last_pose.pose.position.y
            gx, gy = self.waypts[0]
            
            dx = gx - px
            dy = gy - py

            if dx*dx + dy*dy < POS_TOL*POS_TOL:
                self.waypts.pop(0)
                if len(self.waypts) <= 0:
                    self.pub_wpfinal.publish(self.nav_goal)
                    self.reached_goal = True
                else:
                    self.pub_wayp(self.waypts[0])

    # TODO: set timestamp
    def pub_wayp(self, point):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        self.pub_wpinter.publish(msg)

    def pub_path_pts(self, path):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        s = self.last_map.info.resolution
        msg.scale.x = s
        msg.scale.y = s
        msg.scale.z = s

        msg.color.a = 1.0
        msg.color.r = 1
        msg.color.g = 1
        msg.color.b = 0

        pts = []
        for i in range(len(path)):
            pt = Point()
            pt.x = path[i,0]
            pt.y = path[i,1]
            pts.append(pt)
        
        msg.points = pts
        self.pub_waypmap.publish(msg)

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('planner')

    # Define durations
    duration = rospy.Duration(1. / 5)       # 5 Hz
    dt       = duration.to_sec()

    # Instantiate the Planner object
    planner = Planner()

    # Create the timer.
    timer = rospy.Timer(duration, planner.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Planner stopped.")
