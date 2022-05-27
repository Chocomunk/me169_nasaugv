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

from nav_msgs.msg       import OccupancyGrid
from geometry_msgs.msg  import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan

from planning_algorithms import AStarPlan


POS_TOL = 0.15              # (meters)


#
#   Planner Object
#
class Planner:
    # Initialize.
    def __init__(self):
        # # Wait 30sec for a map.
        # rospy.loginfo("Waiting for a map...")
        # self.mapmsg: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid, 30.0)
        
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.last_map = None

        self.planner = AStarPlan()
        self.reached_goal = True
        self.waypts = []

        # TODO: publish waypoint
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

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/occupy', OccupancyGrid, self.cb_occupy)

    def cb_nav_goal(self, msg: PoseStamped):
        assert (msg.header.frame_id == "map"), "Nav goal not in map frame"
        self.nav_goal = msg

        s_x = self.last_pose.pose.position.x
        s_y = self.last_pose.pose.position.y

        e_x = msg.pose.position.x
        e_y = msg.pose.position.y

        waypts = self.planner.search((s_x, s_y), (e_x, e_y))
        self.waypts = waypts.tolist()
        if self.waypts:
            self.pub_path_pts(waypts)
            self.pub_wayp(self.waypts[0])
            self.reached_goal = False

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_occupy(self, msg: OccupancyGrid):
        self.last_map = msg
        self.planner.update_map(msg)

    def cb_timer(self, event):
        if not self.reached_goal:
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
