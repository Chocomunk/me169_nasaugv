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
import heapq
import itertools

import rospy
import numpy as np

from nav_msgs.msg       import OccupancyGrid
from geometry_msgs.msg  import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan

from planar_transform import PlanarTransform


OCC_THRESH = 0.65
FREE_THRESH = 0.196
BOT_RAD = 0.1
POS_TOL = 0.15              # (meters)


class SearchError(Exception):
    pass


class TileStates:
    """
    Defines all possible tile states as enum values.
    """
    UNKNOWN   = 0
    WALL      = 1
    ONDECK    = 2
    PROCESSED = 3


class PriorityQueue:
    """
    Priority Queue implementation inspired by python3 heapq documentation: 
        https://docs.python.org/3/library/heapq.html
    """

    REMOVED = '<removed-task>'

    def __init__(self) -> None:
        self._data = []                      # Stores the internal heapq
        self.entry_finder = {}               # Maps element -> entry
        self.counter = itertools.count()     # Used for duplicate priorities

    def push(self, el, priority):
        """ Inserts an element into the priority queue """
        if el in self.entry_finder:
            self.remove(el)
        count = next(self.counter)
        entry = [priority, count, el]
        self.entry_finder[el] = entry
        heapq.heappush(self._data, entry)

    def remove(self, el):
        """ Marks an element as removed. Raise KeyError if not found. """
        entry = self.entry_finder.pop(el)
        entry[-1] = self.REMOVED

    def pop(self):
        """ Remove and return the lowest priority task. Raise KeyError if empty. """
        while self._data:
            priority, count, el = heapq.heappop(self._data)
            if el is not self.REMOVED:
                del self.entry_finder[el]
                return el
        raise KeyError('pop from an empty priority queue')

    def is_empty(self):
        """ Returns whether the priority queue is empty """
        return len(self.entry_finder) == 0


def euclidian(a, b):
    x = b[0] - a[0]
    y = b[1] - a[1]
    return np.sqrt(x*x + y*y)
    # return abs(b[1] - a[1]) + abs(b[0] - a[0])

def astar_cost(cost_to_reach, coord, end, manhattan_weight=1, *args, **kwargs):
    return cost_to_reach + euclidian(coord, end) * manhattan_weight


class AStarPlan:

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

        # Setup Grid
        self.w = self.map.info.width
        self.h = self.map.info.height
        self.grid = np.array(map_msg.data).reshape((self.h, self.w))
        self.state = np.zeros((self.h, self.w))
        self.state += TileStates.UNKNOWN
        self.state[self.grid > OCC_THRESH] = TileStates.WALL
        self.state[self.grid < 0] = TileStates.WALL

        # Setup transforms
        g_x = map_msg.info.origin.position.x
        g_y = map_msg.info.origin.position.y
        g_qz = map_msg.info.origin.orientation.z
        g_qw = map_msg.info.origin.orientation.w

        self.res = map_msg.info.resolution
        self.map2grid = PlanarTransform(g_x, g_y, g_qz, g_qw)

    def to_map(self, pts):
        return self.map2grid.apply(self.res * pts)
        
    def to_grid(self, pts):
        return (self.map2grid.inv().apply(pts) / self.res).round().astype(int)

    def search(self, start, end, cost_func=astar_cost):
        """
        Perform a discrete planning search over the given problem definition and
        defined cost function. The cost function should be able to take:
            cost_to_reach, current_coordinate, end_coordinate

        Returns: path A list storing the coordinates of the resulting path (ordered)
        """
        start = tuple(reversed(self.to_grid(np.array(start))))
        end = tuple(reversed(self.to_grid(np.array(end))))

        print(start, end)

        # Don't modify the original state
        state = np.copy(self.state)
        M, N = self.h, self.w                       # Get dimensions (row, col)

        cost_to_reach = np.ones((M,N)) * np.inf     # Initialize costs to inf
        parent = np.zeros((M,N,2), dtype=int) - 1   # Initialize parent table to (-1,-1)

        # --------------------- Priority Queue interaction -------------------------
        #   These inner functions will be used to more cleanly interact with the
        #   priority queue backend. Every push/pop operation requires the same
        #   supporting operations (e.g. updating cost and parents), so we wrap them
        #   in functions to make the main algorithm loop more readable

        q = PriorityQueue()         # Define the p-queue of ONDECK states

        def push(coord, c, par):
            """ Push an element `coord` with cost-to-reach `c` and parent `par` """
            s = state[coord]
            child_cost = cost_func(cost_to_reach=c, 
                                    coord=coord, start=start, end=end)

            # This tile was not encountered yet
            if s == TileStates.UNKNOWN:
                q.push(coord, child_cost)
                state[coord] = TileStates.ONDECK
                if par:
                    cost_to_reach[coord] = c
                    parent[coord] = par

            # This tile was seen in another path
            elif s == TileStates.ONDECK:
                # Newer path is lower cost, so update to new
                if c < cost_to_reach[coord]:
                    q.remove(coord)
                    q.push(coord, child_cost)
                    if par:
                        cost_to_reach[coord] = c
                        parent[coord] = par

            # else: do nothing

        def pop():
            """ Pop an element from the p-queue """
            coord = q.pop()
            if state[coord] == TileStates.WALL:
                print("Big problem")
            state[coord] = TileStates.PROCESSED
            return coord

        # ----------------------- Utility Inner Functions -------------------------- 

        def near_nodes(r, c, s):
            d = np.sqrt(2) * s
            return [
                    ((r+s, c+s), d), ((r-s, c-s), d), ((r-s, c+s), d), ((r+s, c-s), d), 
                    ((r+s, c), s),   ((r-s, c), s),   ((r, c+s), s),   ((r, c-s), s)
                    ]

        def next_to_wall(r, c):
            for coord, _ in near_nodes(r, c, math.ceil(BOT_RAD / self.res)):
                if state[coord] == TileStates.WALL:
                    return True

        def neighbors(coord, s=5):
            """ Returns all in-bound neighbors of a coord """
            r, c = coord
            total = near_nodes(r, c, s)
            filt = []
            for (r, c), a in total:
                if (0 <= r < M) and (0 <= c < N) and not next_to_wall(r, c):
                    filt.append(((r,c), a))
            return filt

        def inrange(a, b, s=5):
            x1, y1 = a
            x2, y2 = b
            return abs(x2 - x1) <= s+1 and abs(y2 - y1) <= s+1

        # ------------------------- Main Algorithm Loop ---------------------------- 

        # Initialize search
        push(start, 0, None)
        cost_to_reach[start] = 0

        # Perform search
        found = False
        while not q.is_empty():
            curr = pop()            # Get the next lowest-cost leaf
            if inrange(curr, end):          # If it is the goal, we finished!
                found = True
                break
            for child, c in neighbors(curr):   # Otherwise, check every child
                push(child, cost_to_reach[curr] + c, curr)

        if not found:
            raise SearchError("Could not reach the goal state.")

        # Walk the search path
        path = [curr]                      # Populated backwards from `end`
        curr = tuple(parent[curr])

        # Loop while the parent exists, the start node has no parent
        while (parent[curr] != [-1, -1]).any():
            path.append(curr)                       # Add to the path
            curr = tuple(parent[curr])              # Step to the next parent
        path = list(reversed(path))                 # Reverse the list into the right order.
        path = [(x, y) for y, x in path]

        return self.to_map(np.array(path))


#
#   Planner Object
#
class Planner:
    # Initialize.
    def __init__(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.mapmsg: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid, 30.0)
        
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.last_scan = LaserScan()

        self.planner = AStarPlan(self.mapmsg)
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
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)

    def cb_nav_goal(self, msg: PoseStamped):
        assert (msg.header.frame_id == "map"), "Nav goal not in map frame"
        self.nav_goal = msg

        s_x = self.last_pose.pose.position.x
        s_y = self.last_pose.pose.position.y

        e_x = msg.pose.position.x
        e_y = msg.pose.position.y

        waypts = self.planner.search((s_x, s_y), (e_x, e_y))
        self.waypts = waypts.tolist()
        print(self.waypts)
        if self.waypts:
            self.pub_path_pts(waypts)
            self.pub_wayp(self.waypts[0])
            self.reached_goal = False

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_timer(self, event):
        if not self.reached_goal:
            px = self.last_pose.pose.position.x
            py = self.last_pose.pose.position.y
            gx, gy = self.waypts[0]
            
            dx = gx - px
            dy = gy - py

            print(self.waypts)

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

        s = self.mapmsg.info.resolution
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
    duration = rospy.Duration(1. / 10)       # 10 Hz
    dt       = duration.to_sec()

    # Instantiate the Planner object
    planner = Planner()

    # Create the timer.
    timer = rospy.Timer(duration, planner.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Planner stopped.")
