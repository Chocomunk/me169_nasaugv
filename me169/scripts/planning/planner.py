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

from util.planar_transform import PlanarTransform


OCC_THRESH = 0.65
FREE_THRESH = 0.196


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


def manhattan(a, b):
    return abs(b[1] - a[1]) + abs(b[0] - a[0])

def step_cost(parent_cost, *args, **kwargs):
    return parent_cost + 1

def astar_cost(cost_to_reach, coord, end, manhattan_weight=1, *args, **kwargs):
    return cost_to_reach + manhattan(coord, end) * manhattan_weight


class AStarPlan:

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

        # Setup Grid
        self.grid = map_msg.data
        self.w = self.map.info.width
        self.h = self.map.info.height
        self.state = np.array(self.grid).reshape(self.h, self.w)
        self.state[self.state > OCC_THRESH] = 1
        self.state[self.state < FREE_THRESH] = 0

        # Setup transforms
        g_x = map_msg.info.origin.position.x
        g_y = map_msg.info.origin.position.y
        g_qz = map_msg.info.origin.orientation.z
        g_qw = map_msg.info.origin.orientation.z

        self.res = map_msg.info.resolution
        self.map2grid = PlanarTransform(g_x, g_y, g_qz, g_qw)

    def to_map(self, pts):
        return np.round(self.map2grid.apply(pts) / self.res)
        
    def to_grid(self, pts):
        return self.map2grid.inv().apply(self.res * pts)

    def search(self, start, end, cost_func=astar_cost):
        """
        Perform a discrete planning search over the given problem definition and
        defined cost function. The cost function should be able to take:
            cost_to_reach, current_coordinate, end_coordinate

        Returns: path A list storing the coordinates of the resulting path (ordered)
        """
        start = self.to_grid(np.array(start))
        end = self.to_grid(np.array(end))

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
            """ Push an element `coord` with cost `c` and parent `par` """
            s = state[coord]

            # This tile was not encountered yet
            if s == TileStates.UNKNOWN:
                q.push(coord, c)
                state[coord] = TileStates.ONDECK
                if par:
                    cost_to_reach[coord] = step_cost(cost_to_reach[par])
                    parent[coord] = par

            # This tile was seen in another path
            elif s == TileStates.ONDECK:
                # Newer path is lower cost, so update to new
                if c < cost_to_reach[coord]:     
                    q.remove(coord)
                    q.push(coord, c)
                    if par:
                        cost_to_reach[coord] = step_cost(cost_to_reach[par])
                        parent[coord] = par

            # else: do nothing

        def pop():
            """ Pop an element from the p-queue """
            coord = q.pop()
            state[coord] = TileStates.PROCESSED
            return coord

        # ----------------------- Utility Inner Functions -------------------------- 

        def neighbors(coord):
            """ Returns all in-bound neighbors of a coord """
            r, c = coord
            total = [(r+1, c+1), (r-1, c-1), (r-1, c+1), (r+1, c-1), 
                     (r+1, c),   (r-1, c),   (r, c+1),   (r, c-1)]
            return [(r,c) for r,c in total if (0 <= r < M) and (0 <= c < N)]

        # ------------------------- Main Algorithm Loop ---------------------------- 

        # Initialize search
        push(start, 0, None)
        cost_to_reach[start] = 0

        # Perform search
        found = False
        while not q.is_empty():
            curr = pop()            # Get the next lowest-cost leaf
            if curr == end:         # If it is the goal, we finished!
                found = True
                break
            for child in neighbors(curr):   # Otherwise, check every child
                child_cost = cost_func(cost_to_reach=step_cost(cost_to_reach[curr]), 
                                        coord=child, start=start, end=end)
                push(child, child_cost, curr)

        if not found:
            raise SearchError("Could not reach the goal state.")

        # Walk the search path
        path = [end]                                # Populated backwards from `end`
        curr = tuple(parent[end])

        # Loop while the parent exists, the start node has no parent
        while (parent[curr] != [-1, -1]).any():
            path.append(curr)                       # Add to the path
            curr = tuple(parent[curr])              # Step to the next parent
        path = list(reversed(path))                 # Reverse the list into the right order.

        return self.to_map(np.array(path))


#
#   Planner Object
#
class Planner:
    # Initialize.
    def __init__(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message("/map", OccupancyGrid, 30.0)
        
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.last_scan = LaserScan()

        self.planner = AStarPlan(self.mapmsg)

        # TODO: publish waypoint
        # Create a publisher to send waypoints.
        self.pub_waypoint = rospy.Publisher('/waypoint', PoseStamped,
                                        queue_size=10)

        self.pub_wapmap = rospy.Publisher('/waypmap', Marker,
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
        self.pub_path_pts(waypts)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_timer(self, event):
        pass

    def pub_path_pts(self, path):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        s = 0.05
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
        self.pub_wapmap.publish(msg)

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