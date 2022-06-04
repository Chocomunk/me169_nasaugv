#!/usr/bin/env python3

import rospy

from collections import deque

from nav_msgs.msg       import OccupancyGrid, MapMetaData
from geometry_msgs.msg  import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan
import numpy as np

from occupancy_map import OccupancyMap


PTRUE = 0.95 # Probability that the occupancy reading is correct


def log(x):
    return np.log(x, where=x>0)


def neighborhood(i, h, w, s=1):
    """ Return list of ((row, col), dist) neighbors """
    r, c = i
    d = np.sqrt(2) * s
    neighbors = [
        # ((r+s, c+s), d), ((r-s, c-s), d), ((r-s, c+s), d), ((r+s, c-s), d), 
        ((r+s, c), s),   ((r-s, c), s),   ((r, c+s), s),   ((r, c-s), s)
    ]
    return [((r,c), d) for (r,c), d in neighbors 
            if (0 <= r < h) and (0 <= c < w)]


#
#   Explorer Object
#
class Explorer:
    # Initialize.
    def __init__(self):
        # Define Variables
        self.last_pose = PoseStamped()
        self.nav_goal = PoseStamped()
        self.last_map = None

        # Create a publisher to send the navigation goal to the planner.
        self.pub_wpinter = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                                        queue_size=10)
        # Create a publisher to send the selected exploration point
        self.pub_exppt = rospy.Publisher('/exppt', PoseStamped,
                                        queue_size=10)
        # Create a publisher to send the selected exploration point
        self.pub_igain = rospy.Publisher('/igain', OccupancyGrid,
                                        queue_size=10)

        # Create a subscriber to listen to robot pose in map.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to the map.
        rospy.Subscriber('/map', OccupancyGrid, self.cb_map)

    def eig(self, p):
        # Computes information gain of a cell with probability p
        q = 1-p
        Hp = - p * log(p) - q * log(q) # entropy
        return Hp

        # ppt = p * PTRUE
        # pqt = p * (1-PTRUE)
        # qpt = q * PTRUE
        # qqt = q * (1-PTRUE)
        # EHp = - ppt * log(ppt / (ppt + qqt)) - qqt * log(qqt / (ppt + qqt)) \
        #     - pqt * log(pqt / (qpt + pqt)) - qpt * log(qpt / (qpt + pqt))
        # return Hp - EHp

    def value_iter(self, V, I, coords, out):
        h, w = out.shape
        for i in coords:
            if I[i] > 0:
                out[i] = I[i]
            else:
                out[i] = max(V[j] - d for j, d in neighborhood(i, h, w))

    def reachable_coords(self, occ, start):
        h, w = occ.shape
        q = deque([start])
        reachable = set([start])

        while q:
            coord = q.popleft()
            for child, _ in neighborhood(coord, h, w):
                p = occ[child]
                if not child in reachable and 0 <= p < 0.65:
                    reachable.add(child)
                    q.append(child)

        return list(reachable)

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_map(self, msg: OccupancyGrid):
        self.last_map = msg
        occ_map = OccupancyMap(msg)

        w, h = msg.info.width, msg.info.height
        p = np.array(msg.data).reshape((h,w))

        ipub = OccupancyGrid()
        ipub.info = self.last_map.info

        binary = np.copy(p)
        binary[p < 0] = 100
        binary[p >= 0] = 0
        v1 = np.copy(binary)
        v2 = np.zeros(binary.shape)

        pos = self.last_pose.pose.position
        start = (pos.x, pos.y)
        start_coord = tuple(reversed(occ_map.to_grid(np.array(start))))
        # explored = list(zip(*np.where((p >= 0) & (p < 0.65))))
        explored = self.reachable_coords(p, start_coord)
        # print(explored)
        for v in explored:
            if type(v) != tuple:
                print(type(v))

        print("Start compute")
        start_t = rospy.Time.now()
        k = 0
        same = False
        while not same:
            print(k)
            self.value_iter(v1, binary, explored, v2)
            same = (v1 == v2).all()
            v1 = v2[:,:]
            k += 1
        print("End compute " + str((rospy.Time.now() - start_t).to_sec()))

        ipub.data = (v2).astype(np.uint8).flatten().tobytes()
        self.pub_igain.publish(ipub)
        print("published")

        goal_coords = np.array(list(reversed(np.unravel_index(np.argmax(v2), v2.shape))))
        goal = occ_map.to_map(goal_coords)
        print(goal_coords, goal)

        self.pub_exp(goal)

        # CODE BELOW FOR PUBLSIHING POSESTAMPED MSG
        # self.nav_goal = msg

        # s_x = self.last_pose.pose.position.x
        # s_y = self.last_pose.pose.position.y

        # e_x = msg.pose.position.x
        # e_y = msg.pose.position.y

        # waypts = self.planner.search((s_x, s_y), (e_x, e_y))
        # self.waypts = waypts.tolist()[::3]
        # if self.waypts:
        #     self.pub_path_pts(waypts)
        #     self.pub_wayp(self.waypts[0])
        #     self.reached_goal = False

    def pub_exp(self, point):
        #TODO - Fix to publish a Point
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        self.pub_exppt.publish(msg)

        self.pub_wpinter.publish(msg)

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('exploration')

    # Define durations
    duration = rospy.Duration(1. / 5)       # 5 Hz
    dt       = duration.to_sec()

    # Instantiate the Planner object
    explorer = Explorer()

    # # Create the timer.
    # timer = rospy.Timer(duration, explorer.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Explorer running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Explorer stopped.")