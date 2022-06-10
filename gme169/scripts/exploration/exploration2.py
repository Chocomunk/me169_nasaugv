#!/usr/bin/env python3

import rospy

from collections import deque

from nav_msgs.msg       import OccupancyGrid, MapMetaData
from geometry_msgs.msg  import PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan
import numpy as np

from occupancy_map import OccupancyMap


PTRUE = 0.95 # Probability that the occupancy reading is correct


def log(x):
    return np.log(x, where=x>0)


# def neighborhood(i, h, w, s=1):
#     """ Return list of ((row, col), dist) neighbors """
#     r, c = i
#     d = np.sqrt(2) * s
#     neighbors = [
#         # ((r+s, c+s), d), ((r-s, c-s), d), ((r-s, c+s), d), ((r+s, c-s), d), 
#         ((r+s, c), s),   ((r-s, c), s),   ((r, c+s), s),   ((r, c-s), s)
#     ]
#     return [((r,c), d) for (r,c), d in neighbors 
#             if (0 <= r < h) and (0 <= c < w)]


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

        self.pub_waypmap = rospy.Publisher('/seegoal', Marker,
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

    # def value_iter(self, V, I, coords, out):
    #     h, w = out.shape
    #     for i in coords:
    #         if I[i] > 0:
    #             out[i] = I[i]
    #         else:
    #             out[i] = max(V[j] - d for j, d in neighborhood(i, h, w))

    # def value_iter(self, VTm1, I, coords):
    #     h,w = VTm1.shape
    #     VT = np.zeros(VTm1.shape)
    #     for i in coords:
    #         if I[i] > 0:
    #             VT[i] = I[i]
    #         else:
    #             VT[i] = max(VTm1[j] - d for j, d in neighborhood(i, h, w))
    #     return VT 

    def reachable_coords(self, occ, start):
        h, w = occ.shape
        q = deque([start])
        reachable = set([start])

        while q:
            coord = q.popleft()
            r, c = coord
            for child in [(r+1,c), (r,c+1), (r-1,c), (r,c-1)]:
                p = occ[child]
                if not child in reachable and 0 <= p < 65:
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
        VTm1 = np.copy(binary)

        pos = self.last_pose.pose.position
        start = (pos.x, pos.y)
        start_coord = tuple(reversed(occ_map.to_grid(np.array(start))))
        print(p)

        # explored = list(zip(*np.where((p >= 0) & (p < 25))))
        explored = self.reachable_coords(p, start_coord)
        # print(explored)
        for v in explored:
            if type(v) != tuple:
                print(type(v))

        print("Start compute")
        start_t = rospy.Time.now()
        k = 0
        same = False
        VT = np.zeros(VTm1.shape)

        h,w = VTm1.shape
        while not same:
        #for iter in np.arange(10):
            print(k)
            # VT = self.value_iter(VTm1, binary, explored)
            for i in explored:
                r,c = i
                if binary[i] > 0:
                    VT[i] = binary[i]
                else:
                    VT[i] = max(VTm1[r,c-1]-1, VTm1[r+1,c]-1, VTm1[r,c+1]-1, VTm1[r-1,c]-1)
            same = np.array_equal(VT, VTm1) # (VT == VTm1).all()
            VTm1 = VT[:,:]
            k += 1
        print("End compute " + str((rospy.Time.now() - start_t).to_sec()))
        rmin = np.min(VT) # minimum measurement
        VT = VT + rmin + 1
        rmin = 0
        rmax = np.max(VT) # maximum measurement
        tmin = 0 # minimum range of scaling
        tmax = 100 # max range of scaling
        vtmap = ((VT - rmin)/(rmax - rmin))*(tmax-tmin) + tmin

        ipub.data = (vtmap).astype(np.uint8).flatten().tobytes()
        self.pub_igain.publish(ipub)
        print("published")

        # for i in explored:
        #     cur_max = 0
        #     if VT[i] > cur_max:
        #         cur_max = VT[i]
        #         r, c = i
        #         goal_coords = np.array([r,c])

        goal_coords = np.array(list(reversed(np.unravel_index(np.argmax(VT), VT.shape))))
        goal = occ_map.to_map(goal_coords)
        goal = np.array([goal[0], goal[1]])
        print(goal_coords, goal)

        self.pub_exp(goal)

        pointes = np.zeros((8,2))
        pointes[0,0], pointes[0,1] = goal[0], goal[1]
        # pointes[1,0], pointes[0,1] = goal[0], goal[1]
        # pointes[2,0], pointes[0,1] = goal[0], goal[1]
        # pointes[3,0], pointes[0,1] = -goal[0], -goal[1]
        pointes[4,0], pointes[4,1] = goal[1], goal[0]
        # pointes[5,0], pointes[0,1] = goal[1], goal[0]
        # pointes[6,0], pointes[0,1] = goal[1], goal[0]
        # pointes[7,0], pointes[0,1] = -goal[1], -goal[0]
        self.pub_path_pts(pointes)

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
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 1

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