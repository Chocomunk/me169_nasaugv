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
import numpy as np

from scipy.special import expit

from geometry_msgs.msg  import PoseStamped, Pose
from nav_msgs.msg       import OccupancyGrid, MapMetaData
from sensor_msgs.msg    import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg  import Point

from occupancy_map import MapTransform


# CORR_M = 1.1
# CORR_M = .9
# CORR_B = 0.162
# CORR_B = 0
CORR_A = -2.14e-3
CORR_B = 1.11
CORR_C = 0.154
MAX_DIST_FROM_ROBOT = 4.1
EPSILON = 1e-5

UNKNOWN = 0.5
OCC_THRESH = 0.65
FREE_THRESH = 0.196

L_FREE = np.log(EPSILON)
L_OCC = -L_FREE


def maxpool(mat, kernel_size):
    m, n = mat.shape
    ky, kx = kernel_size
    ny = int(np.ceil(m/float(ky)))
    nx = int(np.ceil(n/float(kx)))
    size = (ny*ky, nx*kx)
    mat_pad = np.full(size,np.nan)
    mat_pad[:m,:n] = mat
    new_shape = (ny,ky,nx,kx)
    return np.nanmax(mat_pad.reshape(new_shape),axis=(1,3))


def AngleDiff(t1: float, t2: float) -> float:
    return (t1-t2) - 2.0*np.pi * round(0.5*(t1-t2)/np.pi)


K = 5


#
#   Mapping Object
#
class Mapping:
    # Initialize.
    def __init__(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.map: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid, 30.0)

        # Setup Grid
        mw = self.map.info.width
        mh = self.map.info.height
        map_grid = np.array(self.map.data).reshape((mh, mw))
        self.grid = maxpool(map_grid, (K,K)) / 100
        self.grid[self.grid < 0] = UNKNOWN
        self.grid[self.grid > OCC_THRESH] = OCC_THRESH
        self.grid[self.grid < FREE_THRESH] = FREE_THRESH
        self.w, self.h = self.grid.shape
        self.res = self.map.info.resolution * K

        # Compute prior and initialize logodds state
        self.prior = np.log(self.grid / (1 - self.grid))
        self.state = self.prior.copy()      # Logodds of occupancy belief

        # Setup map info
        self.info = MapMetaData() 
        self.info.map_load_time = self.map.info.map_load_time
        self.info.resolution = self.res
        self.info.width = self.w
        self.info.height = self.h
        self.info.origin = self.map.info.origin

        # Grid-to-map coords 
        self.map_tf: MapTransform = MapTransform(self.info)
        pts = np.array([(c+0.5,r+0.5) for r in range(self.h) for c in range(self.w)])
        self.map_pts = self.map_tf.to_map(pts).reshape((self.h, self.w, 2))

        # State Variables
        self.last_pose = PoseStamped()
        self.last_scan = LaserScan()

        # -------------------- Publishers/Subscribers --------------------

        # Create a publisher to map space pose.
        self.pub_occ = rospy.Publisher('/occupy', OccupancyGrid,
                                        queue_size=10)
        self.pub_gridpts = rospy.Publisher('/gridpts', Marker,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser, queue_size=1)
        # rospy.Subscriber('/lasermap', Marker, self.cb_laser, queue_size=1)

        self.publish_occupy(self.grid)
        self.pub_grid(self.map_pts)

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_timer(self, event):
        """ Update the map and publish """
        if abs((rospy.Time.now() - self.last_scan.header.stamp).to_sec()) > 1:
            print("SKIPPING")
            return

        # Robot pose
        rx = self.last_pose.pose.position.x         # Robot x-coord
        ry = self.last_pose.pose.position.y         # Robot y-coord
        qz = self.last_pose.pose.orientation.z
        qw = self.last_pose.pose.orientation.w
        rt = 2*np.arctan2(qz, qw)                   # Robot map-space orientation

        # Limits/Tolerances
        ang_min = self.last_scan.angle_min
        ang_inc = self.last_scan.angle_increment
        # max_dist = self.last_scan.range_max
        max_dist = 2.5
        pos_tol = self.res / 2

        # Laser data
        ranges = np.array(self.last_scan.ranges)
        # ranges = CORR_A * np.square(ranges) + CORR_B * ranges + CORR_C
        # ranges = np.array(self.last_scan.ranges) * CORR_M + CORR_B

        # TODO: only iterate through grid spaces that are in-range
        # Update occupancy
        for r in range(self.h):
            # y = (r+0.5) * self.res                                  # Center of grid
            for c in range(self.w):
                # x = (c+0.5) * self.res                              # Center of grid
                x, y = self.map_pts[r, c]
                dx = x-rx                                           # Vec from robot
                dy = y-ry                                           # Vec from robot

                d = np.sqrt(dx*dx + dy*dy)                          # Dist to robot
                phi = np.arctan2(dy, dx) - rt + np.pi               # Robot -> Grid angle
                ang_diff = AngleDiff(phi + ang_inc / 2, ang_min)
                k = int(ang_diff // ang_inc)   # Get laser index from angle

                # TODO: Clamp max occ and free
                # Check if grid is visible
                if 0 <= k < len(ranges) and d <= min(max_dist, ranges[k] + pos_tol):
                    # Compute inverse_range_sensor_model
                    z = ranges[k]                                   # Range reading
                    if z < max_dist and abs(d - z) <= pos_tol:
                        shift = L_OCC - self.prior[r,c]
                        self.state[r,c] = min(L_OCC, self.state[r,c] + shift)
                    else:
                        shift = L_FREE - self.prior[r,c]
                        self.state[r,c] = max(L_FREE, self.state[r,c] + shift)
                # Else: don't update logodds

        # TODO: publish occupancy
        probs = (expit(self.state) * 100).astype(np.uint8)
        self.publish_occupy(probs)
    
    def publish_occupy(self, probs):
        msg = OccupancyGrid()
        msg.info = self.info
        msg.data = probs.flatten().tobytes()
        self.pub_occ.publish(msg)
        # self.pub_grid(self.map_pts)

    def pub_grid(self, gridpts):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        s = 0.02
        msg.scale.x = s
        msg.scale.y = s
        msg.scale.z = s

        msg.color.a = 1.0
        msg.color.r = 1
        msg.color.g = 0
        msg.color.b = 1

        pts = []
        for i in range(self.h):
            for j in range(self.w):
                pt = Point()
                pt.x = gridpts[i,j,0]
                pt.y = gridpts[i,j,1]
                pts.append(pt)
        
        msg.points = pts
        self.pub_gridpts.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('mapping')

    # Define durations
    duration = rospy.Duration(1. / 1)       # 1 Hz
    dt       = duration.to_sec()

    # Instantiate the Odometry object
    localization = Mapping()

    # Create the timer.
    timer = rospy.Timer(duration, localization.cb_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Mapping running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Mapping stopped.")
