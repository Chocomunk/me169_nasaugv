import math
import rospy
import tf2_ros
import numpy as np

from copy import deepcopy

from geometry_msgs.msg  import PoseStamped, TransformStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg       import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker
from sensor_msgs.msg    import LaserScan


CORR_M = 1.1
CORR_B = 0.162
MAX_DIST_FROM_ROBOT = 4.1
EPSILON = 1e-5

UNKNOWN = -1
OCC_THRESH = 0.65
FREE_THRESH = 0.196


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
        map_grid[map_grid == UNKNOWN] = 1
        self.grid = maxpool(map_grid, (K,K))
        self.w, self.h = self.grid.shape

        # TODO: compute prior and initialize logodds state

        # Setup map info
        self.info = MapMetaData() 
        self.info.map_load_time = self.map.info.map_load_time
        self.info.resolution = self.map.info.resolution * K
        self.info.width = self.w
        self.info.height = self.h
        self.info.origin = self.map.info.origin

        # State Variables
        self.last_pose = PoseStamped()
        self.last_scan = LaserScan()

        # -------------------- Publishers/Subscribers --------------------

        # Create a publisher to map space pose.
        self.pub_occ = rospy.Publisher('/occupy', OccupancyGrid,
                                        queue_size=10)

        # Create a subscriber to listen to odometry.
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)

        # Create a subscriber to listen to the laser scan.
        rospy.Subscriber('/scan', LaserScan, self.cb_laser, queue_size=1)
        # rospy.Subscriber('/lasermap', Marker, self.cb_laser, queue_size=1)

    def cb_laser(self, msg: LaserScan):
        self.last_scan = msg

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def cb_timer(self, event):



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
