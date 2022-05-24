import rospy
import numpy as np

from geometry_msgs.msg  import PoseStamped, Pose
from nav_msgs.msg       import OccupancyGrid, MapMetaData
from sensor_msgs.msg    import LaserScan


CORR_M = 1.1
CORR_B = 0.162
MAX_DIST_FROM_ROBOT = 4.1
EPSILON = 1e-5

UNKNOWN = -1
OCC_THRESH = 0.65
FREE_THRESH = 0.196

L_OCC = np.log(1 - EPSILON)
L_FREE = np.log(EPSILON)


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
        """ Update the map and publish """
        # Robot pose
        rx = self.last_pose.pose.position.x         # Robot x-coord
        ry = self.last_pose.pose.position.y         # Robot y-coord
        qz = self.last_pose.pose.orientation.z
        qw = self.last_pose.pose.orientation.w
        rt = 2*np.arctan2(qz, qw)                   # Robot map-space orientation

        # Limits/Tolerances
        ang_min = self.last_scan.angle_min
        ang_inc = self.last_scan.angle_increment
        max_dist = self.last_scan.range_max
        pos_tol = self.res / 2

        # Laser data
        ranges = self.last_scan.ranges

        # TODO: only iterate through grid spaces that are in-range
        # Update occupancy
        for r in range(self.h):
            y = (r+0.5) * self.res                                  # Center of grid
            dy = y-ry                                               # Vec from robot
            for c in range(self.w):
                x = (c+0.5) * self.res                              # Center of grid
                dx = x-rx                                           # Vec from robot

                r = np.sqrt(dx*dx + dy*dy)                          # Dist to robot
                phi = np.atan2(dy, dx) - rt                         # Robot -> Grid angle
                k = int((phi - ang_min + ang_inc / 2) // ang_inc)   # Get laser index from angle

                # Check if grid is visible
                if 0 <= k < len(ranges) and r <= min(max_dist, ranges[k] + pos_tol):
                    # Compute inverse_range_sensor_model
                    z = ranges[k]                                   # Range reading
                    if z < max_dist and abs(r - z) < pos_tol:
                        self.state[r,c] += L_OCC - self.prior[r,c]
                    elif r <= z:
                        self.state[r,c] += L_FREE - self.prior[r,c]
                # Else: don't update logodds

        # TODO: publish occupancy
        probs = 1 - (1 / (1 + np.exp(self.state)))

        msg = OccupancyGrid()
        msg.info = self.info
        msg.data = probs.flatten().tobytes()
        self.pub_occ.publish(msg)


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
