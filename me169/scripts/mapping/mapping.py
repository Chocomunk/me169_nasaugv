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


def inverse_model(gpos, pose: Pose, scan: LaserScan):
    pass


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
        # Get orientation as unit vec
        qz = self.last_pose.pose.orientation.z
        qw = self.last_pose.pose.orientation.w
        robot_t = 2*np.arctan2(qz, qw)
        c, s = np.cos(robot_t), np.sin(robot_t)
        ori = np.array([c,s])

        # Angle limits              
        ang_min = self.last_scan.angle_min
        ang_inc = self.last_scan.angle_increment
        ang_lim = np.cos((self.last_scan.angle_max - self.last_scan.angle_min) / 2)

        # Robot position
        rx = self.last_pose.pose.position.x
        ry = self.last_pose.pose.position.y

        # Range limit
        max_dist = self.last_scan.range_max
        max_dist2 = max_dist*max_dist                       # Pre-square

        # Occupancy tolerance
        pos_tol = self.res / 2
        ang_tol = ang_inc / 2

        # Laser data
        ranges = self.last_scan.ranges

        # Update occupancy
        for r in range(self.h):
            y = (r+0.5) * self.res
            dy = y-ry
            dy2 = dy*dy
            for c in range(self.w):
                x = (c+0.5) * self.res
                dx = x-rx
                dx2 = dx*dx

                # Check dot-product for angle and check total distance
                v = np.array([dx,dy])
                v_hat = v / np.linalg.norm(v)
                if dx2 + dy2 < max_dist2 and np.dot(v_hat, ori) > ang_lim:
                    r = np.sqrt(dx2 + dy2)
                    phi = np.atan2(dy, dx) - robot_t
                    k = int((phi - ang_min + ang_inc / 2) // ang_inc)   # Get laser index from angle
                    theta_k = k * ang_inc + ang_min
                    z = ranges[k]

                    # Compute inverse_range_sensor_model
                    value = 0
                    if r > min(max_dist, z + pos_tol) or abs(phi - theta_k) > ang_tol:
                        value = self.prior[r,c]
                    elif z < max_dist and abs(r - z) < pos_tol:
                        value = L_OCC
                    elif r <= z:
                        value = L_FREE

                    self.state[r,c] += value - self.prior[r,c]

                # Else, leave it the same

        # TODO: publish occupancy


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
