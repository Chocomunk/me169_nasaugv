import math
import rospy
import numpy as np
from scipy.spatial import cKDTree

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg  import Point
from sensor_msgs.msg    import LaserScan

from planar_transform import PlanarTransform


WALLTHRESHOLD = 65      # Believe probability TODO: read from map yaml
MAXDISTANCE = 0.2         # Distance to wall
FRACTION = 0.05            # Correction dampening
POS_LIM = 0.3           # Limit for position correction (meters)
THETA_LIM = np.pi / 12  # Limit for angle correction (radians)

MAX_DIST_FROM_ROBOT = 2.5
EPSILON = 1e-5


def wall_points(mapgrid: OccupancyGrid):
    w, h = mapgrid.info.width, mapgrid.info.height

    map_arr = np.array(mapgrid.data).reshape((h,w))
    wallpts = np.zeros((0,2), dtype=np.int)

    for v in range(h):
        for u in range(w):
            if map_arr[v,u] > WALLTHRESHOLD:
                # Also check the adjacent pixels in a 3x3 grid.
                adjacent = map_arr[max(0,v-1):min(h,v+2), max(0,u-1):min(w,u+2)]
                if not np.all(adjacent > WALLTHRESHOLD):
                    wallpts = np.vstack([wallpts, np.array([u,v])])

    g_x = mapgrid.info.origin.position.x
    g_y = mapgrid.info.origin.position.y
    g_qz = mapgrid.info.origin.orientation.z
    g_qw = mapgrid.info.origin.orientation.w
    res = mapgrid.info.resolution

    g_o = np.array([g_x, g_y])
    g_t = 2 * math.atan2(g_qz, g_qw)
    c, s = np.cos(g_t), np.sin(g_t)
    rotmat = np.array([[c, -s], [s, c]])

    map_wallpts = g_o + res * wallpts @ rotmat.T

    return map_wallpts


def laser2cart(scan: LaserScan):
    """ Returns a tuple (pts, weights) """
    # r = np.array(scan.ranges) * CORR_M + CORR_B
    r = np.array(scan.ranges)
    # r = CORR_A * np.square(r) + CORR_B * r + CORR_C
    t = np.linspace(scan.angle_min, scan.angle_max, len(r))
    idxs = np.where(r < MAX_DIST_FROM_ROBOT)
    r = r[idxs]
    t = t[idxs]
    return r[:,None] * np.array((np.cos(t), np.sin(t))).T, (1 / (r + EPSILON)).flatten()


#
#   Localization Correction
#
class IdentityCorrection:
    # Initialize.
    def __init__(self, map: OccupancyGrid):
        self.map = map      # Not needed, but store for reference
        self.identity_tf = PlanarTransform.basic(0, 0, 0)
    
    def update(self, scan: LaserScan, map2laser: PlanarTransform):
        # Do nothing for now, keep returning the identity transform
        return self.identity_tf


class BasicLeastSquaresCorrection:

    def __init__(self, mapgrid: OccupancyGrid):
        self.map = mapgrid
        wallpts = wall_points(mapgrid)
        self.walltree = cKDTree(wallpts)
        self.pub_wallpts = rospy.Publisher('/wallpts', Marker,
                                        queue_size=10)
        self.pub_lasmap = rospy.Publisher('/lasermap', Marker,
                                        queue_size=10)
    
    def update(self, scan: LaserScan, map2laser: PlanarTransform):
        laserpts, weights = laser2cart(scan)
        laserpts = map2laser.apply(laserpts)

        idxs, wallnear = self.nearest_wallpts(laserpts)
        pts = laserpts[idxs]
        self.pub_wall(wallnear)
        self.pub_laser_map(pts, scan)

        # wts = weights[idxs]
        # dbl_wts = np.tile(wts, (2,1)).swapaxes(0,1).flatten()
        # lam = np.diag(dbl_wts)

        lam = np.diag(weights[idxs])

        lam *= len(idxs) / len(scan.ranges)
        lam = np.square(lam)

        # r = pts.flatten()
        # p = wallnear.flatten()
        # a = p-r

        # rhs = (np.flip(pts, axis=1) * np.array([[-1, 1]])).flatten()[:,None]
        # lhs = np.tile(np.eye(2), (len(pts), 1))
        # J = np.hstack((lhs, rhs))

        r = pts
        p = wallnear
        a = np.linalg.norm(p-r, axis=1)[:,None]

        lhs = p-r
        rhs = np.sum(r * np.flip(p, axis=1) * np.array([[1, -1]]), axis=1)[:,None]
        J = np.divide(np.hstack((lhs, rhs)), a)

        b = J.T @ lam
        J_dag = np.linalg.pinv(b @ J) @ b
        d = (J_dag @ a).flatten()
        assert (d.shape == (3,)), "Wrong delta dimensions: {0} != (3,)".format(d.shape)

        m = np.linalg.norm(d)
        if m > POS_LIM:
            d *= POS_LIM / m
        x, y, t = d

        return FRACTION * PlanarTransform.basic(x, y, t)

    def nearest_wallpts(self, pts):
        dists, idxs = self.walltree.query(pts)

        inrange = np.where(dists < MAXDISTANCE)
        idxs = idxs[inrange]

        return inrange, self.walltree.data[idxs, :]

    def pub_wall(self, wallpts):
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
        msg.color.g = 0
        msg.color.b = 1

        pts = []
        for i in range(len(wallpts)):
            pt = Point()
            pt.x = wallpts[i,0]
            pt.y = wallpts[i,1]
            pts.append(pt)
        
        msg.points = pts
        self.pub_wallpts.publish(msg)

    def pub_laser_map(self, laser_map, scan):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = scan.header.stamp
        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        s = 0.05
        msg.scale.x = s
        msg.scale.y = s
        msg.scale.z = s

        msg.color.a = 1.0
        msg.color.r = 0
        msg.color.g = 0
        msg.color.b = 1

        pts = []
        for i in range(len(laser_map)):
            pt = Point()
            pt.x = laser_map[i,0]
            pt.y = laser_map[i,1]
            pts.append(pt)
        
        msg.points = pts
        self.pub_lasmap.publish(msg)

