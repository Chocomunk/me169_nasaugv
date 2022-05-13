import math
import rospy
import numpy as np
from scipy.spatial import cKDTree

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

from planar_transform import PlanarTransform


WALLTHRESHOLD = 65      # Believe probability TODO: read from map yaml
MAXDISTANCE = 0.5         # Distance to wall
FRACTION = 0.03            # Correction dampening
POS_LIM = 0.3           # Limit for position correction (meters)
THETA_LIM = np.pi / 12  # Limit for angle correction (radians)


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

    print(g_qz, g_qw)

    g_o = np.array([g_x, g_y])
    g_t = 2 * math.atan2(g_qz, g_qw)
    c, s = np.cos(g_t), np.sin(g_t)
    rotmat = np.array([[c, -s], [s, c]])

    map_wallpts = g_o + res * wallpts @ rotmat.T

    return map_wallpts


#
#   Localization Correction
#
class IdentityCorrection:
    # Initialize.
    def __init__(self, map: OccupancyGrid):
        self.map = map      # Not needed, but store for reference
        self.identity_tf = PlanarTransform.basic(0, 0, 0)

    def get_tf(self):
        return self.identity_tf
    
    def update(self, laserpts):
        # Do nothing for now, keep returning the identity transform
        pass


class BasicLeastSquaresCorrection:

    def __init__(self, mapgrid: OccupancyGrid):
        self.map = mapgrid
        self.walltree = cKDTree(wall_points(mapgrid))
        self.correction = PlanarTransform.basic(0, 0, 0)
        self.x_pub = rospy.Publisher("/corrx", Float32, queue_size=10)
        self.y_pub = rospy.Publisher("/corry", Float32, queue_size=10)
        self.t_pub = rospy.Publisher("/corrt", Float32, queue_size=10)
        self.m_pub = rospy.Publisher("/corrm", Float32, queue_size=10)

    def get_tf(self):
        return FRACTION * self.correction
    
    def update(self, laserpts, weights=None):
        idxs, wallnear = self.nearest_wallpts(laserpts)
        pts = laserpts[idxs]

        if weights is None:
            lam = np.eye(len(pts) * 2)
        else:
            wts = weights[idxs]
            dbl_wts = np.tile(wts, (2,1)).swapaxes(0,1).flatten()
            lam = np.diag(dbl_wts)

        r = pts.flatten()
        p = wallnear.flatten()
        a = p-r

        rhs = (np.flip(pts, axis=1) * np.array([[-1, 1]])).flatten()[:,None]
        lhs = np.tile(np.eye(2), (len(pts), 1))
        J = np.hstack((lhs, rhs))

        # if weights is None:
        #     lam = np.eye(len(pts))
        # else:
        #     lam = np.diag(weights[idxs])

        # r = pts
        # p = wallnear
        # a = np.linalg.norm(p-r, axis=1)[:,None]

        # lhs = p-r
        # rhs = np.sum(r * np.flip(p, axis=1) * np.array([[1, -1]]), axis=1)[:,None]
        # J = np.divide(np.hstack((lhs, rhs)), a)

        b = J.T @ lam
        J_dag = np.linalg.pinv(b @ J) @ b
        d = (J_dag @ a).flatten()
        assert (d.shape == (3,)), "Wrong delta dimensions: {0} != (3,)".format(d.shape)

        m = np.linalg.norm(d)
        if m > POS_LIM:
            d *= POS_LIM / m
        x, y, t = d

        # t_abs = np.abs(t)
        # if t_abs > THETA_LIM:
        #     d *= THETA_LIM / t_abs

        self.x_pub.publish(x)
        self.y_pub.publish(y)
        self.t_pub.publish(t)
        self.m_pub.publish(np.linalg.norm(d))

        self.correction = PlanarTransform.basic(x, y, t)

        return pts

    def nearest_wallpts(self, pts):
        dists, idxs = self.walltree.query(pts)

        inrange = np.where(dists < MAXDISTANCE)
        idxs = idxs[inrange]

        return inrange, self.walltree.data[idxs, :]

