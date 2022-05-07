import time
import math
import numpy as np
from scipy.spatial import cKDTree

from nav_msgs.msg import OccupancyGrid

from planar_transform import PlanarTransform


WALLTHRESHOLD = 65      # Believe probability TODO: read from map yaml
MAXDISTANCE = 0.2         # Distance to wall
FRACTION = 0.01            # Correction dampening


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
    g_qw = mapgrid.info.origin.orientation.z
    res = mapgrid.info.resolution

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

    def get_tf(self):
        return FRACTION * self.correction
    
    def update(self, laserpts):
        pts, wallnear = self.nearest_wallpts(laserpts)

        r = pts.flatten()
        p = wallnear.flatten()

        a = p-r

        rhs = (np.flip(pts, axis=1) * np.array([[-1, 1]])).flatten()[:,None]
        lhs = np.tile(np.eye(2), (len(pts), 1))
        J = np.hstack((lhs, rhs))

        d = (np.linalg.pinv(J) @ a).flatten()
        assert (d.shape == (3,)), "Wrong delta dimensions: {0} != (3,)".format(d.shape)

        self.correction = PlanarTransform.basic(*d)

        return pts

    def nearest_wallpts(self, pts):
        dists, idxs = self.walltree.query(pts)

        inrange = np.where(dists < MAXDISTANCE)
        idxs = idxs[inrange]

        return pts[inrange], self.walltree.data[idxs, :]

