import numpy as np

from nav_msgs.msg       import OccupancyGrid

from planar_transform import PlanarTransform

class OccupancyMap:

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

        # Setup Grid
        self.w = self.map.info.width
        self.h = self.map.info.height
        self.grid = np.array(map_msg.data).reshape((self.h, self.w))

        # Setup transforms
        g_x = map_msg.info.origin.position.x
        g_y = map_msg.info.origin.position.y
        g_qz = map_msg.info.origin.orientation.z
        g_qw = map_msg.info.origin.orientation.w

        self.res = map_msg.info.resolution
        self.map2grid = PlanarTransform(g_x, g_y, g_qz, g_qw)

    def to_map(self, pts):
        return self.map2grid.apply(self.res * pts)
        
    def to_grid(self, pts):
        return (self.map2grid.inv().apply(pts) / self.res).round().astype(int)