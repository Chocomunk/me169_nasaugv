from nav_msgs.msg       import MapMetaData
from planar_transform import PlanarTransform

class MapTransform:

    def __init__(self, map_info: MapMetaData):
        # Setup Grid
        self.w = map_info.width
        self.h = map_info.height

        # Setup transforms
        g_x = map_info.origin.position.x
        g_y = map_info.origin.position.y
        g_qz = map_info.origin.orientation.z
        g_qw = map_info.origin.orientation.w

        self.res = map_info.resolution
        self.map2grid = PlanarTransform(g_x, g_y, g_qz, g_qw)

    def to_map(self, pts):
        return self.map2grid.apply(self.res * pts)
        
    def to_grid(self, pts):
        return (self.map2grid.inv().apply(pts) / self.res).round().astype(int)