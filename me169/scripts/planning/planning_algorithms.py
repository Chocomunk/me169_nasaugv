import math
import numpy as np

from nav_msgs.msg       import OccupancyGrid

from planar_transform import PlanarTransform
from occupancy_map import OccupancyMap
from planning_util import TileStates, PriorityQueue


OCC_THRESH = 0.65 * 100
FREE_THRESH = 0.196 * 100
BOT_RAD = 0.15


class SearchError(Exception):
    pass


def astar_search(start, end, reachable, get_neighbors):
    """
    Perform a discrete planning search over the given problem definition and
    defined cost function. The cost function should be able to take:
        cost_to_reach, current_coordinate, end_coordinate

    Returns: path A list storing the coordinates of the resulting path (ordered)
    """
    state = {}
    cost_to_reach = {}
    parent = {}

    # --------------------- Priority Queue interaction -------------------------
    #   These inner functions will be used to more cleanly interact with the
    #   priority queue backend. Every push/pop operation requires the same
    #   supporting operations (e.g. updating cost and parents), so we wrap them
    #   in functions to make the main algorithm loop more readable

    q = PriorityQueue()         # Define the p-queue of ONDECK states

    def push(coord, c, par):
        """ Push an element `coord` with cost-to-reach `c` and parent `par` """
        dx = end[0] - coord[0]
        dy = end[1] - coord[1]
        child_cost = c + np.sqrt(dx*dx + dy*dy)

        # This tile was not encountered yet
        if coord not in state:
            q.push(coord, child_cost)
            state[coord] = TileStates.ONDECK
            if par:
                cost_to_reach[coord] = c
                parent[coord] = par

        # This tile was seen in another path
        elif state[coord] == TileStates.ONDECK:
            # Newer path is lower cost, so update to new
            if c < cost_to_reach[coord]:
                q.remove(coord)
                q.push(coord, child_cost)
                if par:
                    cost_to_reach[coord] = c
                    parent[coord] = par

        # else: do nothing

    def pop():
        """ Pop an element from the p-queue """
        coord = q.pop()
        state[coord] = TileStates.PROCESSED
        return coord

    # ------------------------- Main Algorithm Loop ---------------------------- 

    # Initialize search
    push(start, 0, None)
    cost_to_reach[start] = 0

    # Perform search
    found = False
    while not q.is_empty():
        curr = pop()                            # Get the next lowest-cost leaf
        print(curr)
        if reachable(curr, end):                # If it is the goal, we finished!
            found = True
            break
        for child, c in get_neighbors(curr):    # Otherwise, check every child
            push(child, cost_to_reach[curr] + c, curr)

    if not found:
        raise SearchError("Could not reach the goal state.")

    # return list(state.keys())

    # Walk the search path
    path = [curr]                      # Populated backwards from `end`
    curr = tuple(parent[curr])

    # Loop while the parent exists, the start node has no parent
    while curr in parent:
        path.append(curr)                       # Add to the path
        curr = parent[curr]                     # Step to the next parent
    return list(reversed(path))                 # Reverse the list into the right order.


class AStarPlan:

    def __init__(self):
        # Declare variables (not initialized)
        self.h, self.w = 0, 0
        self.occ_map = None

    def update_map(self, map_msg: OccupancyGrid):
        self.w = map_msg.info.width
        self.h = map_msg.info.height
        self.res = map_msg.info.resolution
        self.grid = np.array(map_msg.data).reshape((self.h, self.w))
        self.occ_map = OccupancyMap(map_msg)

    def near_nodes(self, r, c, s):
        """ Return list of ((row, col), dist) neighbors """
        d = np.sqrt(2) * s
        neighbors = [
            ((r+s, c+s), d), ((r-s, c-s), d), ((r-s, c+s), d), ((r+s, c-s), d), 
            ((r+s, c), s),   ((r-s, c), s),   ((r, c+s), s),   ((r, c-s), s)
        ]
        return [((r,c), d) for (r,c), d in neighbors 
                if (0 <= r < self.h) and (0 <= c < self.w)]

    def next_to_wall(self, r, c):
        for coord, _ in self.near_nodes(r, c, int(0.5 + BOT_RAD / self.res)):
            print(coord, self.grid[coord])
            s = self.grid[coord]
            if s >= OCC_THRESH:
                return True

    def neighbors(self, coord, s=1):
        """ Returns all in-bound neighbors of a coord """
        r, c = coord
        neighborhood = self.near_nodes(r, c, s)
        neighborhood_filtered = []
        for (r, c), dist in neighborhood:
            if not self.next_to_wall(r, c):
                neighborhood_filtered.append(((r,c), dist))
            else:
                print((r,c))
        return neighborhood_filtered

    def end_inrange(self, pos, end, s=1):
        r1, c1 = pos
        r2, c2 = end
        return abs(r2 - r1) <= s and abs(c2 - c1) <= s

    def search(self, start, end):
        if not self.occ_map:
            raise SearchError("Did not set map")

        # Reverse dimensions from (x,y) -> (r,c)
        grid_start = tuple(reversed(self.occ_map.to_grid(np.array(start))))
        grid_end = tuple(reversed(self.occ_map.to_grid(np.array(end))))

        # Find path with A*
        path = astar_search(grid_start, grid_end, self.end_inrange, self.neighbors)

        # for r in range(self.h):
        #     for c in range(self.w):
        #         s = self.grid[r,c]
        #         if s >= OCC_THRESH or s < 0:
        #             path.append((r,c))

        # Reverse dimensions from (r,c) -> (x,y)
        path = [(x, y) for y, x in path]
        return self.occ_map.to_map(np.array(path))


    # def __search__(self, start, end, cost_func=astar_cost):
    #     """
    #     Perform a discrete planning search over the given problem definition and
    #     defined cost function. The cost function should be able to take:
    #         cost_to_reach, current_coordinate, end_coordinate

    #     Returns: path A list storing the coordinates of the resulting path (ordered)
    #     """
    #     # NOTE: NEEDED
    #     start = tuple(reversed(self.to_grid(np.array(start))))
    #     end = tuple(reversed(self.to_grid(np.array(end))))

    #     # Don't modify the original state
    #     state = np.copy(self.state)
    #     M, N = self.h, self.w                       # Get dimensions (row, col)

    #     cost_to_reach = np.ones((M,N)) * np.inf     # Initialize costs to inf
    #     parent = np.zeros((M,N,2), dtype=int) - 1   # Initialize parent table to (-1,-1)

    #     # --------------------- Priority Queue interaction -------------------------
    #     #   These inner functions will be used to more cleanly interact with the
    #     #   priority queue backend. Every push/pop operation requires the same
    #     #   supporting operations (e.g. updating cost and parents), so we wrap them
    #     #   in functions to make the main algorithm loop more readable

    #     q = PriorityQueue()         # Define the p-queue of ONDECK states

    #     def push(coord, c, par):
    #         """ Push an element `coord` with cost-to-reach `c` and parent `par` """
    #         s = state[coord]
    #         child_cost = cost_func(cost_to_reach=c, 
    #                                 coord=coord, start=start, end=end)

    #         # This tile was not encountered yet
    #         if s == TileStates.UNKNOWN:
    #             q.push(coord, child_cost)
    #             state[coord] = TileStates.ONDECK
    #             if par:
    #                 cost_to_reach[coord] = c
    #                 parent[coord] = par

    #         # This tile was seen in another path
    #         elif s == TileStates.ONDECK:
    #             # Newer path is lower cost, so update to new
    #             if c < cost_to_reach[coord]:
    #                 q.remove(coord)
    #                 q.push(coord, child_cost)
    #                 if par:
    #                     cost_to_reach[coord] = c
    #                     parent[coord] = par

    #         # else: do nothing

    #     def pop():
    #         """ Pop an element from the p-queue """
    #         coord = q.pop()
    #         if state[coord] == TileStates.WALL:
    #             print("Big problem")
    #         state[coord] = TileStates.PROCESSED
    #         return coord

    #     # ----------------------- Utility Inner Functions -------------------------- 

    #     def near_nodes(r, c, s):
    #         d = np.sqrt(2) * s
    #         return [
    #                 ((r+s, c+s), d), ((r-s, c-s), d), ((r-s, c+s), d), ((r+s, c-s), d), 
    #                 ((r+s, c), s),   ((r-s, c), s),   ((r, c+s), s),   ((r, c-s), s)
    #                 ]

    #     def next_to_wall(r, c):
    #         for coord, _ in near_nodes(r, c, math.ceil(BOT_RAD / self.res)):
    #             if state[coord] == TileStates.WALL:
    #                 return True

    #     def neighbors(coord, s=5):
    #         """ Returns all in-bound neighbors of a coord """
    #         r, c = coord
    #         total = near_nodes(r, c, s)
    #         filt = []
    #         for (r, c), a in total:
    #             if (0 <= r < M) and (0 <= c < N) and not next_to_wall(r, c):
    #                 filt.append(((r,c), a))
    #         return filt

    #     def inrange(a, b, s=5):
    #         x1, y1 = a
    #         x2, y2 = b
    #         return abs(x2 - x1) <= s+1 and abs(y2 - y1) <= s+1

    #     # ------------------------- Main Algorithm Loop ---------------------------- 

    #     # Initialize search
    #     push(start, 0, None)
    #     cost_to_reach[start] = 0

    #     # Perform search
    #     found = False
    #     while not q.is_empty():
    #         curr = pop()            # Get the next lowest-cost leaf
    #         if inrange(curr, end):          # If it is the goal, we finished!
    #             found = True
    #             break
    #         for child, c in neighbors(curr):   # Otherwise, check every child
    #             push(child, cost_to_reach[curr] + c, curr)

    #     if not found:
    #         raise SearchError("Could not reach the goal state.")

    #     # Walk the search path
    #     path = [curr]                      # Populated backwards from `end`
    #     curr = tuple(parent[curr])

    #     # Loop while the parent exists, the start node has no parent
    #     while (parent[curr] != [-1, -1]).any():
    #         path.append(curr)                       # Add to the path
    #         curr = tuple(parent[curr])              # Step to the next parent
    #     path = list(reversed(path))                 # Reverse the list into the right order.
        
    #     # NOTE: NEEDED
    #     path = [(x, y) for y, x in path]
    #     return self.to_map(np.array(path))
