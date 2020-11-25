from typing import Tuple, Optional


class OccupancyGrid:
    resolution = 0
    size = (0, 0)
    origin = (0, 0)

    def __init__(self, origin, size, resolution):
        """
        Initializes:
            origin as self.origin,
            size as self.size,
            resolution as self.resolution,
            flat grid of -1 in the occupancy grid as self.grid
        @type origin: Tuple[float, float]
        @type size: Tuple[int, int]
        @type resolution: float
        """
        origin = self.origin
        size = self.size
        resolution = self.resolution



    def to_grid(self, world_x, world_y):
        """
        Converts world coordinates to grid coordinates, or None if world coordinates are out of bound
        @type world_x: float
        @type world_y: float
        @rtype: Optional[Tuple[int, int]]
        """
        if world_x > self.size[0] or world_y > self.size[1] or world_x < 0 or world_y < 0:
            return None


    def to_world(self, grid_x, grid_y):
        """
        Converts grid coordinates to world coordinates, or None if converted coordinates are out of bounds
        @author Callum
        @type grid_x: int
        @type grid_y: int
        @rtype: Optional[Tuple[float, float]]
        """
        half_cell_size = self.resolution / 2
        world_x = grid_x * self.resolution + self.origin[0] + half_cell_size
        world_y = grid_y * self.resolution + self.origin[1] + half_cell_size
        if world_x > self.size[0] or world_y > self.size[1] or world_x < 0 or world_y < 0:
            return None
        return world_x, world_y

    def mark_visited(self, world_x, world_y):
        """
        Marks the appropriate grid cell as 100 as dictated by the provided world coordinates
        @type world_x: float
        @type world_y: float
        """
