from typing import Tuple, Optional


class OccupancyGrid:
    resolution = 0
    size = (0, 0)
    origin = (0, 0)
    grid = None

    def __init__(self, origin, size, resolution):
        """
        Initializes:
            origin as self.origin,
            size as self.size,
            resolution as self.resolution,
            flat grid of -1 in the occupancy grid as self.grid
        @author Oliver
        @type origin: Tuple[float, float]
        @type size: Tuple[int, int]
        @type resolution: float
        """
        self.origin = origin
        self.size = size
        self.resolution = resolution
        self.grid = [-1] * int((size[0] * size[1] / pow(resolution, 2)))

    #      print("grid_vec_length = {}\n").format(grid_vec_length)

    def to_grid(self, world_x, world_y):
        """
        Converts world coordinates to grid coordinates, or None if world coordinates are out of bound
        @author Oliver
        @type world_x: float
        @type world_y: float
        @rtype: Optional[Tuple[int, int]]
        """
        print world_x, world_y, self.size, self.origin
        if world_x < self.origin[0] or world_y < self.origin[1] or world_x > self.origin[
            0] + self.size[0] or world_y > self.origin[1] + self.size[1]:
            return None
        inverse_resolution = 1 / float(self.resolution)
        grid_x = int((world_x - self.origin[0]) * inverse_resolution)
        grid_y = int((world_y - self.origin[1]) * inverse_resolution)

        return grid_x, grid_y

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
        return int(world_x), int(world_y)

    def mark_visited(self, world_x, world_y):
        """
        Marks the appropriate grid cell as 100 as dictated by the provided world coordinates
        @author Oliver
        @type world_x: float
        @type world_y: float
        """

        grid_x, grid_y = self.to_grid(world_x, world_y)

        if (grid_x is None) or (grid_y is None):
            return
        print grid_x, grid_y, len(self.grid), grid_y * self.size[0] + grid_x, grid_y * int(
            self.size[0] / self.resolution) + grid_x

        self.grid[grid_y * self.size[0] + grid_x] = 100
