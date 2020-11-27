#!/usr/bin/python
import unittest

from occupancy_grid import OccupancyGrid
from typing import Tuple, Optional, List


class TestToGrid(unittest.TestCase):

    def to_grid(self, occupancy_grid, test_cases):
        """
        Asserts that all test cases pass for the to_grid function
        @author Callum
        @param occupancy_grid: Occupancy grid to test on
        @type occupancy_grid: OccupancyGrid
        @param test_cases: List of Tuples, element 1 is the world coordinates as a Tuple,
        element 2 is the expected output
        @type test_cases: List[Tuple[Tuple[float, float], Optional[Tuple[float, float]]]]
        """
        for test_case in test_cases:
            self.assertEqual(occupancy_grid.to_grid(test_case[0][0], test_case[0][1]), test_case[1],
                             "to_grid{} should be {}!".format(test_case[0], test_case[1]))

    def test_bounds(self):
        """
        @author Callum
        """
        self.to_grid(OccupancyGrid((0, 0), (20, 20), 1), [
            ((5, 7), (5, 7)),
            ((3, 10), (3, 10)),
            ((-6, -1), None),
            ((25, 4), None)
        ])

    def test_resolution(self):
        """
        @author Callum
        """
        self.to_grid(OccupancyGrid((0, 0), (20, 20), 0.5), [
            ((5, 5), (10, 10)),
            ((0, 0), (0, 0)),
            ((-6, -1), None),
            ((25, 4), None)
        ])
        self.to_grid(OccupancyGrid((0, 0), (20, 20), 2), [
            ((5, 5), (2, 2)),
            ((0, 0), (0, 0)),
            ((-6, -1), None),
            ((25, 4), None)
        ])

    def test_origin(self):
        """
        @author Callum
        """
        self.to_grid(OccupancyGrid((-5, -5), (20, 20), 1), [
            ((5, 5), (10, 10)),
            ((0, 0), (5, 5)),
            ((-6, -1), None),
            ((25, 4), None)
        ])

    def test_pooling(self):
        """
        @author Callum
        """
        self.to_grid(OccupancyGrid((0, 0), (20, 20), 1), [
            ((5.25, 7.75), (5, 7)),
            ((3.35, 10.63), (3, 10)),
            ((-6, -1), None),
            ((20.1, 4), None),
            ((20, 4), (20, 4))
        ])
        
class TestToWorld(unittest.TestCase):
   
    def to_world(self, occupancy_grid, test_cases):
        for test_case in test_cases:
            self.assertEqual(occupancy_grid.to_world(test_case[0][0], test_case[0][1]), test_case[1],
                             "to_world{} should be {}!".format(test_case[0], test_case[1]))

    def test_bounds(self):
        """
        @author Mohamed
        """
        self.to_world(OccupancyGrid((0, 0), (20, 20), 1), [
            ((5, 7), (5, 7)),
            ((3, 10), (3, 10)),
            ((-6, -1), None),
            ((50, 4), None)
        ])

    def test_resolution(self):
        """
        @author Mohamed
        """
        self.to_world(OccupancyGrid((0, 0), (20, 20), 0.5), [
            ((10, 10), (5, 5)),
            ((0, 0), (0, 0)),
            ((-6, -1), None),
            ((50, 4), None)
        ])
        self.to_world(OccupancyGrid((0, 0), (20, 20), 2), [
            ((2, 2), (5, 5)),
            ((0, 0), (1, 1)),
            ((-6, -1), None),
            ((50, 4), None)
        ])

    def test_origin(self):
        """
        @author Mohamed
        """
        self.to_world(OccupancyGrid((-5, -5), (20, 20), 1), [
            ((9, 9), (4, 4)),
            ((5, 5), (0, 0)),
            ((-6, -1), None),
            ((50, 4), None)
        ])

if __name__ == '__main__':
    unittest.main()
