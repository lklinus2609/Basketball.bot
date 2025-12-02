import unittest
from unittest.mock import MagicMock
import sys
import os

# Add parent directory to path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from state_machine import BasketballRobotStateMachine
import config

class TestLocalization(unittest.TestCase):
    def setUp(self):
        # Mock communication object
        self.mock_comm = MagicMock()
        self.sm = BasketballRobotStateMachine(self.mock_comm)
        
        # Use config dimensions
        self.L = config.ARENA_LENGTH
        self.W = config.ARENA_WIDTH
        self.CENTER_X = self.L / 2
        self.CENTER_Y = self.W / 2

    def test_extract_four_walls_ideal(self):
        """Test extracting walls from ideal scan data"""
        # Simulate readings for a robot at center
        # North (90°) = CENTER_Y, South (270°) = CENTER_Y, East (0°) = CENTER_X, West (180°) = CENTER_X
        
        scan_readings = [
            (0, self.CENTER_X),    # East
            (90, self.CENTER_Y),   # North
            (180, self.CENTER_X),  # West
            (270, self.CENTER_Y)   # South
        ]
        
        walls = self.sm._extract_four_walls(scan_readings)
        
        self.assertEqual(walls['EAST'][1], self.CENTER_X)
        self.assertEqual(walls['NORTH'][1], self.CENTER_Y)
        self.assertEqual(walls['WEST'][1], self.CENTER_X)
        self.assertEqual(walls['SOUTH'][1], self.CENTER_Y)

    def test_extract_four_walls_corner_vs_perpendicular(self):
        """Test if logic correctly identifies wall distance vs corner distance"""
        # Robot at center (36, 36)
        # East Wall (at x=72) is 36 units away at 0 degrees.
        # North-East Corner (at x=72, y=0) is at 45 degrees (approx).
        # Distance to corner: sqrt(36^2 + 36^2) = 50.91
        
        # Scan readings in East Quadrant (0-90)
        scan_readings = [
            (0, 36.0),      # Perpendicular (Correct wall distance)
            (45, 50.91),    # Corner (Furthest point)
            (90, 36.0),     # North (Perpendicular)
            (180, 36.0),    # West
            (270, 36.0)     # South
        ]
        
        walls = self.sm._extract_four_walls(scan_readings)
        
        # The logic should use min() to find the perpendicular distance (closest point)
        # So we expect 36.0, NOT 50.91
        self.assertEqual(walls['EAST'][1], 36.0) 


    def test_calculate_position_center(self):
        """Test position calculation for robot at center"""
        walls = {
            'EAST': (0, self.CENTER_X),
            'NORTH': (90, self.CENTER_Y),
            'WEST': (180, self.CENTER_X),
            'SOUTH': (270, self.CENTER_Y)
        }
        
        pos = self.sm._calculate_position_from_walls(walls)
        
        # x = ARENA_LENGTH - dist_east = L - L/2 = L/2
        # y = dist_north = W/2
        
        self.assertEqual(pos, (self.CENTER_X, self.CENTER_Y))

    def test_calculate_position_offset(self):
        """Test position calculation for robot at offset position"""
        # Robot at (10, 20)
        # x = 10 (close to back wall/West)
        # y = 20 (close to left wall/North)
        
        target_x = 10.0
        target_y = 20.0
        
        # Distances:
        # East (x=L): L - 10
        # West (x=0): 10
        # North (y=0): 20
        # South (y=W): W - 20
        
        dist_east = self.L - target_x
        dist_west = target_x
        dist_north = target_y
        dist_south = self.W - target_y
        
        walls = {
            'EAST': (0, dist_east),
            'NORTH': (90, dist_north),
            'WEST': (180, dist_west),
            'SOUTH': (270, dist_south)
        }
        
        pos = self.sm._calculate_position_from_walls(walls)
        
        self.assertEqual(pos, (target_x, target_y))

    def test_calculate_position_missing_wall(self):
        """Test position calculation with missing wall data"""
        # Missing West and South
        walls = {
            'EAST': (0, self.CENTER_X),
            'NORTH': (90, self.CENTER_Y)
        }
        
        pos = self.sm._calculate_position_from_walls(walls)
        
        # Should still calculate based on available data
        self.assertEqual(pos, (self.CENTER_X, self.CENTER_Y))

if __name__ == '__main__':
    unittest.main()
