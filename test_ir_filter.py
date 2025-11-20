import unittest
from collections import deque
from communication import AStarCommunication
from config import IR_FILTER_SAMPLES, IR_FILTER_THRESHOLD_HIGH, IR_FILTER_THRESHOLD_LOW

class TestIRFilter(unittest.TestCase):
    def setUp(self):
        # Mock the communication class to test only the filter
        self.comm = AStarCommunication()
        # Reset filter state manually
        self.comm.ir_history['ir_center'] = deque(maxlen=IR_FILTER_SAMPLES)
        self.comm.ir_filtered_state['ir_center'] = False

    def update_filter(self, raw_value):
        self.comm._update_ir_filter('ir_center', raw_value)
        return self.comm.ir_filtered_state['ir_center']

    def test_filter_logic(self):
        print("\nTesting Hysteresis Filter Logic:")
        
        # 1. Initial State: OFF
        self.assertFalse(self.comm.ir_filtered_state['ir_center'])
        print("  [OK] Initial state is OFF")

        # 2. Weak Signal (3 detections) -> Should stay OFF
        for _ in range(3):
            state = self.update_filter(0) # 0 = Detected
        self.assertFalse(state)
        print("  [OK] Weak signal (3/10) -> Still OFF")

        # 3. Strong Signal (7 detections total) -> Should turn ON
        # We already have 3, add 4 more = 7
        for _ in range(4):
            state = self.update_filter(0)
        self.assertTrue(state)
        print("  [OK] Strong signal (7/10) -> Turns ON")

        # 4. Noise/Flicker (Missed detections) -> Should stay ON
        # Add some "Not Detected" (1) samples
        # Current history is mostly 0s. Let's add 3 '1's.
        # Detections will drop from 7 to ~6 or 5 depending on what fell off
        for _ in range(3):
            state = self.update_filter(1) # 1 = Not Detected
        self.assertTrue(state)
        print("  [OK] Flicker (3 missed frames) -> Stays ON (Sticky)")

        # 5. Signal Loss (Continuous '1's) -> Should turn OFF
        # Push enough '1's to flush out the '0's
        for _ in range(10):
            state = self.update_filter(1)
        self.assertFalse(state)
        print("  [OK] Signal lost -> Turns OFF")

if __name__ == '__main__':
    unittest.main()
