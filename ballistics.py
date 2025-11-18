"""
Ballistics calculations for basketball robot
Includes RPM lookup table with interpolation
"""

import math
from config import (
    RPM_LOOKUP_TABLE,
    BASKET_ANGLES,
    SHOOTER_HEIGHT_INCHES,
    HOOP_HEIGHT_INCHES,
    LAUNCH_ANGLE_DEGREES,
    FLYWHEEL_RADIUS_MM
)


class BallisticsCalculator:
    """Handles shooting parameter calculations"""

    def __init__(self):
        # Sort lookup table by distance for interpolation
        self.rpm_table = sorted(RPM_LOOKUP_TABLE.items())

    def get_required_rpm(self, distance_inches):
        """
        Get required flywheel RPM for given distance using lookup table
        with linear interpolation

        Args:
            distance_inches: Horizontal distance to target (from ultrasonic)

        Returns:
            Required RPM for both flywheels
        """
        # Clamp distance to table range
        min_dist = self.rpm_table[0][0]
        max_dist = self.rpm_table[-1][0]

        if distance_inches <= min_dist:
            return self.rpm_table[0][1]
        if distance_inches >= max_dist:
            return self.rpm_table[-1][1]

        # Find bounding distances for interpolation
        for i in range(len(self.rpm_table) - 1):
            d1, rpm1 = self.rpm_table[i]
            d2, rpm2 = self.rpm_table[i + 1]

            if d1 <= distance_inches <= d2:
                # Linear interpolation
                ratio = (distance_inches - d1) / (d2 - d1)
                rpm = rpm1 + ratio * (rpm2 - rpm1)
                return int(rpm)

        # Fallback (shouldn't reach here)
        return self.rpm_table[len(self.rpm_table) // 2][1]

    def get_lazy_susan_angle(self, beacon_id):
        """
        Get required lazy susan angle for target beacon

        Args:
            beacon_id: 'LEFT', 'CENTER', or 'RIGHT'

        Returns:
            Angle in degrees
        """
        return BASKET_ANGLES.get(beacon_id, 0.0)

    def calculate_rpm_from_physics(self, distance_inches):
        """
        Calculate required RPM using projectile motion physics
        (Optional - use for generating initial lookup table)

        With shooter height = hoop height, trajectory is symmetric
        """
        # Convert to meters
        distance_m = distance_inches * 0.0254
        theta = math.radians(LAUNCH_ANGLE_DEGREES)
        g = 9.81  # m/s²

        # For flat trajectory (same start and end height):
        # Range = v₀² * sin(2θ) / g
        # Solve for v₀
        v0_squared = (distance_m * g) / math.sin(2 * theta)

        if v0_squared < 0:
            return None  # No solution

        v0 = math.sqrt(v0_squared)

        # Convert velocity to RPM
        # v = ω * r, where ω = RPM * 2π/60
        r = FLYWHEEL_RADIUS_MM / 1000.0  # Convert to meters
        omega = v0 / r  # rad/s
        rpm = omega * 60 / (2 * math.pi)

        return int(rpm)

    def generate_lookup_table(self, distances):
        """
        Generate RPM lookup table from physics model
        Useful for initial calibration

        Args:
            distances: List of distances to calculate

        Returns:
            Dictionary of {distance: rpm}
        """
        table = {}
        for dist in distances:
            rpm = self.calculate_rpm_from_physics(dist)
            if rpm:
                table[dist] = rpm
        return table

    def estimate_flight_time(self, distance_inches, rpm):
        """
        Estimate ball flight time (for predictive shooting)

        Args:
            distance_inches: Target distance
            rpm: Flywheel RPM

        Returns:
            Estimated flight time in seconds
        """
        # Calculate launch velocity
        r = FLYWHEEL_RADIUS_MM / 1000.0
        omega = rpm * 2 * math.pi / 60
        v0 = omega * r

        theta = math.radians(LAUNCH_ANGLE_DEGREES)

        # Flight time for flat trajectory: t = range / (v₀ cos θ)
        distance_m = distance_inches * 0.0254
        flight_time = distance_m / (v0 * math.cos(theta))

        return flight_time


# Global instance
ballistics = BallisticsCalculator()


if __name__ == "__main__":
    # Test the ballistics calculator
    print("=== Ballistics Calculator Test ===\n")

    # Test lookup table interpolation
    test_distances = [24, 28, 36, 42, 50, 60]

    print("RPM Lookup (with interpolation):")
    for dist in test_distances:
        rpm = ballistics.get_required_rpm(dist)
        flight_time = ballistics.estimate_flight_time(dist, rpm)
        print(f"  {dist}\" → {rpm} RPM (flight time: {flight_time:.3f}s)")

    print("\nPhysics-based RPM calculation:")
    for dist in test_distances:
        rpm = ballistics.calculate_rpm_from_physics(dist)
        if rpm:
            print(f"  {dist}\" → {rpm} RPM (theoretical)")

    print("\nLazy Susan Angles:")
    for basket in ['LEFT', 'CENTER', 'RIGHT']:
        angle = ballistics.get_lazy_susan_angle(basket)
        print(f"  {basket}: {angle}°")
