"""
Differential Kinematics for Thor Robot Arm
Handles conversion between joint angles (Art5, Art6) and differential motor positions (V, W)

The robot uses a bevel gear differential mechanism for the last two wrist joints:
- Art5 and Art6 are joint angles (logical)
- Motor V (Drive 5) and Motor W (Drive 6) are physical motors
- The differential couples these motors mechanically

Forward Differential (joint angles → motor positions):
    Motor_V = Art6 + Art5
    Motor_W = Art6 - Art5

Inverse Differential (motor positions → joint angles):
    Art5 = (Motor_V - Motor_W) / 2
    Art6 = (Motor_V + Motor_W) / 2
"""

from typing import Tuple
import logging

logger = logging.getLogger(__name__)


class DifferentialKinematics:
    """
    Helper class for differential kinematics calculations
    """

    @staticmethod
    def joint_to_motor(art5: float, art6: float) -> Tuple[float, float]:
        """
        Convert joint angles to motor positions (forward differential)

        Args:
            art5: Joint 5 angle in degrees
            art6: Joint 6 angle in degrees

        Returns:
            tuple: (motor_v, motor_w) in degrees
        """
        motor_v = art6 + art5
        motor_w = art6 - art5
        return (motor_v, motor_w)

    @staticmethod
    def motor_to_joint(motor_v: float, motor_w: float) -> Tuple[float, float]:
        """
        Convert motor positions to joint angles (inverse differential)

        Args:
            motor_v: Motor V (Drive 5) position in degrees
            motor_w: Motor W (Drive 6) position in degrees

        Returns:
            tuple: (art5, art6) in degrees
        """
        art5 = (motor_v - motor_w) / 2.0
        art6 = (motor_v + motor_w) / 2.0
        return (art5, art6)

    @staticmethod
    def move_art5_only(current_motor_v: float, current_motor_w: float, new_art5: float) -> Tuple[float, float, float]:
        """
        Calculate motor positions to move Art5 while keeping Art6 stationary

        Args:
            current_motor_v: Current Motor V position in degrees
            current_motor_w: Current Motor W position in degrees
            new_art5: Desired Art5 position in degrees

        Returns:
            tuple: (motor_v, motor_w, art6_kept_at) - new motor positions and the Art6 value kept constant
        """
        # Calculate current Art6 to keep it stationary
        _, art6 = DifferentialKinematics.motor_to_joint(current_motor_v, current_motor_w)

        # Calculate new motor positions
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(new_art5, art6)

        logger.debug(f"Move Art5 only: Art5={new_art5:.2f}deg, Art6={art6:.2f}deg (kept) -> V={motor_v:.2f}deg, W={motor_w:.2f}deg")

        return (motor_v, motor_w, art6)

    @staticmethod
    def move_art6_only(current_motor_v: float, current_motor_w: float, new_art6: float) -> Tuple[float, float, float]:
        """
        Calculate motor positions to move Art6 while keeping Art5 stationary

        Args:
            current_motor_v: Current Motor V position in degrees
            current_motor_w: Current Motor W position in degrees
            new_art6: Desired Art6 position in degrees

        Returns:
            tuple: (motor_v, motor_w, art5_kept_at) - new motor positions and the Art5 value kept constant
        """
        # Calculate current Art5 to keep it stationary
        art5, _ = DifferentialKinematics.motor_to_joint(current_motor_v, current_motor_w)

        # Calculate new motor positions
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, new_art6)

        logger.debug(f"Move Art6 only: Art5={art5:.2f}deg (kept), Art6={new_art6:.2f}deg -> V={motor_v:.2f}deg, W={motor_w:.2f}deg")

        return (motor_v, motor_w, art5)

    @staticmethod
    def validate_differential_consistency(motor_v: float, motor_w: float, art5: float, art6: float, tolerance: float = 0.1) -> Tuple[bool, str]:
        """
        Verify that motor positions and joint angles are consistent

        Args:
            motor_v, motor_w: Motor positions in degrees
            art5, art6: Joint angles in degrees
            tolerance: Maximum acceptable error in degrees

        Returns:
            tuple: (is_consistent, error_message)
        """
        # Calculate what the motors should be for these joint angles
        expected_v, expected_w = DifferentialKinematics.joint_to_motor(art5, art6)

        error_v = abs(motor_v - expected_v)
        error_w = abs(motor_w - expected_w)

        if error_v > tolerance or error_w > tolerance:
            error_msg = f"Differential inconsistency: V error={error_v:.3f}°, W error={error_w:.3f}° (tolerance={tolerance}°)"
            return (False, error_msg)

        return (True, "Differential kinematics consistent")


if __name__ == "__main__":
    # Test the differential kinematics
    import sys
    logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

    print("Differential Kinematics Test\n")
    print("=" * 60)

    # Test 1: Forward differential
    print("\nTest 1: Joint to Motor (Forward Differential)")
    art5, art6 = 30.0, 45.0
    motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
    print(f"  Input:  Art5={art5}°, Art6={art6}°")
    print(f"  Output: Motor_V={motor_v}°, Motor_W={motor_w}°")

    # Test 2: Inverse differential
    print("\nTest 2: Motor to Joint (Inverse Differential)")
    calc_art5, calc_art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)
    print(f"  Input:  Motor_V={motor_v}°, Motor_W={motor_w}°")
    print(f"  Output: Art5={calc_art5}°, Art6={calc_art6}°")
    print(f"  Match:  {abs(art5 - calc_art5) < 0.001 and abs(art6 - calc_art6) < 0.001}")

    # Test 3: Move Art5 only
    print("\nTest 3: Move Art5 Only")
    current_v, current_w = 75.0, 15.0
    new_art5 = 50.0
    new_v, new_w, kept_art6 = DifferentialKinematics.move_art5_only(current_v, current_w, new_art5)
    print(f"  Current: V={current_v}°, W={current_w}°")
    print(f"  New Art5: {new_art5}°")
    print(f"  Result: V={new_v}°, W={new_w}° (Art6 kept at {kept_art6}°)")

    # Test 4: Move Art6 only
    print("\nTest 4: Move Art6 Only")
    new_art6 = 60.0
    new_v, new_w, kept_art5 = DifferentialKinematics.move_art6_only(current_v, current_w, new_art6)
    print(f"  Current: V={current_v}°, W={current_w}°")
    print(f"  New Art6: {new_art6}°")
    print(f"  Result: V={new_v}°, W={new_w}° (Art5 kept at {kept_art5}°)")

    # Test 5: Validate consistency
    print("\nTest 5: Validate Consistency")
    is_consistent, msg = DifferentialKinematics.validate_differential_consistency(37.5, 7.5, 30.0, 45.0)
    print(f"  Consistent: {is_consistent}")
    print(f"  Message: {msg}")

    print("\n" + "=" * 60)
    print("All tests complete!")
