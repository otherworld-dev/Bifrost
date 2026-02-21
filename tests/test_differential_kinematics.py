"""
Tests for differential_kinematics.py

The Thor robot uses a bevel gear differential mechanism for wrist joints:
- Art5 and Art6 are logical joint angles
- Motor V and Motor W are physical motors
- J6 (roll) has a 2:1 mechanical reduction through the differential
- Forward: Motor_V = 2*Art6 + Art5, Motor_W = 2*Art6 - Art5
- Inverse: Art5 = (V - W) / 2, Art6 = (V + W) / 4
"""

import pytest
from differential_kinematics import DifferentialKinematics


class TestJointToMotor:
    """Tests for forward differential conversion (joint angles -> motor positions)"""

    def test_zero_angles(self):
        """Both joints at zero should give zero motor positions"""
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(0.0, 0.0)
        assert motor_v == 0.0
        assert motor_w == 0.0

    def test_positive_art5_only(self):
        """Art5 positive, Art6 zero: V=Art5, W=-Art5 (motors opposite)"""
        art5, art6 = 30.0, 0.0
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        # V = 2*0 + 30 = 30
        # W = 2*0 - 30 = -30
        assert motor_v == 30.0
        assert motor_w == -30.0

    def test_positive_art6_only(self):
        """Art6 positive, Art5 zero: V=W=2*Art6 (motors same direction, doubled)"""
        art5, art6 = 0.0, 45.0
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        # V = 2*45 + 0 = 90
        # W = 2*45 - 0 = 90
        assert motor_v == 90.0
        assert motor_w == 90.0

    def test_both_positive(self):
        """Both joints positive"""
        art5, art6 = 30.0, 45.0
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        # V = 2*45 + 30 = 120
        # W = 2*45 - 30 = 60
        assert motor_v == 120.0
        assert motor_w == 60.0

    def test_negative_angles(self):
        """Negative joint angles"""
        art5, art6 = -20.0, -30.0
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        # V = 2*(-30) + (-20) = -80
        # W = 2*(-30) - (-20) = -40
        assert motor_v == -80.0
        assert motor_w == -40.0

    def test_mixed_signs(self):
        """Art5 negative, Art6 positive"""
        art5, art6 = -15.0, 60.0
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        # V = 2*60 + (-15) = 105
        # W = 2*60 - (-15) = 135
        assert motor_v == 105.0
        assert motor_w == 135.0

    def test_art6_90_requires_180_motor(self):
        """J6=90 should require 180 motor degrees (2:1 ratio)"""
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(0.0, 90.0)
        assert motor_v == 180.0
        assert motor_w == 180.0


class TestMotorToJoint:
    """Tests for inverse differential conversion (motor positions -> joint angles)"""

    def test_zero_motors(self):
        """Both motors at zero should give zero joint angles"""
        art5, art6 = DifferentialKinematics.motor_to_joint(0.0, 0.0)
        assert art5 == 0.0
        assert art6 == 0.0

    def test_equal_motors(self):
        """Equal motor positions means Art5=0 (motors move together for Art6 only)"""
        motor_v, motor_w = 180.0, 180.0
        art5, art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)
        # Art5 = (180 - 180) / 2 = 0
        # Art6 = (180 + 180) / 4 = 90
        assert art5 == 0.0
        assert art6 == 90.0

    def test_opposite_motors(self):
        """Opposite motor positions means Art6=0 (motors move opposite for Art5 only)"""
        motor_v, motor_w = 30.0, -30.0
        art5, art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)
        # Art5 = (30 - (-30)) / 2 = 30
        # Art6 = (30 + (-30)) / 4 = 0
        assert art5 == 30.0
        assert art6 == 0.0

    def test_general_case(self):
        """General case with both joints non-zero"""
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)
        # Art5 = (120 - 60) / 2 = 30
        # Art6 = (120 + 60) / 4 = 45
        assert art5 == 30.0
        assert art6 == 45.0


class TestRoundTrip:
    """Tests that verify forward->inverse and inverse->forward conversions are consistent"""

    @pytest.mark.parametrize("art5,art6", [
        (0.0, 0.0),
        (30.0, 45.0),
        (-20.0, 60.0),
        (90.0, -90.0),
        (-45.0, -45.0),
        (0.0, 180.0),
        (90.0, 0.0),
        (-90.0, 180.0),
    ])
    def test_joint_to_motor_to_joint(self, art5, art6):
        """Converting joint->motor->joint should return original values"""
        motor_v, motor_w = DifferentialKinematics.joint_to_motor(art5, art6)
        recovered_art5, recovered_art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)

        assert abs(recovered_art5 - art5) < 1e-10
        assert abs(recovered_art6 - art6) < 1e-10

    @pytest.mark.parametrize("motor_v,motor_w", [
        (0.0, 0.0),
        (120.0, 60.0),
        (-80.0, -40.0),
        (100.0, -100.0),
        (-200.0, 200.0),
        (180.0, 180.0),
    ])
    def test_motor_to_joint_to_motor(self, motor_v, motor_w):
        """Converting motor->joint->motor should return original values"""
        art5, art6 = DifferentialKinematics.motor_to_joint(motor_v, motor_w)
        recovered_v, recovered_w = DifferentialKinematics.joint_to_motor(art5, art6)

        assert abs(recovered_v - motor_v) < 1e-10
        assert abs(recovered_w - motor_w) < 1e-10


class TestMoveArt5Only:
    """Tests for moving Art5 while keeping Art6 stationary"""

    def test_move_art5_keeps_art6(self):
        """Moving Art5 should not change Art6"""
        # V=120, W=60 -> Art5=30, Art6=45
        current_v, current_w = 120.0, 60.0
        new_art5 = 50.0

        new_v, new_w, kept_art6 = DifferentialKinematics.move_art5_only(
            current_v, current_w, new_art5
        )

        # Verify Art6 is kept at 45
        assert kept_art6 == 45.0

        # Verify new motor positions produce correct joint angles
        calc_art5, calc_art6 = DifferentialKinematics.motor_to_joint(new_v, new_w)
        assert abs(calc_art5 - 50.0) < 1e-10
        assert abs(calc_art6 - 45.0) < 1e-10

    def test_move_art5_to_zero(self):
        """Moving Art5 to zero should make V and W equal (pure Art6)"""
        # V=120, W=60 -> Art5=30, Art6=45
        current_v, current_w = 120.0, 60.0

        new_v, new_w, kept_art6 = DifferentialKinematics.move_art5_only(
            current_v, current_w, 0.0
        )

        # When Art5=0: V = W = 2*Art6 = 2*45 = 90
        assert new_v == new_w
        assert new_v == 90.0


class TestMoveArt6Only:
    """Tests for moving Art6 while keeping Art5 stationary"""

    def test_move_art6_keeps_art5(self):
        """Moving Art6 should not change Art5"""
        # V=120, W=60 -> Art5=30, Art6=45
        current_v, current_w = 120.0, 60.0
        new_art6 = 60.0

        new_v, new_w, kept_art5 = DifferentialKinematics.move_art6_only(
            current_v, current_w, new_art6
        )

        # Verify Art5 is kept at 30
        assert kept_art5 == 30.0

        # Verify new motor positions produce correct joint angles
        calc_art5, calc_art6 = DifferentialKinematics.motor_to_joint(new_v, new_w)
        assert abs(calc_art5 - 30.0) < 1e-10
        assert abs(calc_art6 - 60.0) < 1e-10

    def test_move_art6_to_zero(self):
        """Moving Art6 to zero should make V and W opposite"""
        # V=120, W=60 -> Art5=30, Art6=45
        current_v, current_w = 120.0, 60.0

        new_v, new_w, kept_art5 = DifferentialKinematics.move_art6_only(
            current_v, current_w, 0.0
        )

        # When Art6=0: V = Art5 = 30, W = -Art5 = -30
        assert abs(new_v - (-new_w)) < 1e-10
        assert abs(new_v - 30.0) < 1e-10

    def test_art6_doubling_effect(self):
        """Art6 change should move motors 2x the joint change"""
        # Start at Art5=0, Art6=0 -> V=0, W=0
        new_v, new_w, _ = DifferentialKinematics.move_art6_only(0.0, 0.0, 45.0)
        # V = 2*45 + 0 = 90, W = 2*45 - 0 = 90
        assert new_v == 90.0
        assert new_w == 90.0


class TestValidateDifferentialConsistency:
    """Tests for consistency validation between motor and joint values"""

    def test_consistent_values(self):
        """Consistent motor and joint values should pass validation"""
        # V=120, W=60 -> Art5=30, Art6=45
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = 30.0, 45.0

        is_consistent, msg = DifferentialKinematics.validate_differential_consistency(
            motor_v, motor_w, art5, art6
        )

        assert is_consistent is True
        assert "consistent" in msg.lower()

    def test_inconsistent_art5(self):
        """Wrong Art5 value should fail validation"""
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = 35.0, 45.0  # Art5 is wrong (should be 30)

        is_consistent, msg = DifferentialKinematics.validate_differential_consistency(
            motor_v, motor_w, art5, art6
        )

        assert is_consistent is False
        assert "inconsistency" in msg.lower()

    def test_inconsistent_art6(self):
        """Wrong Art6 value should fail validation"""
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = 30.0, 50.0  # Art6 is wrong (should be 45)

        is_consistent, msg = DifferentialKinematics.validate_differential_consistency(
            motor_v, motor_w, art5, art6
        )

        assert is_consistent is False
        assert "inconsistency" in msg.lower()

    def test_tolerance_within_bounds(self):
        """Small errors within tolerance should pass"""
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = 30.05, 44.95  # Small errors

        is_consistent, _ = DifferentialKinematics.validate_differential_consistency(
            motor_v, motor_w, art5, art6, tolerance=0.1
        )

        assert is_consistent is True

    def test_tolerance_exceeded(self):
        """Errors exceeding tolerance should fail"""
        motor_v, motor_w = 120.0, 60.0
        art5, art6 = 30.2, 44.8  # Errors exceed default tolerance

        is_consistent, _ = DifferentialKinematics.validate_differential_consistency(
            motor_v, motor_w, art5, art6, tolerance=0.1
        )

        assert is_consistent is False
