"""
Tests for robot_controller.py

Tests for robot state management and movement coordination.
"""

import pytest
from robot_controller import RobotController


class TestInitialization:
    """Tests for RobotController initialisation"""

    def test_default_initialization(self):
        """Controller should initialise with default values"""
        controller = RobotController()

        # All positions should start at 0
        positions = controller.get_current_positions()
        for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
            assert positions[joint] == 0.0

    def test_motor_positions_initialized(self):
        """Differential motors should be initialized to 0"""
        controller = RobotController()
        motor_v, motor_w = controller.get_differential_motor_positions()
        assert motor_v == 0.0
        assert motor_w == 0.0

    def test_update_count_starts_at_zero(self):
        """Position update count should start at 0"""
        controller = RobotController()
        assert controller.get_position_update_count() == 0

    def test_joint_config_exists(self):
        """Joint configuration should be set"""
        controller = RobotController()
        assert 'Art1' in controller.joint_config
        assert 'Art6' in controller.joint_config
        assert 'Gripper' in controller.joint_config


class TestInitializeFromSpinboxes:
    """Tests for initializing from GUI spinbox values"""

    def test_initialize_sets_positions(self, sample_joint_angles):
        """Initialise should set all joint positions"""
        controller = RobotController()
        controller.initialize_from_spinboxes(sample_joint_angles)

        positions = controller.get_current_positions()
        assert positions['Art1'] == 0.0
        assert positions['Art2'] == 90.0
        assert positions['Art3'] == 45.0
        assert positions['Art5'] == 30.0
        assert positions['Art6'] == 45.0

    def test_initialize_sets_differential_motors(self, sample_joint_angles):
        """Initialise should calculate differential motor positions"""
        controller = RobotController()
        controller.initialize_from_spinboxes(sample_joint_angles)

        motor_v, motor_w = controller.get_differential_motor_positions()
        # V = 2*Art6 + Art5 = 2*45 + 30 = 120
        # W = 2*Art6 - Art5 = 2*45 - 30 = 60
        assert motor_v == 120.0
        assert motor_w == 60.0

    def test_partial_initialization(self):
        """Initialise with partial data should only update provided joints"""
        controller = RobotController()
        partial = {'Art1': 10.0, 'Art2': 20.0}
        controller.initialize_from_spinboxes(partial)

        positions = controller.get_current_positions()
        assert positions['Art1'] == 10.0
        assert positions['Art2'] == 20.0


class TestValidatePosition:
    """Tests for position validation"""

    def test_valid_position(self):
        """Valid position within limits should pass"""
        controller = RobotController()
        valid, value = controller.validate_position('X', 100.0)
        assert valid is True
        assert value == 100.0

    def test_position_out_of_bounds(self):
        """Position outside limits should be clamped"""
        controller = RobotController()
        # X typically has limits like (-180, 180)
        valid, value = controller.validate_position('X', 200.0)
        assert valid is False
        # Value should be clamped to max limit

    def test_unknown_axis_accepted(self):
        """Unknown axis should be accepted as-is"""
        controller = RobotController()
        valid, value = controller.validate_position('UNKNOWN', 999.0)
        assert valid is True
        assert value == 999.0


class TestUpdatePositionsFromFirmware:
    """Tests for updating positions from firmware feedback"""

    def test_update_sets_positions(self, sample_firmware_positions):
        """Update should set all axis positions"""
        controller = RobotController()
        result = controller.update_positions_from_firmware(sample_firmware_positions)

        assert result['X'] == 10.0
        assert result['Y'] == 20.0
        assert result['Z'] == 30.0
        assert result['U'] == 5.0

    def test_update_calculates_art5_art6(self, sample_firmware_positions):
        """Update should calculate Art5 and Art6 from V and W"""
        controller = RobotController()
        result = controller.update_positions_from_firmware(sample_firmware_positions)

        # V=75, W=15 -> Art5=(75-15)/2=30, Art6=(75+15)/4=22.5
        assert result['Art5'] == 30.0
        assert result['Art6'] == 22.5

    def test_update_increments_counter(self, sample_firmware_positions):
        """Each update should increment position counter"""
        controller = RobotController()

        controller.update_positions_from_firmware(sample_firmware_positions)
        assert controller.get_position_update_count() == 1

        controller.update_positions_from_firmware(sample_firmware_positions)
        assert controller.get_position_update_count() == 2

    def test_update_tracks_motor_positions(self, sample_firmware_positions):
        """Update should track motor V and W"""
        controller = RobotController()
        controller.update_positions_from_firmware(sample_firmware_positions)

        motor_v, motor_w = controller.get_differential_motor_positions()
        assert motor_v == 75.0
        assert motor_w == 15.0


class TestGetCurrentPositions:
    """Tests for retrieving current positions"""

    def test_returns_copy(self):
        """Should return a copy, not reference to internal state"""
        controller = RobotController()
        pos1 = controller.get_current_positions()
        pos1['Art1'] = 999.0

        pos2 = controller.get_current_positions()
        assert pos2['Art1'] != 999.0  # Should not be modified


class TestCalculateDifferentialMove:
    """Tests for differential movement calculation"""

    def test_move_art5(self, sample_firmware_positions):
        """Calculate motors to move Art5"""
        controller = RobotController()
        controller.update_positions_from_firmware(sample_firmware_positions)

        motor_v, motor_w, kept = controller.calculate_differential_move('Art5', 50.0)

        # Art6 should be kept at current value (22.5, from V=75,W=15)
        assert kept == 22.5

        # New motor positions should produce Art5=50, Art6=22.5
        # V = 2*22.5 + 50 = 95
        # W = 2*22.5 - 50 = -5
        assert motor_v == 95.0
        assert motor_w == -5.0

    def test_move_art6(self, sample_firmware_positions):
        """Calculate motors to move Art6"""
        controller = RobotController()
        controller.update_positions_from_firmware(sample_firmware_positions)

        motor_v, motor_w, kept = controller.calculate_differential_move('Art6', 60.0)

        # Art5 should be kept at current value (30)
        assert kept == 30.0

        # New motor positions should produce Art5=30, Art6=60
        # V = 2*60 + 30 = 150
        # W = 2*60 - 30 = 90
        assert motor_v == 150.0
        assert motor_w == 90.0

    def test_invalid_joint_raises_error(self, sample_firmware_positions):
        """Invalid joint name should raise ValueError"""
        controller = RobotController()
        controller.update_positions_from_firmware(sample_firmware_positions)

        with pytest.raises(ValueError):
            controller.calculate_differential_move('Art3', 50.0)


class TestUpdateDifferentialMotors:
    """Tests for updating differential motor state"""

    def test_update_sets_motors(self):
        """Update should set motor positions"""
        controller = RobotController()
        controller.update_differential_motors(100.0, 20.0)

        motor_v, motor_w = controller.get_differential_motor_positions()
        assert motor_v == 100.0
        assert motor_w == 20.0

    def test_update_recalculates_joints(self):
        """Update should recalculate Art5 and Art6"""
        controller = RobotController()
        controller.update_differential_motors(100.0, 20.0)

        positions = controller.get_current_positions()
        # Art5 = (100 - 20) / 2 = 40
        # Art6 = (100 + 20) / 4 = 30
        assert positions['Art5'] == 40.0
        assert positions['Art6'] == 30.0


class TestCheckDifferentialInitialized:
    """Tests for differential initialization check"""

    def test_not_initialized_at_start(self):
        """Should not be initialized when both motors are 0"""
        controller = RobotController()
        assert controller.check_differential_initialized() is False

    def test_initialized_after_update(self, sample_firmware_positions):
        """Should be initialized after position update"""
        controller = RobotController()
        controller.update_positions_from_firmware(sample_firmware_positions)
        assert controller.check_differential_initialized() is True

    def test_initialized_when_v_nonzero(self):
        """Should be initialized when V is non-zero"""
        controller = RobotController()
        controller.update_differential_motors(10.0, 0.0)
        assert controller.check_differential_initialized() is True

    def test_initialized_when_w_nonzero(self):
        """Should be initialized when W is non-zero"""
        controller = RobotController()
        controller.update_differential_motors(0.0, 10.0)
        assert controller.check_differential_initialized() is True


class TestJointConfiguration:
    """Tests for joint configuration structure"""

    def test_all_joints_have_axis(self):
        """All joints should have axis mapping"""
        controller = RobotController()
        for joint_name, config in controller.joint_config.items():
            assert 'axis' in config

    def test_all_joints_have_type(self):
        """All joints should have type"""
        controller = RobotController()
        for joint_name, config in controller.joint_config.items():
            assert 'type' in config
            assert config['type'] in ['simple', 'coupled', 'differential', 'gripper']

    def test_differential_joints(self):
        """Art5 and Art6 should be differential type"""
        controller = RobotController()
        assert controller.joint_config['Art5']['type'] == 'differential'
        assert controller.joint_config['Art6']['type'] == 'differential'

    def test_gripper_config(self):
        """Gripper should have gripper type"""
        controller = RobotController()
        assert controller.joint_config['Gripper']['type'] == 'gripper'


class TestStateIntegration:
    """Integration tests for state management"""

    def test_full_update_cycle(self, sample_firmware_positions):
        """Test complete update cycle"""
        controller = RobotController()

        # Initial state
        assert controller.get_position_update_count() == 0

        # Update from firmware
        controller.update_positions_from_firmware(sample_firmware_positions)
        assert controller.get_position_update_count() == 1

        # Verify positions
        positions = controller.get_current_positions()
        assert positions['Art1'] == sample_firmware_positions['X']
        assert positions['Art2'] == sample_firmware_positions['Y']
        assert positions['Art3'] == sample_firmware_positions['Z']
        assert positions['Art4'] == sample_firmware_positions['U']

        # Verify differential calculation
        motor_v, motor_w = controller.get_differential_motor_positions()
        assert motor_v == sample_firmware_positions['V']
        assert motor_w == sample_firmware_positions['W']

    def test_differential_move_workflow(self, sample_firmware_positions):
        """Test workflow for making a differential move"""
        controller = RobotController()

        # Initialise from firmware
        controller.update_positions_from_firmware(sample_firmware_positions)

        # Calculate move for Art5
        new_v, new_w, kept_art6 = controller.calculate_differential_move('Art5', 45.0)

        # Apply the move (simulate sending to firmware and receiving response)
        controller.update_differential_motors(new_v, new_w)

        # Verify result
        positions = controller.get_current_positions()
        assert abs(positions['Art5'] - 45.0) < 1e-10
        assert abs(positions['Art6'] - kept_art6) < 1e-10
