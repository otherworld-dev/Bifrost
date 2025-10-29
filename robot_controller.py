"""
Robot Controller Module
Manages robot state, position tracking, and movement coordination

This class encapsulates all robot control logic that was previously scattered
throughout the BifrostGUI class. It provides a clean interface for:
- Joint position tracking and validation
- Differential kinematics state management
- Movement command coordination
- Position history integration
"""

from typing import Dict, Tuple, Optional, Callable
import logging
import config
import differential_kinematics as diff_kin

logger = logging.getLogger(__name__)


class RobotController:
    """
    Central controller for robot state and movement coordination

    Responsibilities:
    - Track current joint positions (including differential motors)
    - Validate position commands against safety limits
    - Manage differential kinematics state (Art5/Art6 -> Motor V/W)
    - Coordinate movement commands
    - Integrate with position history tracking
    """

    def __init__(self):
        """Initialize robot controller with default state"""

        # Joint configuration: maps joint names to firmware axes
        # This defines the robot's kinematic structure
        self.joint_config = {
            'Art1': {
                'axis': 'X',
                'type': 'simple',
                'log_name': 'Art1 (Joint 1)',
                'limits': config.POSITION_LIMITS.get('X', (-180, 180))
            },
            'Art2': {
                'axis': 'Y',
                'type': 'coupled',
                'log_name': 'Art2 (Joint 2 - COUPLED)',
                'drives': '1+2',
                'limits': config.POSITION_LIMITS.get('Y', (-180, 180))
            },
            'Art3': {
                'axis': 'Z',
                'type': 'simple',
                'log_name': 'Art3 (Joint 3)',
                'limits': config.POSITION_LIMITS.get('Z', (-180, 180))
            },
            'Art4': {
                'axis': 'U',
                'type': 'simple',
                'log_name': 'Art4 (Joint 4)',
                'limits': config.POSITION_LIMITS.get('U', (-180, 180))
            },
            'Art5': {
                'axis': 'V+W',
                'type': 'differential',
                'log_name': 'Art5 (DIFFERENTIAL)',
                'limits': (-180, 180)  # Joint space limits
            },
            'Art6': {
                'axis': 'V+W',
                'type': 'differential',
                'log_name': 'Art6 (DIFFERENTIAL)',
                'limits': (-180, 180)  # Joint space limits
            },
            'Gripper': {
                'axis': 'GRIPPER',
                'type': 'gripper',
                'log_name': 'Gripper',
                'limits': (0, 100)  # Percentage
            }
        }

        # Current positions in joint space (degrees)
        self.current_positions = {
            'Art1': 0.0,
            'Art2': 0.0,
            'Art3': 0.0,
            'Art4': 0.0,
            'Art5': 0.0,
            'Art6': 0.0,
            'Gripper': 0.0
        }

        # Differential motor positions (Motor V and W)
        # These are the actual motor positions, calculated from Art5/Art6
        self.current_motor_v = 0.0
        self.current_motor_w = 0.0

        # Desired joint positions (used for differential control)
        self.desired_art5 = 0.0
        self.desired_art6 = 0.0

        # Last valid motor positions (for validation)
        self.last_valid_positions = {
            'X': 0.0, 'Y': 0.0, 'Z': 0.0,
            'U': 0.0, 'V': 0.0, 'W': 0.0
        }

        # Position update tracking
        self.position_update_count = 0

        logger.info("RobotController initialized")
        self._log_configuration()

    def _log_configuration(self) -> None:
        """Log robot configuration for debugging"""
        logger.info("="*60)
        logger.info("ROBOT CONTROLLER CONFIGURATION")
        logger.info("="*60)
        logger.info("Joint Configuration:")
        for joint_name, config in self.joint_config.items():
            logger.info(f"  {joint_name}: {config['log_name']} -> Axis: {config['axis']} (Type: {config['type']})")
        logger.info("="*60)

    def initialize_from_spinboxes(self, spinbox_values: Dict[str, float]) -> None:
        """
        Initialize controller state from GUI spinbox values

        Args:
            spinbox_values: Dictionary mapping joint names to current spinbox values
                          e.g., {'Art1': 0.0, 'Art2': 0.0, ...}
        """
        for joint_name, value in spinbox_values.items():
            if joint_name in self.current_positions:
                self.current_positions[joint_name] = value

        # Initialize differential motor positions from Art5/Art6
        initial_art5 = self.current_positions['Art5']
        initial_art6 = self.current_positions['Art6']
        self.current_motor_v, self.current_motor_w = diff_kin.DifferentialKinematics.joint_to_motor(
            initial_art5, initial_art6
        )
        self.desired_art5 = initial_art5
        self.desired_art6 = initial_art6

        logger.info(f"Initialized from spinboxes - Art5={initial_art5}°, Art6={initial_art6}° -> V={self.current_motor_v}°, W={self.current_motor_w}°")

    def validate_position(self, axis: str, value: float) -> Tuple[bool, float]:
        """
        Validate position value for reasonableness

        Args:
            axis: Axis name (X, Y, Z, U, V, W)
            value: Position value in degrees

        Returns:
            Tuple of (is_valid, sanitized_value)
                - is_valid: True if value is within acceptable range
                - sanitized_value: Clamped value if out of range, original if valid
        """
        if axis not in config.POSITION_LIMITS:
            return (True, value)  # Unknown axis, accept as-is

        min_val, max_val = config.POSITION_LIMITS[axis]

        # Check absolute limits
        if not (min_val <= value <= max_val):
            logger.warning(f"Position out of bounds: {axis}={value:.2f}° (limits: {min_val} to {max_val})")
            # Clamp to limits
            clamped_value = max(min_val, min(max_val, value))
            return (False, clamped_value)

        # Check for impossible changes (position jumped too far too fast)
        if axis in self.last_valid_positions:
            last_value = self.last_valid_positions[axis]
            change = abs(value - last_value)
            if change > config.MAX_POSITION_CHANGE_PER_UPDATE:
                logger.warning(
                    f"Impossible position change detected: {axis} changed {change:.2f}° "
                    f"in one update (max: {config.MAX_POSITION_CHANGE_PER_UPDATE}°)"
                )
                # Use last valid value
                return (False, last_value)

        return (True, value)

    def update_positions_from_firmware(self, firmware_positions: Dict[str, float]) -> Dict[str, float]:
        """
        Update internal state from firmware position feedback (M114 response)

        Args:
            firmware_positions: Dictionary of axis positions from firmware
                              e.g., {'X': 10.0, 'Y': 20.0, 'Z': 0.0, 'U': 0.0, 'V': -110.0, 'W': -290.0}

        Returns:
            Dictionary of validated positions including calculated Art5/Art6
        """
        # Validate all positions
        validated = {}
        for axis in ['X', 'Y', 'Z', 'U', 'V', 'W']:
            valid, value = self.validate_position(axis, firmware_positions.get(axis, 0.0))
            validated[axis] = value
            self.last_valid_positions[axis] = value

            if not valid:
                logger.warning(f"Position validation corrected for {axis}")

        # Update motor positions
        self.current_motor_v = validated['V']
        self.current_motor_w = validated['W']

        # Calculate joint angles from differential motor positions
        art5, art6 = diff_kin.DifferentialKinematics.motor_to_joint(
            self.current_motor_v, self.current_motor_w
        )

        # Update joint positions
        self.current_positions['Art1'] = validated['X']
        self.current_positions['Art2'] = validated['Y']
        self.current_positions['Art3'] = validated['Z']
        self.current_positions['Art4'] = validated['U']
        self.current_positions['Art5'] = art5
        self.current_positions['Art6'] = art6

        # Track desired positions
        self.desired_art5 = art5
        self.desired_art6 = art6

        # Increment update counter
        self.position_update_count += 1

        # Add calculated values to return dict
        validated['Art5'] = art5
        validated['Art6'] = art6

        return validated

    def get_current_positions(self) -> Dict[str, float]:
        """
        Get current joint positions

        Returns:
            Dictionary of current positions for all joints
        """
        return self.current_positions.copy()

    def get_differential_motor_positions(self) -> Tuple[float, float]:
        """
        Get current differential motor positions

        Returns:
            Tuple of (motor_v, motor_w) in degrees
        """
        return (self.current_motor_v, self.current_motor_w)

    def calculate_differential_move(self, joint_name: str, target_value: float) -> Tuple[float, float, float]:
        """
        Calculate differential motor positions for moving Art5 or Art6

        Args:
            joint_name: 'Art5' or 'Art6'
            target_value: Desired joint angle in degrees

        Returns:
            Tuple of (motor_v, motor_w, kept_joint_value)
                - motor_v, motor_w: New motor positions
                - kept_joint_value: Value of the other joint that's being held constant

        Raises:
            ValueError: If joint_name is not 'Art5' or 'Art6'
        """
        if joint_name == 'Art5':
            motor_v, motor_w, kept_value = diff_kin.DifferentialKinematics.move_art5_only(
                self.current_motor_v, self.current_motor_w, target_value
            )
            kept_joint = 'Art6'
        elif joint_name == 'Art6':
            motor_v, motor_w, kept_value = diff_kin.DifferentialKinematics.move_art6_only(
                self.current_motor_v, self.current_motor_w, target_value
            )
            kept_joint = 'Art5'
        else:
            raise ValueError(f"Invalid joint name for differential move: {joint_name}. Must be 'Art5' or 'Art6'.")

        # Log the calculation
        current_art5, current_art6 = diff_kin.DifferentialKinematics.motor_to_joint(
            self.current_motor_v, self.current_motor_w
        )
        logger.info(f"Differential move calculation for {joint_name}:")
        logger.info(f"  BEFORE: Motor_V={self.current_motor_v:.2f}° Motor_W={self.current_motor_w:.2f}° -> Art5={current_art5:.2f}° Art6={current_art6:.2f}°")
        logger.info(f"  AFTER:  Motor_V={motor_v:.2f}° Motor_W={motor_w:.2f}° -> {joint_name}={target_value:.2f}° {kept_joint}={kept_value:.2f}° ({kept_joint} kept)")

        return (motor_v, motor_w, kept_value)

    def update_differential_motors(self, motor_v: float, motor_w: float) -> None:
        """
        Update tracked differential motor positions

        Args:
            motor_v: New Motor V position
            motor_w: New Motor W position
        """
        self.current_motor_v = motor_v
        self.current_motor_w = motor_w

        # Update joint positions from motors
        art5, art6 = diff_kin.DifferentialKinematics.motor_to_joint(motor_v, motor_w)
        self.current_positions['Art5'] = art5
        self.current_positions['Art6'] = art6
        self.desired_art5 = art5
        self.desired_art6 = art6

    def check_differential_initialized(self) -> bool:
        """
        Check if differential motors have been initialized with position feedback

        Returns:
            True if motors are initialized (not at 0,0), False otherwise
        """
        return not (self.current_motor_v == 0.0 and self.current_motor_w == 0.0)

    def get_position_update_count(self) -> int:
        """
        Get the number of position updates received

        Returns:
            Update count
        """
        return self.position_update_count


if __name__ == "__main__":
    # Test the RobotController
    import sys
    logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

    print("RobotController Test")
    print("=" * 60)

    # Create controller
    controller = RobotController()

    # Test 1: Initialize from spinboxes
    print("\nTest 1: Initialize from spinboxes")
    spinbox_values = {
        'Art1': 0.0,
        'Art2': 90.0,
        'Art3': 45.0,
        'Art4': 0.0,
        'Art5': 30.0,
        'Art6': 45.0,
        'Gripper': 0.0
    }
    controller.initialize_from_spinboxes(spinbox_values)
    print(f"Current positions: {controller.get_current_positions()}")
    print(f"Differential motors: V={controller.current_motor_v:.2f}°, W={controller.current_motor_w:.2f}°")

    # Test 2: Position validation
    print("\nTest 2: Position validation")
    valid, value = controller.validate_position('X', 150.0)
    print(f"Valid position (X=150°): {valid}, value={value}")

    valid, value = controller.validate_position('X', 200.0)
    print(f"Out of range (X=200°): {valid}, value={value} (should be clamped)")

    # Test 3: Update from firmware
    print("\nTest 3: Update from firmware")
    firmware_data = {
        'X': 10.0,
        'Y': 20.0,
        'Z': 30.0,
        'U': 5.0,
        'V': 85.0,  # Art6 + Art5 = 45 + 40 = 85
        'W': 5.0    # Art6 - Art5 = 45 - 40 = 5
    }
    updated = controller.update_positions_from_firmware(firmware_data)
    print(f"Updated positions (with Art5/Art6 calculated): {updated}")

    # Test 4: Differential move calculation
    print("\nTest 4: Differential move calculation")
    motor_v, motor_w, kept = controller.calculate_differential_move('Art5', 50.0)
    print(f"Move Art5 to 50°: V={motor_v:.2f}°, W={motor_w:.2f}°, Art6 kept at {kept:.2f}°")

    print("\n" + "=" * 60)
    print("All tests complete!")
