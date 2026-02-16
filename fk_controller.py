"""
FK Controller Module
Handles forward kinematics calculations and joint movement orchestration

This module provides:
- FK calculation orchestration
- Joint movement logic (simple, coupled, differential)
- Slider/spinbox synchronization via callbacks
- MoveAll functionality
- 3D visualization update callbacks
"""

import logging
from typing import Dict, Optional, Callable, Tuple
from dataclasses import dataclass

import forward_kinematics as fk
import differential_kinematics as diff_kin
from command_builder import CommandBuilder

logger = logging.getLogger(__name__)


@dataclass
class JointValues:
    """Container for all joint angle values"""
    art1: float = 0.0
    art2: float = 0.0
    art3: float = 0.0
    art4: float = 0.0
    art5: float = 0.0
    art6: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary with Art names"""
        return {
            'Art1': self.art1, 'Art2': self.art2, 'Art3': self.art3,
            'Art4': self.art4, 'Art5': self.art5, 'Art6': self.art6
        }

    def to_list(self) -> list:
        """Convert to list [art1, art2, ..., art6]"""
        return [self.art1, self.art2, self.art3, self.art4, self.art5, self.art6]


@dataclass
class FKResult:
    """Container for FK calculation result"""
    valid: bool
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    error_msg: str = ""

    def position_tuple(self) -> Tuple[float, float, float]:
        """Get (x, y, z) position tuple"""
        return (self.x, self.y, self.z)

    def orientation_tuple(self) -> Tuple[float, float, float]:
        """Get (roll, pitch, yaw) orientation tuple"""
        return (self.roll, self.pitch, self.yaw)


class FKController:
    """
    Controls forward kinematics calculations and joint movements.

    This class handles FK calculation logic and joint movement orchestration,
    using callbacks for all GUI updates to maintain decoupling.
    """

    def __init__(
        self,
        robot_controller=None,
        command_sender=None,
        spinbox_update_callback: Optional[Callable[[str, float], None]] = None,
        slider_update_callback: Optional[Callable[[str, int], None]] = None,
        visualization_update_callback: Optional[Callable[[list], None]] = None,
        get_movement_params_callback: Optional[Callable[[], Tuple[str, str]]] = None,
        no_connection_callback: Optional[Callable[[], None]] = None
    ):
        """
        Initialize FK controller.

        Args:
            robot_controller: RobotController for joint config and differential calculations
            command_sender: CommandSender for sending movement commands
            spinbox_update_callback: Callback(joint_name, value) to update spinbox
            slider_update_callback: Callback(joint_name, slider_value) to update slider
            visualization_update_callback: Callback(joint_angles_list) to update 3D viz
            get_movement_params_callback: Callback() -> (movement_type, feedrate)
            no_connection_callback: Callback() when serial not connected
        """
        self.robot_controller = robot_controller
        self.command_sender = command_sender
        self.spinbox_update_callback = spinbox_update_callback
        self.slider_update_callback = slider_update_callback
        self.visualization_update_callback = visualization_update_callback
        self.get_movement_params_callback = get_movement_params_callback
        self.no_connection_callback = no_connection_callback

        # Jog mode state (synced from main controller)
        self.jog_mode_enabled = False

        # Current joint values (for calculations)
        self._current_values = JointValues()

        # Last FK result
        self.last_result: Optional[FKResult] = None

    @property
    def joint_config(self) -> Dict:
        """Get joint configuration from robot controller"""
        if self.robot_controller:
            return self.robot_controller.joint_config
        return {}

    def set_jog_mode(self, enabled: bool) -> None:
        """Set jog mode state"""
        self.jog_mode_enabled = enabled
        logger.debug(f"FK jog mode {'enabled' if enabled else 'disabled'}")

    def calculate_fk(self, joint_values: JointValues) -> FKResult:
        """
        Calculate forward kinematics for given joint values.

        Args:
            joint_values: JointValues with all joint angles in degrees

        Returns:
            FKResult with end effector position/orientation
        """
        self._current_values = joint_values

        try:
            # Calculate FK - returns list of joint positions including TCP
            positions = fk.compute_all_joint_positions(
                joint_values.art1, joint_values.art2, joint_values.art3,
                joint_values.art4, joint_values.art5, joint_values.art6
            )

            # TCP is the last position
            tcp = positions[-1]
            x, y, z = tcp[0], tcp[1], tcp[2]

            result = FKResult(
                valid=True,
                x=x, y=y, z=z
            )

            logger.debug(f"FK calculated: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

        except Exception as e:
            result = FKResult(valid=False, error_msg=str(e))
            logger.error(f"FK calculation error: {e}")

        self.last_result = result
        return result

    def update_visualization(self, joint_values: JointValues) -> None:
        """
        Update 3D visualization with current joint values.

        Args:
            joint_values: JointValues with all joint angles
        """
        if self.visualization_update_callback:
            angles = joint_values.to_list()
            self.visualization_update_callback(angles)
            logger.debug("3D visualization updated")

    def slider_changed(self, joint_name: str, slider_value: int) -> None:
        """
        Handle slider value change - update corresponding spinbox.

        Args:
            joint_name: Joint name (Art1, Art2, etc.)
            slider_value: Raw slider value (typically 10x the angle)
        """
        spinbox_value = slider_value / 10.0
        if self.spinbox_update_callback:
            self.spinbox_update_callback(joint_name, spinbox_value)

    def spinbox_changed(self, joint_name: str, spinbox_value: float) -> None:
        """
        Handle spinbox value change - update corresponding slider.

        Args:
            joint_name: Joint name (Art1, Art2, etc.)
            spinbox_value: Spinbox value in degrees
        """
        slider_value = int(spinbox_value * 10)
        if self.slider_update_callback:
            self.slider_update_callback(joint_name, slider_value)

    def move_joint(self, joint_name: str, joint_value: float) -> bool:
        """
        Move a single joint to specified position.

        Args:
            joint_name: Joint name (Art1, Art2, etc.)
            joint_value: Target angle in degrees

        Returns:
            True if command sent, False otherwise
        """
        if joint_name not in self.joint_config:
            logger.warning(f"Unknown joint: {joint_name}")
            return False

        config = self.joint_config[joint_name]

        # Handle based on joint type
        if config['type'] in ('simple', 'coupled'):
            return self._move_simple(joint_name, joint_value, config)
        elif config['type'] == 'differential':
            return self._move_differential(joint_name, joint_value, config)

        return False

    def _move_simple(self, joint_name: str, joint_value: float, config: Dict) -> bool:
        """
        Move a simple single-axis joint (also handles coupled motors).

        Args:
            joint_name: Joint name
            joint_value: Target angle in degrees
            config: Joint configuration dict

        Returns:
            True if command sent, False otherwise
        """
        # Log with appropriate detail based on joint type
        if config['type'] == 'coupled':
            logger.info(
                f"{config['log_name']} commanded to: {joint_value}° "
                f"Axis: {config['axis']} (Drives {config['drives']})"
            )
        else:
            logger.info(
                f"{config['log_name']} commanded to: {joint_value}° "
                f"Axis: {config['axis']}"
            )

        # Get movement parameters
        movement_type, feedrate = self._get_movement_params()

        # Build command
        command = CommandBuilder.build_single_axis_command(
            movement_type, config['axis'], joint_value, feedrate
        )

        # Send command
        return self._send_command(command)

    def _move_differential(self, joint_name: str, joint_value: float, config: Dict) -> bool:
        """
        Move a differential joint (Art5/Art6).

        Args:
            joint_name: Joint name (Art5 or Art6)
            joint_value: Target angle in degrees
            config: Joint configuration dict

        Returns:
            True if command sent, False otherwise
        """
        if not self.robot_controller:
            logger.error("No robot controller for differential calculation")
            return False

        # Check if we have valid position feedback
        if not self.robot_controller.check_differential_initialized():
            logger.warning(
                "No position feedback received yet - differential control may be inaccurate!"
            )

        # Get current joint values and set the target for the moving joint
        current_motor_v, current_motor_w = self.robot_controller.get_differential_motor_positions()
        current_art5, current_art6 = diff_kin.DifferentialKinematics.motor_to_joint(
            current_motor_v, current_motor_w
        )

        if joint_name == 'Art5':
            target_art5 = joint_value
            target_art6 = current_art6
        else:  # Art6
            target_art5 = current_art5
            target_art6 = joint_value

        # Calculate new motor positions
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(target_art5, target_art6)

        # Update tracked positions in controller
        self.robot_controller.update_differential_motors(motor_v, motor_w)

        logger.info(
            f"{config['log_name']} commanded to: {joint_value}° -> "
            f"Differential: V={motor_v:.2f}°, W={motor_w:.2f}°"
        )

        # Get movement parameters
        movement_type, feedrate = self._get_movement_params()

        # Build command (both V and W motors)
        command = CommandBuilder.build_axis_command(
            movement_type,
            {"V": motor_v, "W": motor_w},
            feedrate
        )

        # Send command
        return self._send_command(command)

    def move_all(self, joint_values: JointValues) -> bool:
        """
        Move all joints simultaneously.

        Args:
            joint_values: JointValues with all target angles

        Returns:
            True if command sent, False otherwise
        """
        # Calculate differential motor positions
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(
            joint_values.art5, joint_values.art6
        )

        logger.info(
            f"MoveAll: Art5={joint_values.art5}° Art6={joint_values.art6}° -> "
            f"Differential: V={motor_v:.2f}° W={motor_w:.2f}°"
        )

        # Get movement parameters
        movement_type, feedrate = self._get_movement_params()

        # Build command for all axes
        command = CommandBuilder.build_axis_command(
            movement_type,
            {
                "X": joint_values.art1,
                "Y": joint_values.art2,
                "Z": joint_values.art3,
                "U": joint_values.art4,
                "V": motor_v,
                "W": motor_w
            },
            feedrate
        )

        # Send command
        return self._send_command(command)

    def sync_to_actual(self, positions: Dict[str, float]) -> JointValues:
        """
        Create JointValues from actual robot positions.

        Args:
            positions: Dictionary with Art1-Art6 position values

        Returns:
            JointValues with actual positions
        """
        return JointValues(
            art1=positions.get('Art1', 0.0),
            art2=positions.get('Art2', 0.0),
            art3=positions.get('Art3', 0.0),
            art4=positions.get('Art4', 0.0),
            art5=positions.get('Art5', 0.0),
            art6=positions.get('Art6', 0.0)
        )

    def _get_movement_params(self) -> Tuple[str, str]:
        """Get current movement parameters (type and feedrate)"""
        if self.get_movement_params_callback:
            return self.get_movement_params_callback()
        return ("G0", "")  # Default: rapid move, no feedrate

    def _send_command(self, command: str) -> bool:
        """
        Send command via command sender.

        Args:
            command: G-code command to send

        Returns:
            True if sent, False otherwise
        """
        if not self.command_sender:
            logger.warning("No command sender configured")
            return False

        result = self.command_sender.send_if_connected(
            command,
            error_callback=self.no_connection_callback
        )

        if not result:
            logger.warning("Command not sent - serial not connected")

        return result


def create_joint_values(
    art1: float = 0, art2: float = 0, art3: float = 0,
    art4: float = 0, art5: float = 0, art6: float = 0
) -> JointValues:
    """
    Create JointValues with specified angles.

    Args:
        art1-art6: Joint angles in degrees

    Returns:
        JointValues instance
    """
    return JointValues(
        art1=art1, art2=art2, art3=art3,
        art4=art4, art5=art5, art6=art6
    )
