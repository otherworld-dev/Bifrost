"""
IK Controller Module
Handles inverse kinematics calculations and jog mode behavior

This module provides:
- IK calculation orchestration with debouncing
- Jog mode behavior for IK adjustments
- Callbacks for GUI updates (decoupled from Qt widgets)
- Frame-aware IK solving (automatic conversion from active frame to base)
"""

import logging
import numpy as np
from typing import Dict, Optional, Callable, Tuple, TYPE_CHECKING
from dataclasses import dataclass

import inverse_kinematics as ik
import forward_kinematics as fk
from coordinate_frames import FrameManager, pose_from_xyz_rpy, pose_to_xyz_rpy

if TYPE_CHECKING:
    from coordinate_frames import FrameManager

logger = logging.getLogger(__name__)


@dataclass
class IKTarget:
    """Container for IK target position"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = -np.pi / 2  # Default: tool pointing down
    yaw: float = 0.0


@dataclass
class IKSolutionResult:
    """Container for IK solution with all joint angles"""
    valid: bool
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    q4: float = 0.0
    q5: float = 0.0
    q6: float = 0.0
    error_msg: str = ""

    def to_dict(self) -> Dict[str, float]:
        """Convert joint angles to dictionary"""
        return {
            'q1': self.q1, 'q2': self.q2, 'q3': self.q3,
            'q4': self.q4, 'q5': self.q5, 'q6': self.q6
        }


class IKController:
    """
    Controls inverse kinematics calculations and jog mode behavior.

    This class handles IK calculation logic separate from GUI widgets,
    using callbacks for all GUI updates.
    """

    # Style constants
    VALID_STYLE = "background-color:rgb(200, 255, 200)"  # Light green
    INVALID_STYLE = "background-color:rgb(255, 200, 200)"  # Light red

    def __init__(
        self,
        output_update_callback: Optional[Callable[[str, str, str], None]] = None,
        spinbox_update_callback: Optional[Callable[[Dict[str, float]], None]] = None,
        style_update_callback: Optional[Callable[[str], None]] = None,
        move_callback: Optional[Callable[[], None]] = None,
        frame_manager: Optional[FrameManager] = None
    ):
        """
        Initialize IK controller.

        Args:
            output_update_callback: Callback(x_text, y_text, z_text) to update output labels
            spinbox_update_callback: Callback(joint_values_dict) to update FK spinboxes
            style_update_callback: Callback(style_string) to update output frame style
            move_callback: Callback() to execute movement (for jog mode)
            frame_manager: Optional FrameManager for frame-aware IK solving
        """
        self.output_update_callback = output_update_callback
        self.spinbox_update_callback = spinbox_update_callback
        self.style_update_callback = style_update_callback
        self.move_callback = move_callback
        self.frame_manager = frame_manager

        # State
        self.jog_mode_enabled = False
        self.last_solution: Optional[IKSolutionResult] = None
        self.current_target = IKTarget()

    def set_jog_mode(self, enabled: bool) -> None:
        """
        Enable or disable jog mode.

        Args:
            enabled: Whether jog mode is enabled
        """
        self.jog_mode_enabled = enabled
        logger.debug(f"IK jog mode {'enabled' if enabled else 'disabled'}")

    def set_frame_manager(self, frame_manager: FrameManager) -> None:
        """
        Set or update the frame manager.

        Args:
            frame_manager: FrameManager instance
        """
        self.frame_manager = frame_manager
        logger.debug("Frame manager set on IK controller")

    def _transform_target_to_base(self, target: IKTarget) -> IKTarget:
        """
        Transform target from active frame to base frame.

        If no frame manager or active frame is 'base', returns target unchanged.

        Args:
            target: IKTarget in active frame coordinates

        Returns:
            IKTarget in base frame coordinates
        """
        if self.frame_manager is None:
            return target

        active_frame = self.frame_manager.get_active_frame()
        if active_frame == "base":
            return target

        # Build 4x4 pose matrix from target
        # Note: target orientation is in radians
        target_pose = pose_from_xyz_rpy(
            target.x, target.y, target.z,
            target.roll, target.pitch, target.yaw
        )

        # Transform from active frame to base frame
        base_pose = self.frame_manager.transform_pose(target_pose, active_frame, "base")

        # Account for tool offset if not default
        tool_offset = self.frame_manager.get_active_tool_offset()
        # The IK expects the TCP target, so if we have a tool offset,
        # we need to remove it (IK will position TCP, tool tip extends from there)
        if not np.allclose(tool_offset, np.eye(4)):
            # Target is tool tip position, need to back-calculate TCP position
            # TCP_pose @ tool_offset = tool_tip_pose
            # TCP_pose = tool_tip_pose @ inv(tool_offset)
            base_pose = base_pose @ np.linalg.inv(tool_offset)

        # Extract position and orientation from transformed pose
        x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(base_pose)

        logger.debug(
            f"IK frame transform: {active_frame} -> base: "
            f"({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) -> ({x:.1f}, {y:.1f}, {z:.1f})"
        )

        return IKTarget(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

    def calculate_ik(self, target: IKTarget) -> IKSolutionResult:
        """
        Calculate 6-DOF inverse kinematics for target position.

        If a frame manager is set, the target is automatically transformed
        from the active frame to base frame before solving.

        Args:
            target: IKTarget with position and orientation (in active frame)

        Returns:
            IKSolutionResult with joint angles or error
        """
        self.current_target = target

        # Transform target from active frame to base frame
        base_target = self._transform_target_to_base(target)

        logger.info(f"IK 6-DOF: Calculating for target X={base_target.x}, Y={base_target.y}, Z={base_target.z}")

        # Solve full 6-DOF IK in base frame
        solution = ik.solve_ik_full(
            base_target.x, base_target.y, base_target.z,
            roll=base_target.roll, pitch=base_target.pitch, yaw=base_target.yaw
        )

        # Convert to our result type
        if solution.valid:
            result = IKSolutionResult(
                valid=True,
                q1=solution.q1,
                q2=solution.q2,
                q3=solution.q3,
                q4=solution.q4,
                q5=solution.q5,
                q6=solution.q6
            )
            logger.info(
                f"IK 6-DOF: Valid solution - "
                f"q1={result.q1:.2f}°, q2={result.q2:.2f}°, q3={result.q3:.2f}°, "
                f"q4={result.q4:.2f}°, q5={result.q5:.2f}°, q6={result.q6:.2f}°"
            )
        else:
            result = IKSolutionResult(
                valid=False,
                error_msg=solution.error_msg
            )
            logger.warning(f"IK 6-DOF: Invalid solution - {solution.error_msg}")

        self.last_solution = result
        return result

    def calculate_and_update(self, x: float, y: float, z: float,
                             roll_deg: float = 0.0, pitch_deg: float = -90.0,
                             yaw_deg: float = 0.0) -> IKSolutionResult:
        """
        Calculate IK and update all GUI elements via callbacks.

        This is the main entry point for IK calculation with GUI updates.

        Args:
            x, y, z: Target position in mm
            roll_deg, pitch_deg, yaw_deg: Target orientation in degrees

        Returns:
            IKSolutionResult with joint angles or error
        """
        target = IKTarget(
            x=x, y=y, z=z,
            roll=np.radians(roll_deg),
            pitch=np.radians(pitch_deg),
            yaw=np.radians(yaw_deg)
        )
        result = self.calculate_ik(target)

        # Update GUI via callbacks
        self._update_gui(result)

        return result

    def adjust_axis(self, axis: str, current_value: float, delta: float) -> float:
        """
        Calculate new value for axis adjustment.

        Args:
            axis: Axis letter ('X', 'Y', 'Z')
            current_value: Current axis value
            delta: Amount to add

        Returns:
            New axis value
        """
        new_value = current_value + delta
        logger.debug(f"IK axis {axis} adjusted: {current_value} + {delta} = {new_value}")
        return new_value

    def handle_jog_adjustment(self, axis: str, delta: float, get_current_values: Callable) -> Optional[float]:
        """
        Handle IK jog adjustment - adjust axis and optionally trigger movement.

        Args:
            axis: Axis letter ('X', 'Y', 'Z')
            delta: Amount to adjust
            get_current_values: Callback() -> (x, y, z) to get current input values

        Returns:
            New axis value, or None if adjustment not possible
        """
        x, y, z = get_current_values()

        # Calculate new value
        if axis == 'X':
            new_value = x + delta
        elif axis == 'Y':
            new_value = y + delta
        elif axis == 'Z':
            new_value = z + delta
        else:
            logger.warning(f"Unknown IK axis: {axis}")
            return None

        logger.debug(f"[JOG MODE] IK {axis} jogged by {delta}mm")

        return new_value

    def execute_jog_move_if_valid(self) -> bool:
        """
        Execute movement if in jog mode and last solution was valid.

        Returns:
            True if movement was executed, False otherwise
        """
        if not self.jog_mode_enabled:
            return False

        if self.last_solution and self.last_solution.valid:
            if self.move_callback:
                self.move_callback()
                logger.debug("[JOG MODE] IK solution valid, executing movement")
                return True
            else:
                logger.warning("[JOG MODE] No move callback configured")
                return False
        else:
            logger.warning("[JOG MODE] IK solution invalid, movement not executed")
            return False

    def calculate_jacobian_move(self, current_joints: list, cartesian_delta: list,
                                damping: float = 0.5) -> Optional[Dict[str, float]]:
        """
        Compute new joint angles for a Cartesian increment using the Jacobian.

        Uses separate position (3x6) and orientation (3x6) sub-Jacobians
        to avoid scaling issues between mm and radians.

        Args:
            current_joints: [q1, q2, q3, q4, q5, q6] in degrees
            cartesian_delta: [dx, dy, dz, droll, dpitch, dyaw] (mm and radians)
            damping: DLS damping factor (higher = more stable near singularities)

        Returns:
            Dict with joint angles {'Art1': ..., 'Art6': ...} or None on failure
        """
        try:
            q = np.array(current_joints, dtype=np.float64)
            dx = np.array(cartesian_delta, dtype=np.float64)

            J_full = fk.compute_jacobian(*q)

            # Split into position and orientation components
            dp = dx[:3]  # position delta (mm)
            dw = dx[3:]  # orientation delta (radians)

            has_pos = np.any(np.abs(dp) > 1e-10)
            has_ori = np.any(np.abs(dw) > 1e-10)

            dq = np.zeros(6)

            if has_pos and not has_ori:
                # Position-only: use 3x6 position sub-Jacobian
                J_pos = J_full[:3, :]
                JJT = J_pos @ J_pos.T
                dls = J_pos.T @ np.linalg.inv(JJT + damping**2 * np.eye(3))
                dq = dls @ dp

            elif has_ori and not has_pos:
                # Orientation-only: use 3x6 orientation sub-Jacobian
                J_ori = J_full[3:, :]
                JJT = J_ori @ J_ori.T
                dls = J_ori.T @ np.linalg.inv(JJT + (damping * 0.01)**2 * np.eye(3))
                dq = dls @ dw

            else:
                # Mixed: scale Jacobian to balance mm and rad
                # Scale position rows by 1/100 so both are ~same magnitude
                J_scaled = J_full.copy()
                J_scaled[:3, :] /= 100.0
                dx_scaled = dx.copy()
                dx_scaled[:3] /= 100.0
                JJT = J_scaled @ J_scaled.T
                dls = J_scaled.T @ np.linalg.inv(JJT + damping**2 * np.eye(6))
                dq = dls @ dx_scaled

            new_q = q + dq

            # Clamp to joint limits
            from inverse_kinematics import JOINT_LIMITS
            for i, joint_num in enumerate(range(1, 7)):
                lo, hi = JOINT_LIMITS[joint_num]
                new_q[i] = np.clip(new_q[i], lo, hi)

            logger.debug(
                f"[JACOBIAN] dq=[{dq[0]:.3f}, {dq[1]:.3f}, {dq[2]:.3f}, "
                f"{dq[3]:.3f}, {dq[4]:.3f}, {dq[5]:.3f}]"
            )

            return {
                'Art1': float(new_q[0]),
                'Art2': float(new_q[1]),
                'Art3': float(new_q[2]),
                'Art4': float(new_q[3]),
                'Art5': float(new_q[4]),
                'Art6': float(new_q[5]),
            }

        except Exception as e:
            logger.error(f"[JACOBIAN] Failed: {e}")
            return None

    def is_solution_valid(self) -> bool:
        """Check if the last IK solution was valid."""
        return self.last_solution is not None and self.last_solution.valid

    def get_joint_angles(self) -> Optional[Dict[str, float]]:
        """
        Get joint angles from last valid solution.

        Returns:
            Dictionary with joint angles or None if no valid solution
        """
        if self.last_solution and self.last_solution.valid:
            return self.last_solution.to_dict()
        return None

    def _update_gui(self, result: IKSolutionResult) -> None:
        """Update GUI elements via callbacks based on IK result."""
        if result.valid:
            # Update output labels
            if self.output_update_callback:
                self.output_update_callback(
                    f"{result.q1:.2f}º",
                    f"{result.q2:.2f}º",
                    f"{result.q3:.2f}º"
                )

            # Update FK spinboxes
            if self.spinbox_update_callback:
                self.spinbox_update_callback({
                    'Art1': result.q1,
                    'Art2': result.q2,
                    'Art3': result.q3,
                    'Art4': result.q4,
                    'Art5': result.q5,
                    'Art6': result.q6
                })

            # Update style (valid)
            if self.style_update_callback:
                self.style_update_callback(self.VALID_STYLE)
        else:
            # Update output labels with placeholder
            if self.output_update_callback:
                self.output_update_callback("--", "--", "--")

            # Update style (invalid)
            if self.style_update_callback:
                self.style_update_callback(self.INVALID_STYLE)


def create_ik_target(x: float, y: float, z: float,
                     roll: float = 0, pitch: float = -np.pi/2, yaw: float = 0) -> IKTarget:
    """
    Create an IK target with default tool-down orientation.

    Args:
        x, y, z: Target position in mm
        roll, pitch, yaw: Orientation in radians (default: tool pointing down)

    Returns:
        IKTarget instance
    """
    return IKTarget(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
