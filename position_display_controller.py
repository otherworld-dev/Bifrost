"""
Position Display Controller Module
Handles position updates, parsing, and GUI display throttling

This module provides:
- M114 position response parsing and processing
- M119 endstop response parsing
- GUI update throttling
- Position history recording with calibration
"""

import logging
import time
from typing import Dict, Optional, Callable
from dataclasses import dataclass

import config
import parsing_patterns
from robot_controller import RobotController

logger = logging.getLogger(__name__)


@dataclass
class PositionData:
    """Container for processed position data"""
    # Raw axis positions
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    # Calculated joint positions (Art5/Art6 from differential)
    art5: float = 0.0
    art6: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary format"""
        return {
            'X': self.x, 'Y': self.y, 'Z': self.z,
            'U': self.u, 'V': self.v, 'W': self.w,
            'Art5': self.art5, 'Art6': self.art6
        }


@dataclass
class EndstopStatus:
    """Container for endstop status"""
    axis: str
    status: str
    is_triggered: bool

    @property
    def display_text(self) -> str:
        """Get compact display text"""
        if "not stopped" in self.status:
            return f"{self.axis}: OK"
        elif "min" in self.status:
            return f"{self.axis}: MIN"
        elif "max" in self.status:
            return f"{self.axis}: MAX"
        else:
            return f"{self.axis}: {self.status[:6]}"

    @property
    def style_class(self) -> str:
        """Get style class for display"""
        if "not stopped" in self.status:
            return "ok"  # Green
        elif "min" in self.status or "max" in self.status:
            return "triggered"  # Red
        else:
            return "unknown"  # Yellow


class PositionDisplayController:
    """
    Controls position display updates with throttling and history recording.

    This class handles the logic of position processing separate from GUI widgets.
    It uses callbacks to update the GUI, keeping display logic decoupled.
    """

    # State colour mapping for visual feedback
    STATE_COLORS = {
        'Idle': 'rgb(0, 255, 0)',      # Green - ready
        'Run': 'rgb(0, 255, 0)',       # Green - running
        'Home': 'rgb(85, 255, 255)',   # Cyan - homing
        'Alarm': 'rgb(255, 255, 0)',   # Yellow - warning
        'Hold': 'rgb(255, 0, 0)',      # Red - stopped
    }

    # Endstop style colours
    ENDSTOP_STYLES = {
        'ok': "background-color: rgb(200, 255, 200); padding: 2px; border-radius: 3px;",
        'triggered': "background-color: rgb(255, 200, 200); padding: 2px; border-radius: 3px;",
        'unknown': "background-color: rgb(255, 255, 200); padding: 2px; border-radius: 3px;",
    }

    def __init__(
        self,
        robot_controller: RobotController,
        position_history=None,
        gui_update_callback: Optional[Callable] = None,
        state_update_callback: Optional[Callable] = None,
        endstop_update_callback: Optional[Callable] = None
    ):
        """
        Initialize position display controller.

        Args:
            robot_controller: RobotController instance for state management
            position_history: Optional PositionHistory instance for recording
            gui_update_callback: Callback(positions_dict) to update GUI labels
            state_update_callback: Callback(state, color) to update state display
            endstop_update_callback: Callback(axis, text, style) to update endstop labels
        """
        self.robot_controller = robot_controller
        self.position_history = position_history

        # Callbacks for GUI updates (decouples from Qt widgets)
        self.gui_update_callback = gui_update_callback
        self.state_update_callback = state_update_callback
        self.endstop_update_callback = endstop_update_callback

        # Throttling state
        self.last_gui_update_time = 0.0
        self.position_update_count = 0

        # Backward compatibility references (updated from robot_controller)
        self.current_motor_v = 0.0
        self.current_motor_w = 0.0
        self.desired_art5 = 0.0
        self.desired_art6 = 0.0

    def process_m114_response(self, data: str) -> Optional[Dict[str, float]]:
        """
        Process M114 position response and update all displays.

        This method orchestrates position processing through several steps:
        1. Parse raw M114 response
        2. Validate all positions
        3. Update internal state and differential kinematics
        4. Record to position history
        5. Update GUI displays (throttled)

        Args:
            data: Raw M114 response string from firmware

        Returns:
            Processed positions dict or None if parsing failed
        """
        try:
            # Step 1: Parse M114 response
            pos_dict = self._parse_m114(data)
            if not pos_dict:
                return None

            # Step 2: Validate we have differential axes (V and W required)
            if 'V' not in pos_dict or 'W' not in pos_dict:
                logger.warning("M114 response missing V or W axis - skipping update")
                return None

            # Step 3: Validate all positions
            positions = self._validate_all_positions(pos_dict)

            # Step 4: Update internal state and calculate differential kinematics
            positions = self._update_internal_state(positions)

            # Step 5: Record to position history (sampled)
            self._record_position_history(positions)

            # Step 6: Update GUI displays (throttled)
            self._update_gui_positions(positions)

            return positions

        except (ValueError, KeyError, IndexError) as e:
            logger.error(f"Error parsing M114 response: {e}")
            logger.debug(f"Problematic data: {data}")
            logger.exception("Position parsing error")
            return None

    def process_m119_response(self, data: str) -> Optional[Dict[str, EndstopStatus]]:
        """
        Process M119 endstop response and update displays.

        Args:
            data: Raw M119 response string from firmware

        Returns:
            Dictionary of EndstopStatus by axis or None if parsing failed
        """
        try:
            endstops = parsing_patterns.parse_m119_response(data)
            if not endstops:
                return None

            result = {}
            for axis in ['X', 'Y', 'Z', 'U', 'V', 'W']:
                if axis in endstops:
                    status = endstops[axis]
                    is_triggered = "not stopped" not in status
                    endstop_status = EndstopStatus(
                        axis=axis,
                        status=status,
                        is_triggered=is_triggered
                    )
                    result[axis] = endstop_status

                    # Update GUI via callback
                    if self.endstop_update_callback:
                        style = self.ENDSTOP_STYLES.get(
                            endstop_status.style_class,
                            self.ENDSTOP_STYLES['unknown']
                        )
                        self.endstop_update_callback(
                            axis,
                            endstop_status.display_text,
                            style
                        )

            return result

        except Exception as e:
            logger.error(f"Error parsing M119 endstop response: {e}")
            logger.debug(f"Problematic data: {data}")
            return None

    def update_state(self, state: str) -> None:
        """
        Update robot state display.

        Args:
            state: State string (Idle, Run, Home, Alarm, Hold)
        """
        color = self.STATE_COLORS.get(state, 'rgb(255, 255, 255)')

        if self.state_update_callback:
            self.state_update_callback(state, color)

    def _parse_m114(self, data: str) -> Optional[Dict[str, float]]:
        """Parse M114 response using parsing_patterns module."""
        result = parsing_patterns.parse_m114_response(data)
        return result if result else None

    def _validate_all_positions(self, pos_dict: Dict[str, float]) -> Dict[str, float]:
        """
        Validate all axis positions and return sanitized values.

        Args:
            pos_dict: Dictionary of raw positions from M114

        Returns:
            Dictionary of validated positions for all axes
        """
        AXES = ['X', 'Y', 'Z', 'U', 'V', 'W']
        positions = {}

        for axis in AXES:
            valid, value = self.robot_controller.validate_position(
                axis, pos_dict.get(axis, 0.0)
            )
            positions[axis] = value

            if not valid:
                logger.warning(f"Position validation corrected for {axis}")

        return positions

    def _update_internal_state(self, positions: Dict[str, float]) -> Dict[str, float]:
        """
        Update internal tracking variables from validated positions.

        Args:
            positions: Dictionary of validated axis positions

        Returns:
            Updated positions with calculated Art5/Art6
        """
        # Use RobotController to update all state
        updated_positions = self.robot_controller.update_positions_from_firmware(positions)

        # Keep local references for backward compatibility
        self.current_motor_v = self.robot_controller.current_motor_v
        self.current_motor_w = self.robot_controller.current_motor_w
        self.desired_art5 = self.robot_controller.desired_art5
        self.desired_art6 = self.robot_controller.desired_art6
        self.position_update_count = self.robot_controller.position_update_count

        return updated_positions

    def _record_position_history(self, positions: Dict[str, float]) -> None:
        """
        Record position to history (sampled based on config).

        Records firmware angles directly - FK applies calibration internally
        via direction and theta_offset from DH parameters.

        Args:
            positions: Dictionary with all position data
        """
        if self.position_history is None:
            return

        if self.position_update_count % config.POSITION_HISTORY_SAMPLE_RATE == 0:
            # Record firmware angles directly - FK will apply calibration
            self.position_history.add_snapshot(
                art1=positions['X'],
                art2=positions['Y'],
                art3=positions['Z'],
                art4=positions['U'],
                art5=positions['Art5'],
                art6=positions['Art6']
            )

    def _log_positions(self, positions: Dict[str, float]) -> None:
        """Log position update details (for debugging)."""
        logger.debug(
            f"Position feedback - X:{positions['X']:.2f} Y:{positions['Y']:.2f} "
            f"Z:{positions['Z']:.2f} U:{positions['U']:.2f} V:{positions['V']:.2f} "
            f"W:{positions['W']:.2f}"
        )
        logger.info(
            f"  Art1<-X:{positions['X']:.2f}° | Art2<-Y:{positions['Y']:.2f}° | "
            f"Art3<-Z:{positions['Z']:.2f}° | Art4<-U:{positions['U']:.2f}° | "
            f"Art5(calc):{positions['Art5']:.2f}° | Art6(calc):{positions['Art6']:.2f}°"
        )

    def _update_gui_positions(self, positions: Dict[str, float]) -> None:
        """
        Update GUI labels with position data (throttled).

        Args:
            positions: Dictionary with all position data
        """
        # Log positions (throttled)
        if self.position_update_count % config.LOGGING_INTERVAL_POSITIONS == 0:
            self._log_positions(positions)

        # GUI updates (throttled by time)
        current_time = time.time()
        if current_time - self.last_gui_update_time >= config.GUI_UPDATE_INTERVAL:
            self.last_gui_update_time = current_time

            # Update GUI via callback
            if self.gui_update_callback:
                self.gui_update_callback(positions)

            # Set status to Idle since M114 doesn't provide status
            self.update_state("Idle")

    def should_update_gui(self) -> bool:
        """Check if enough time has passed for a GUI update."""
        current_time = time.time()
        return current_time - self.last_gui_update_time >= config.GUI_UPDATE_INTERVAL

    def get_state_color(self, state: str) -> str:
        """Get colour for a given state."""
        return self.STATE_COLORS.get(state, 'rgb(255, 255, 255)')

    def get_endstop_style(self, style_class: str) -> str:
        """Get style string for endstop display."""
        return self.ENDSTOP_STYLES.get(style_class, self.ENDSTOP_STYLES['unknown'])
