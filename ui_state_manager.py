"""
UI State Manager Module
Handles UI state management for enable/disable patterns, visual feedback

This module provides:
- Jog mode visual state management
- Connection state UI updates
- Feed rate enable/disable based on movement type
- Homing button state management
- Centralized style constants
"""

import logging
from typing import Optional, Callable, Dict, List
from dataclasses import dataclass
from enum import Enum

import config

logger = logging.getLogger(__name__)


class ConnectionState(Enum):
    """Connection state enumeration"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"


class RobotState(Enum):
    """Robot operational state"""
    IDLE = "Idle"
    RUNNING = "Run"
    HOLD = "Hold"
    ALARM = "Alarm"
    DISCONNECTED = "Disconnected"


@dataclass
class StyleConfig:
    """Configuration for widget styles"""
    # Jog mode styles
    JOG_MODE_HIGHLIGHT = "rgb(255, 200, 100)"
    JOG_MODE_CHECKBOX_ENABLED = (
        "color: rgb(200, 80, 0); background-color: {highlight}; "
        "padding: 3px; border-radius: 3px; font-weight: bold;"
    )
    JOG_MODE_CHECKBOX_DISABLED = "color: rgb(200, 80, 0); font-weight: bold;"

    # Button disabled style
    BUTTON_DISABLED = "background-color: rgb(200, 200, 200);"
    BUTTON_ENABLED = ""

    # Robot state colors
    STATE_COLORS = {
        RobotState.IDLE: "rgb(0, 255, 0)",      # Green
        RobotState.RUNNING: "rgb(0, 255, 0)",   # Green
        RobotState.HOLD: "rgb(255, 0, 0)",      # Red
        RobotState.ALARM: "rgb(255, 255, 0)",   # Yellow
        RobotState.DISCONNECTED: "rgb(255, 0, 0)",  # Red
    }


class UIStateManager:
    """
    Manages UI state and visual feedback.

    This class coordinates UI state changes using callbacks to apply
    changes to actual widgets, maintaining decoupling from Qt.
    """

    def __init__(
        self,
        # Callbacks for applying state changes
        set_widget_enabled: Optional[Callable[[str, bool], None]] = None,
        set_widget_style: Optional[Callable[[str, str], None]] = None,
        set_widget_visible: Optional[Callable[[str, bool], None]] = None,
        set_widget_text: Optional[Callable[[str, str], None]] = None,
    ):
        """
        Initialize UI state manager.

        Args:
            set_widget_enabled: Callback(widget_name, enabled) to enable/disable
            set_widget_style: Callback(widget_name, style) to set stylesheet
            set_widget_visible: Callback(widget_name, visible) to show/hide
            set_widget_text: Callback(widget_name, text) to set text
        """
        self.set_widget_enabled = set_widget_enabled
        self.set_widget_style = set_widget_style
        self.set_widget_visible = set_widget_visible
        self.set_widget_text = set_widget_text

        # Current state
        self._jog_mode_enabled = False
        self._connection_state = ConnectionState.DISCONNECTED
        self._robot_state = RobotState.DISCONNECTED
        self._is_homing = False

        # Style configuration
        self.styles = StyleConfig()

        # Widget groups for batch operations
        self.fk_go_buttons = [f'FKGoButton{joint}' for joint in
                              ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']]

    @property
    def jog_mode_enabled(self) -> bool:
        """Get current jog mode state"""
        return self._jog_mode_enabled

    @property
    def connection_state(self) -> ConnectionState:
        """Get current connection state"""
        return self._connection_state

    @property
    def robot_state(self) -> RobotState:
        """Get current robot state"""
        return self._robot_state

    @property
    def is_homing(self) -> bool:
        """Get homing state"""
        return self._is_homing

    def update_jog_mode(self, enabled: bool) -> Dict[str, any]:
        """
        Update UI for jog mode state change.

        Args:
            enabled: True to enable jog mode visuals

        Returns:
            Dictionary of state changes applied
        """
        self._jog_mode_enabled = enabled
        changes = {'jog_mode': enabled, 'widgets': []}

        if enabled:
            # Highlight jog mode checkbox
            highlight = getattr(config, 'JOG_MODE_VISUAL_HIGHLIGHT',
                              self.styles.JOG_MODE_HIGHLIGHT)
            checkbox_style = self.styles.JOG_MODE_CHECKBOX_ENABLED.format(
                highlight=highlight
            )
            self._apply_style('JogModeCheckBox', checkbox_style)
            changes['widgets'].append(('JogModeCheckBox', 'style', checkbox_style))

            # Disable all FK Go buttons
            for button in self.fk_go_buttons:
                self._apply_enabled(button, False)
                self._apply_style(button, self.styles.BUTTON_DISABLED)
                changes['widgets'].append((button, 'enabled', False))

            # Disable Go All button
            self._apply_enabled('FKGoAllButton', False)
            self._apply_style('FKGoAllButton', self.styles.BUTTON_DISABLED)
            changes['widgets'].append(('FKGoAllButton', 'enabled', False))

            # Hide ExecuteMovementButton (modern GUI)
            self._apply_visible('ExecuteMovementButton', False)
            changes['widgets'].append(('ExecuteMovementButton', 'visible', False))

            # Disable gripper go button
            self._apply_enabled('GoButtonGripper', False)
            self._apply_style('GoButtonGripper', self.styles.BUTTON_DISABLED)
            changes['widgets'].append(('GoButtonGripper', 'enabled', False))

        else:
            # Restore normal checkbox style
            self._apply_style('JogModeCheckBox', self.styles.JOG_MODE_CHECKBOX_DISABLED)
            changes['widgets'].append(('JogModeCheckBox', 'style', 'normal'))

            # Re-enable all FK Go buttons
            for button in self.fk_go_buttons:
                self._apply_enabled(button, True)
                self._apply_style(button, self.styles.BUTTON_ENABLED)
                changes['widgets'].append((button, 'enabled', True))

            # Re-enable Go All button
            self._apply_enabled('FKGoAllButton', True)
            self._apply_style('FKGoAllButton', self.styles.BUTTON_ENABLED)
            changes['widgets'].append(('FKGoAllButton', 'enabled', True))

            # Show ExecuteMovementButton (modern GUI)
            self._apply_visible('ExecuteMovementButton', True)
            changes['widgets'].append(('ExecuteMovementButton', 'visible', True))

            # Re-enable gripper go button
            self._apply_enabled('GoButtonGripper', True)
            self._apply_style('GoButtonGripper', self.styles.BUTTON_ENABLED)
            changes['widgets'].append(('GoButtonGripper', 'enabled', True))

        logger.debug(f"Jog mode visuals updated: {enabled}")
        return changes

    def update_connection_state(
        self,
        state: ConnectionState,
        port: str = "",
        baudrate: int = 0
    ) -> Dict[str, any]:
        """
        Update UI for connection state change.

        Args:
            state: New connection state
            port: Serial port (for connected state)
            baudrate: Baud rate (for connected state)

        Returns:
            Dictionary of state changes applied
        """
        self._connection_state = state
        changes = {'connection_state': state.value, 'widgets': []}

        if state == ConnectionState.CONNECTED:
            self._apply_text('ConnectButton', 'Disconnect')
            self._apply_enabled('ConnectButton', True)
            changes['widgets'].append(('ConnectButton', 'text', 'Disconnect'))

            # Update robot state to Idle
            self.update_robot_state(RobotState.IDLE)

        elif state == ConnectionState.CONNECTING:
            self._apply_text('ConnectButton', 'Connecting...')
            self._apply_enabled('ConnectButton', False)
            changes['widgets'].append(('ConnectButton', 'text', 'Connecting...'))

        elif state == ConnectionState.DISCONNECTED:
            self._apply_text('ConnectButton', 'Connect')
            self._apply_enabled('ConnectButton', True)
            changes['widgets'].append(('ConnectButton', 'text', 'Connect'))

            # Update robot state to Disconnected
            self.update_robot_state(RobotState.DISCONNECTED)

        logger.debug(f"Connection state updated: {state.value}")
        return changes

    def update_robot_state(self, state: RobotState) -> Dict[str, any]:
        """
        Update UI for robot state change.

        Args:
            state: New robot state

        Returns:
            Dictionary of state changes applied
        """
        self._robot_state = state
        changes = {'robot_state': state.value, 'widgets': []}

        # Get colour for state
        color = self.styles.STATE_COLORS.get(state, "rgb(255, 255, 255)")
        style = f"background-color: {color}"

        self._apply_style('RobotStateDisplay', style)
        self._apply_text('RobotStateDisplay', state.value)
        changes['widgets'].append(('RobotStateDisplay', 'style', style))
        changes['widgets'].append(('RobotStateDisplay', 'text', state.value))

        logger.debug(f"Robot state updated: {state.value}")
        return changes

    def update_homing_state(self, is_homing: bool) -> Dict[str, any]:
        """
        Update UI for homing state.

        Args:
            is_homing: True if homing in progress

        Returns:
            Dictionary of state changes applied
        """
        self._is_homing = is_homing
        changes = {'is_homing': is_homing, 'widgets': []}

        if is_homing:
            self._apply_enabled('HomeButton', False)
            self._apply_text('HomeButton', 'Homing...')
            changes['widgets'].append(('HomeButton', 'enabled', False))
            changes['widgets'].append(('HomeButton', 'text', 'Homing...'))
        else:
            self._apply_enabled('HomeButton', True)
            self._apply_text('HomeButton', 'Home')
            changes['widgets'].append(('HomeButton', 'enabled', True))
            changes['widgets'].append(('HomeButton', 'text', 'Home'))

        logger.debug(f"Homing state updated: {is_homing}")
        return changes

    def update_feed_rate_enabled(self, movement_type: str) -> Dict[str, any]:
        """
        Update feed rate input enabled state based on movement type.

        Args:
            movement_type: "G0" (rapid) or "G1" (linear with feedrate)

        Returns:
            Dictionary of state changes applied
        """
        enabled = movement_type == "G1"
        changes = {'feed_rate_enabled': enabled, 'widgets': []}

        self._apply_enabled('FeedRateLabel', enabled)
        self._apply_enabled('FeedRateInput', enabled)
        changes['widgets'].append(('FeedRateLabel', 'enabled', enabled))
        changes['widgets'].append(('FeedRateInput', 'enabled', enabled))

        logger.debug(f"Feed rate enabled: {enabled} (movement type: {movement_type})")
        return changes

    def get_state_color(self, state_text: str) -> str:
        """
        Get color for a robot state string.

        Args:
            state_text: State text (e.g., "Idle", "Run", "Hold")

        Returns:
            RGB color string
        """
        # Map text to enum
        state_map = {
            "Idle": RobotState.IDLE,
            "Run": RobotState.RUNNING,
            "Hold": RobotState.HOLD,
            "Alarm": RobotState.ALARM,
            "Disconnected": RobotState.DISCONNECTED,
        }

        state = state_map.get(state_text, RobotState.IDLE)
        return self.styles.STATE_COLORS.get(state, "rgb(255, 255, 255)")

    def get_jog_mode_changes(self, enabled: bool) -> List[tuple]:
        """
        Get list of widget changes for jog mode without applying.

        Args:
            enabled: Jog mode state

        Returns:
            List of (widget_name, property, value) tuples
        """
        changes = []

        if enabled:
            highlight = getattr(config, 'JOG_MODE_VISUAL_HIGHLIGHT',
                              self.styles.JOG_MODE_HIGHLIGHT)
            checkbox_style = self.styles.JOG_MODE_CHECKBOX_ENABLED.format(
                highlight=highlight
            )
            changes.append(('JogModeCheckBox', 'style', checkbox_style))

            for button in self.fk_go_buttons:
                changes.append((button, 'enabled', False))
                changes.append((button, 'style', self.styles.BUTTON_DISABLED))

            changes.append(('FKGoAllButton', 'enabled', False))
            changes.append(('FKGoAllButton', 'style', self.styles.BUTTON_DISABLED))
            changes.append(('ExecuteMovementButton', 'visible', False))
            changes.append(('GoButtonGripper', 'enabled', False))
            changes.append(('GoButtonGripper', 'style', self.styles.BUTTON_DISABLED))
        else:
            changes.append(('JogModeCheckBox', 'style', self.styles.JOG_MODE_CHECKBOX_DISABLED))

            for button in self.fk_go_buttons:
                changes.append((button, 'enabled', True))
                changes.append((button, 'style', self.styles.BUTTON_ENABLED))

            changes.append(('FKGoAllButton', 'enabled', True))
            changes.append(('FKGoAllButton', 'style', self.styles.BUTTON_ENABLED))
            changes.append(('ExecuteMovementButton', 'visible', True))
            changes.append(('GoButtonGripper', 'enabled', True))
            changes.append(('GoButtonGripper', 'style', self.styles.BUTTON_ENABLED))

        return changes

    def _apply_enabled(self, widget_name: str, enabled: bool) -> None:
        """Apply enabled state via callback"""
        if self.set_widget_enabled:
            self.set_widget_enabled(widget_name, enabled)

    def _apply_style(self, widget_name: str, style: str) -> None:
        """Apply style via callback"""
        if self.set_widget_style:
            self.set_widget_style(widget_name, style)

    def _apply_visible(self, widget_name: str, visible: bool) -> None:
        """Apply visibility via callback"""
        if self.set_widget_visible:
            self.set_widget_visible(widget_name, visible)

    def _apply_text(self, widget_name: str, text: str) -> None:
        """Apply text via callback"""
        if self.set_widget_text:
            self.set_widget_text(widget_name, text)


def create_ui_state_manager(
    set_enabled: Callable = None,
    set_style: Callable = None,
    set_visible: Callable = None,
    set_text: Callable = None
) -> UIStateManager:
    """
    Create a UIStateManager with callbacks.

    Args:
        set_enabled: Callback(widget_name, enabled)
        set_style: Callback(widget_name, style)
        set_visible: Callback(widget_name, visible)
        set_text: Callback(widget_name, text)

    Returns:
        Configured UIStateManager instance
    """
    return UIStateManager(
        set_widget_enabled=set_enabled,
        set_widget_style=set_style,
        set_widget_visible=set_visible,
        set_widget_text=set_text
    )
