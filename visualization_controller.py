"""
Visualization Controller Module
Handles 3D robot visualization updates and display options

This module provides:
- Visualization update logic
- Display option management
- DH preview mode handling
- Position history visualization
"""

import logging
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass, field

import config

logger = logging.getLogger(__name__)


@dataclass
class DisplayOptions:
    """Container for visualisation display options"""
    show_robot: bool = True
    show_trajectory: bool = False
    show_base_frame: bool = True
    show_workspace: bool = False
    show_grid: bool = True
    show_labels: bool = False
    auto_rotate: bool = False

    def to_dict(self) -> Dict[str, bool]:
        """Convert to dictionary format"""
        return {
            'show_robot': self.show_robot,
            'show_trajectory': self.show_trajectory,
            'show_base_frame': self.show_base_frame,
            'show_workspace': self.show_workspace,
            'show_grid': self.show_grid,
            'show_labels': self.show_labels,
            'auto_rotate': self.auto_rotate
        }

    @staticmethod
    def compact_view(show_trajectory: bool = False, auto_rotate: bool = False) -> 'DisplayOptions':
        """Create options for compact view (modern GUI)"""
        return DisplayOptions(
            show_robot=True,
            show_trajectory=show_trajectory,
            show_base_frame=True,
            show_workspace=False,
            show_grid=True,
            show_labels=False,
            auto_rotate=auto_rotate
        )


@dataclass
class VisualizationState:
    """Container for visualisation state"""
    dh_preview_mode: bool = False
    update_count: int = 0
    last_position_count: int = 0


class VisualizationController:
    """
    Controls 3D robot visualisation updates.

    This class separates visualisation logic from GUI concerns,
    using callbacks for all widget interactions.
    """

    # Default time window for compact view (seconds)
    DEFAULT_COMPACT_WINDOW = 60

    def __init__(
        self,
        # Position history source
        position_history=None,
        # Callbacks for canvas operations
        update_canvas_callback: Optional[Callable[[Any, int, Dict], None]] = None,
        reset_view_callback: Optional[Callable[[], None]] = None,
        # Callbacks for getting display options
        get_trajectory_enabled: Optional[Callable[[], bool]] = None,
        get_auto_rotate_enabled: Optional[Callable[[], bool]] = None,
        get_time_window: Optional[Callable[[], int]] = None,
        get_display_checkboxes: Optional[Callable[[], Dict[str, bool]]] = None,
        # DH parameter callbacks
        reload_dh_parameters: Optional[Callable[[], None]] = None,
    ):
        """
        Initialize visualization controller.

        Args:
            position_history: Position history object to visualize
            update_canvas_callback: Callback(history, window_size, options) to update canvas
            reset_view_callback: Callback() to reset view to default
            get_trajectory_enabled: Callback() -> bool for trajectory checkbox
            get_auto_rotate_enabled: Callback() -> bool for auto-rotate checkbox
            get_time_window: Callback() -> int for time window value
            get_display_checkboxes: Callback() -> dict for all display checkboxes
            reload_dh_parameters: Callback() to reload DH parameters from file
        """
        self.position_history = position_history
        self.update_canvas_callback = update_canvas_callback
        self.reset_view_callback = reset_view_callback
        self.get_trajectory_enabled = get_trajectory_enabled
        self.get_auto_rotate_enabled = get_auto_rotate_enabled
        self.get_time_window = get_time_window
        self.get_display_checkboxes = get_display_checkboxes
        self.reload_dh_parameters = reload_dh_parameters

        # Internal state
        self._state = VisualizationState()

    @property
    def dh_preview_mode(self) -> bool:
        """Get DH preview mode state"""
        return self._state.dh_preview_mode

    @dh_preview_mode.setter
    def dh_preview_mode(self, value: bool) -> None:
        """Set DH preview mode state"""
        self._state.dh_preview_mode = value

    @property
    def update_count(self) -> int:
        """Get update count"""
        return self._state.update_count

    def get_compact_display_options(self) -> DisplayOptions:
        """
        Get display options for compact view (modern GUI).

        Returns:
            DisplayOptions configured for compact view
        """
        show_trajectory = False
        auto_rotate = False

        if self.get_trajectory_enabled:
            show_trajectory = self.get_trajectory_enabled()
        if self.get_auto_rotate_enabled:
            auto_rotate = self.get_auto_rotate_enabled()

        return DisplayOptions.compact_view(
            show_trajectory=show_trajectory,
            auto_rotate=auto_rotate
        )

    def get_full_display_options(self) -> Dict[str, bool]:
        """
        Get display options from checkboxes (classic GUI).

        Returns:
            Dictionary of display options
        """
        if self.get_display_checkboxes:
            return self.get_display_checkboxes()
        return DisplayOptions().to_dict()

    def should_skip_update(self) -> bool:
        """
        Check if update should be skipped.

        Returns:
            True if update should be skipped (e.g., in DH preview mode)
        """
        return self._state.dh_preview_mode

    def update_modern_visualization(self) -> bool:
        """
        Update visualization for modern GUI (compact controls).

        Returns:
            True if update was performed, False if skipped
        """
        if self.should_skip_update():
            logger.debug("Skipping visualization update (DH preview mode)")
            return False

        if not self.update_canvas_callback:
            logger.debug("No canvas callback configured")
            return False

        try:
            options = self.get_compact_display_options()
            window_size = self.DEFAULT_COMPACT_WINDOW

            history_len = len(self.position_history) if self.position_history else 0
            logger.debug(f"Updating 3D visualization: {history_len} positions")

            self.update_canvas_callback(
                self.position_history,
                window_size,
                options.to_dict()
            )

            self._state.update_count += 1
            self._state.last_position_count = history_len
            return True

        except RuntimeError as e:
            # OpenGL context errors can happen during window resize/minimize
            logger.debug(f"OpenGL context error (safe to ignore): {e}")
            return False
        except Exception as e:
            logger.error(f"Error updating 3D visualization: {e}")
            return False

    def update_embedded_visualization(self) -> bool:
        """
        Update visualization for classic GUI (embedded graph with full controls).

        Returns:
            True if update was performed, False if skipped
        """
        if not self.update_canvas_callback:
            return False

        try:
            # Get time window from callback
            window_size = self.DEFAULT_COMPACT_WINDOW
            if self.get_time_window:
                window_size = self.get_time_window()

            # Get display options from checkboxes
            options = self.get_full_display_options()

            self.update_canvas_callback(
                self.position_history,
                window_size,
                options
            )

            self._state.update_count += 1
            return True

        except Exception as e:
            logger.error(f"Error updating embedded visualization: {e}")
            return False

    def reset_view(self) -> bool:
        """
        Reset visualization view to default isometric angle.

        Returns:
            True if reset was performed
        """
        if self.reset_view_callback:
            self.reset_view_callback()
            logger.info("3D view reset to isometric")
            return True
        return False

    def enter_dh_preview_mode(self) -> None:
        """Enter DH preview mode (disables auto-updates)"""
        self._state.dh_preview_mode = True
        logger.debug("Entered DH preview mode")

    def exit_dh_preview_mode(self) -> None:
        """
        Exit DH preview mode and restore saved DH parameters.

        Calls reload_dh_parameters callback to restore saved state.
        """
        if self._state.dh_preview_mode:
            self._state.dh_preview_mode = False

            if self.reload_dh_parameters:
                self.reload_dh_parameters()
                logger.info("Exited DH preview mode, restored saved parameters")
            else:
                logger.info("Exited DH preview mode")

    def on_mode_changed(self, mode_index: int, dh_mode_index: int = 5) -> None:
        """
        Handle mode switching - exit DH preview mode when leaving DH panel.

        Args:
            mode_index: Current mode index
            dh_mode_index: Index of DH params mode (default 5)
        """
        if mode_index != dh_mode_index:
            self.exit_dh_preview_mode()

    def get_update_interval_ms(self) -> int:
        """Get recommended update interval in milliseconds"""
        return config.GRAPH_UPDATE_INTERVAL_MS


def create_visualization_controller(
    position_history=None,
    update_canvas: Callable = None,
    reset_view: Callable = None,
    get_trajectory: Callable = None,
    get_auto_rotate: Callable = None,
    get_time_window: Callable = None,
    get_checkboxes: Callable = None,
    reload_dh: Callable = None
) -> VisualizationController:
    """
    Create a VisualizationController with callbacks.

    Args:
        position_history: Position history object
        update_canvas: Callback(history, window, options) to update canvas
        reset_view: Callback() to reset view
        get_trajectory: Callback() -> bool for trajectory enabled
        get_auto_rotate: Callback() -> bool for auto-rotate enabled
        get_time_window: Callback() -> int for time window
        get_checkboxes: Callback() -> dict for display checkboxes
        reload_dh: Callback() to reload DH parameters

    Returns:
        Configured VisualizationController instance
    """
    return VisualizationController(
        position_history=position_history,
        update_canvas_callback=update_canvas,
        reset_view_callback=reset_view,
        get_trajectory_enabled=get_trajectory,
        get_auto_rotate_enabled=get_auto_rotate,
        get_time_window=get_time_window,
        get_display_checkboxes=get_checkboxes,
        reload_dh_parameters=reload_dh
    )
