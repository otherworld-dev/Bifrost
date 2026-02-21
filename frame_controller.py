"""
Frame Controller Module
Orchestrates frame management between GUI and core logic.

This module provides:
- Frame selection management
- Teaching workflow coordination
- Integration with IK/FK controllers
- Callbacks for GUI updates (decoupled from Qt widgets)
"""

import logging
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from coordinate_frames import (
    CoordinateFrame,
    FrameManager,
    FrameType,
)
from frame_teaching import FrameTeacher, TeachingProgress, TeachingState

logger = logging.getLogger(__name__)


class FrameController:
    """
    Orchestrates frame management between GUI and core logic.

    Uses callbacks for GUI updates (matches existing controller patterns).
    """

    def __init__(
        self,
        frame_manager: Optional[FrameManager] = None,
        # Callbacks
        on_frame_changed: Optional[Callable[[str], None]] = None,
        on_tool_changed: Optional[Callable[[str], None]] = None,
        on_frames_updated: Optional[Callable[[List[str]], None]] = None,
        on_tools_updated: Optional[Callable[[List[str]], None]] = None,
        on_workpieces_updated: Optional[Callable[[List[str]], None]] = None,
        on_teaching_progress: Optional[Callable[[TeachingProgress], None]] = None,
        on_base_frame_changed: Optional[Callable[[], None]] = None,
        get_current_tcp: Optional[Callable[[], np.ndarray]] = None,
    ):
        """
        Initialize frame controller.

        Args:
            frame_manager: FrameManager instance (created if None)
            on_frame_changed: Callback(frame_name) when active frame changes
            on_tool_changed: Callback(tool_name) when active tool changes
            on_frames_updated: Callback(frame_list) when frame list changes
            on_tools_updated: Callback(tool_list) when tool list changes
            on_workpieces_updated: Callback(workpiece_list) when workpiece list changes
            on_teaching_progress: Callback(progress) during frame teaching
            on_base_frame_changed: Callback() when base frame is updated
            get_current_tcp: Callback() -> (x, y, z) to get current TCP position
        """
        self.frame_manager = frame_manager or FrameManager()
        self.frame_teacher = FrameTeacher(self.frame_manager)

        # Callbacks
        self.on_frame_changed = on_frame_changed
        self.on_tool_changed = on_tool_changed
        self.on_frames_updated = on_frames_updated
        self.on_tools_updated = on_tools_updated
        self.on_workpieces_updated = on_workpieces_updated
        self.on_teaching_progress = on_teaching_progress
        self.on_base_frame_changed = on_base_frame_changed
        self.get_current_tcp = get_current_tcp

        # Config file path
        self._config_path: Optional[Path] = None

    def set_config_path(self, path: Path) -> None:
        """Set path for frame configuration file."""
        self._config_path = Path(path)

    # ========== Frame Selection ==========

    def select_frame(self, frame_name: str) -> bool:
        """
        Set the active working frame.

        Args:
            frame_name: Frame to make active

        Returns:
            True if successful
        """
        try:
            self.frame_manager.set_active_frame(frame_name)
            if self.on_frame_changed:
                self.on_frame_changed(frame_name)
            logger.info(f"Active frame changed to: {frame_name}")
            return True
        except KeyError as e:
            logger.error(f"Failed to select frame: {e}")
            return False

    def select_tool(self, tool_name: str) -> bool:
        """
        Set the active tool.

        Args:
            tool_name: Tool frame to make active

        Returns:
            True if successful
        """
        try:
            self.frame_manager.set_active_tool(tool_name)
            if self.on_tool_changed:
                self.on_tool_changed(tool_name)
            logger.info(f"Active tool changed to: {tool_name}")
            return True
        except (KeyError, ValueError) as e:
            logger.error(f"Failed to select tool: {e}")
            return False

    def get_active_frame(self) -> str:
        """Get current active frame name."""
        return self.frame_manager.get_active_frame()

    def get_active_tool(self) -> str:
        """Get current active tool name."""
        return self.frame_manager.get_active_tool()

    # ========== Frame Listing ==========

    def get_all_frames(self) -> List[str]:
        """Get list of all frame names."""
        return self.frame_manager.list_frames()

    def get_selectable_frames(self) -> List[str]:
        """
        Get list of frames that can be selected as active frame.

        Includes: base, tcp, all workpiece frames
        """
        frames = ["base", "tcp"]
        frames.extend(self.frame_manager.list_frames(FrameType.WORKPIECE))
        return frames

    def get_tools(self) -> List[str]:
        """Get list of tool frame names."""
        return self.frame_manager.list_frames(FrameType.TOOL)

    def get_workpieces(self) -> List[str]:
        """Get list of workpiece frame names."""
        return self.frame_manager.list_frames(FrameType.WORKPIECE)

    def get_frame_info(self, name: str) -> Optional[Dict]:
        """
        Get information about a frame.

        Returns:
            Dict with frame details or None if not found
        """
        try:
            frame = self.frame_manager.get_frame(name)
            return {
                "name": frame.name,
                "type": frame.frame_type.value,
                "position": frame.position.tolist(),
                "euler_deg": frame.euler_degrees.tolist(),
                "parent": frame.parent_frame,
                "description": frame.description
            }
        except KeyError:
            return None

    # ========== Frame CRUD ==========

    def create_tool_frame(
        self,
        name: str,
        offset_x: float = 0,
        offset_y: float = 0,
        offset_z: float = 0,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0,
        description: str = ""
    ) -> bool:
        """
        Create a new tool frame.

        Args:
            name: Tool name (must be unique)
            offset_x, offset_y, offset_z: Position offset from TCP (mm)
            roll, pitch, yaw: Orientation offset (degrees)
            description: Optional description

        Returns:
            True if successful
        """
        if self.frame_manager.frame_exists(name):
            logger.error(f"Frame already exists: {name}")
            return False

        try:
            self.frame_manager.create_tool_frame(
                name, offset_x, offset_y, offset_z,
                roll, pitch, yaw, description
            )
            self._notify_tools_updated()
            self._auto_save()
            return True
        except Exception as e:
            logger.error(f"Failed to create tool frame: {e}")
            return False

    def create_workpiece_frame(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0,
        description: str = ""
    ) -> bool:
        """
        Create a new workpiece frame (direct entry, not teaching).

        Args:
            name: Workpiece name (must be unique)
            x, y, z: Position in base frame (mm)
            roll, pitch, yaw: Orientation (degrees)
            description: Optional description

        Returns:
            True if successful
        """
        if self.frame_manager.frame_exists(name):
            logger.error(f"Frame already exists: {name}")
            return False

        try:
            self.frame_manager.create_workpiece_frame(
                name, x, y, z, roll, pitch, yaw, description
            )
            self._notify_workpieces_updated()
            self._auto_save()
            return True
        except Exception as e:
            logger.error(f"Failed to create workpiece frame: {e}")
            return False

    def delete_frame(self, name: str) -> bool:
        """
        Delete a frame.

        Args:
            name: Frame to delete

        Returns:
            True if successful
        """
        try:
            frame = self.frame_manager.get_frame(name)
            frame_type = frame.frame_type

            self.frame_manager.remove_frame(name)

            if frame_type == FrameType.TOOL:
                self._notify_tools_updated()
            elif frame_type == FrameType.WORKPIECE:
                self._notify_workpieces_updated()

            self._auto_save()
            return True
        except (KeyError, ValueError) as e:
            logger.error(f"Failed to delete frame: {e}")
            return False

    def update_base_frame(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0
    ) -> None:
        """
        Update the base frame (robot mounting offset).

        Args:
            x, y, z: Position offset from world origin (mm)
            roll, pitch, yaw: Orientation offset (degrees)
        """
        self.frame_manager.update_base_frame(x, y, z, roll, pitch, yaw)
        self._auto_save()
        if self.on_base_frame_changed:
            self.on_base_frame_changed()
        logger.info(f"Base frame updated: ({x}, {y}, {z}), ({roll}, {pitch}, {yaw})")

    # ========== Frame Teaching ==========

    def start_teaching_workpiece(self, name: str, description: str = "") -> TeachingProgress:
        """
        Start 3-point teaching for a new workpiece frame.

        Args:
            name: Name for the new frame
            description: Optional description

        Returns:
            TeachingProgress with initial state
        """
        progress = self.frame_teacher.start_teaching(name, description)
        if self.on_teaching_progress:
            self.on_teaching_progress(progress)
        return progress

    def record_teaching_point(self) -> TeachingProgress:
        """
        Record current TCP position as the next teaching point.

        Requires get_current_tcp callback to be set.

        Returns:
            Updated TeachingProgress
        """
        if not self.frame_teacher.is_teaching:
            logger.warning("Not in teaching mode")
            return self.frame_teacher.progress

        if not self.get_current_tcp:
            logger.error("No TCP position callback configured")
            return self.frame_teacher.progress

        tcp_pos = self.get_current_tcp()
        progress = self.frame_teacher.record_point(tcp_pos)

        if self.on_teaching_progress:
            self.on_teaching_progress(progress)

        return progress

    def finish_teaching(self) -> Optional[CoordinateFrame]:
        """
        Complete teaching and add frame to manager.

        Returns:
            Created frame if successful, None otherwise
        """
        frame = self.frame_teacher.finish_teaching()

        if frame:
            self._notify_workpieces_updated()
            self._auto_save()

        # Notify teaching complete
        if self.on_teaching_progress:
            self.on_teaching_progress(self.frame_teacher.progress)

        return frame

    def cancel_teaching(self) -> None:
        """Cancel current teaching session."""
        self.frame_teacher.cancel_teaching()
        if self.on_teaching_progress:
            self.on_teaching_progress(self.frame_teacher.progress)

    @property
    def is_teaching(self) -> bool:
        """Check if currently in teaching mode."""
        return self.frame_teacher.is_teaching

    @property
    def teaching_progress(self) -> TeachingProgress:
        """Get current teaching progress."""
        return self.frame_teacher.progress

    # ========== TCP Management ==========

    def update_tcp_transform(self, tcp_transform: np.ndarray) -> None:
        """
        Update TCP transform from FK computation.

        Should be called whenever joint positions change.

        Args:
            tcp_transform: 4x4 TCP pose in base frame
        """
        self.frame_manager.update_tcp_transform(tcp_transform)

    # ========== Persistence ==========

    def load_frames(self, filepath: Optional[Path] = None) -> bool:
        """
        Load frame definitions from file.

        Args:
            filepath: Path to load from (uses _config_path if None)

        Returns:
            True if successful
        """
        path = filepath or self._config_path
        if not path:
            logger.warning("No config path set for frame loading")
            return False

        try:
            self.frame_manager.load_from_file(path)
            self._notify_all_updated()
            logger.info(f"Loaded frames from: {path}")
            return True
        except FileNotFoundError:
            logger.info(f"Frame config not found (will create on save): {path}")
            return False
        except Exception as e:
            logger.error(f"Failed to load frames: {e}")
            return False

    def save_frames(self, filepath: Optional[Path] = None) -> bool:
        """
        Save frame definitions to file.

        Args:
            filepath: Path to save to (uses _config_path if None)

        Returns:
            True if successful
        """
        path = filepath or self._config_path
        if not path:
            logger.warning("No config path set for frame saving")
            return False

        try:
            self.frame_manager.save_to_file(path)
            logger.info(f"Saved frames to: {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save frames: {e}")
            return False

    def _auto_save(self) -> None:
        """Auto-save if config path is set."""
        if self._config_path:
            self.save_frames()

    # ========== Notification Helpers ==========

    def _notify_all_updated(self) -> None:
        """Notify all frame lists updated."""
        self._notify_frames_updated()
        self._notify_tools_updated()
        self._notify_workpieces_updated()

    def _notify_frames_updated(self) -> None:
        """Notify that selectable frames list changed."""
        if self.on_frames_updated:
            self.on_frames_updated(self.get_selectable_frames())

    def _notify_tools_updated(self) -> None:
        """Notify that tools list changed."""
        if self.on_tools_updated:
            self.on_tools_updated(self.get_tools())

    def _notify_workpieces_updated(self) -> None:
        """Notify that workpieces list changed."""
        if self.on_workpieces_updated:
            self.on_workpieces_updated(self.get_workpieces())


if __name__ == "__main__":
    # Test the module
    logging.basicConfig(level=logging.DEBUG)

    print("Frame Controller Module Test")
    print("=" * 50)

    # Create controller with callbacks
    def on_frame_changed(name):
        print(f"  [Callback] Frame changed: {name}")

    def on_tools_updated(tools):
        print(f"  [Callback] Tools updated: {tools}")

    def get_tcp():
        return np.array([200, 100, 150])

    controller = FrameController(
        on_frame_changed=on_frame_changed,
        on_tools_updated=on_tools_updated,
        get_current_tcp=get_tcp
    )

    print("\nSelectable frames:", controller.get_selectable_frames())
    print("Tools:", controller.get_tools())

    # Create a tool
    print("\nCreating tool 'vacuum'...")
    controller.create_tool_frame("vacuum", offset_z=100, description="Vacuum gripper")

    # Select the tool
    print("\nSelecting tool...")
    controller.select_tool("vacuum")
    print(f"Active tool: {controller.get_active_tool()}")

    # Test teaching workflow
    print("\n" + "=" * 50)
    print("Testing frame teaching...")

    progress = controller.start_teaching_workpiece("test_fixture")
    print(f"  State: {progress.state.value}, Message: {progress.message}")

    progress = controller.record_teaching_point()
    print(f"  Point 1 recorded, State: {progress.state.value}")

    # Simulate moving TCP
    controller.get_current_tcp = lambda: np.array([300, 100, 150])
    progress = controller.record_teaching_point()
    print(f"  Point 2 recorded, State: {progress.state.value}")

    controller.get_current_tcp = lambda: np.array([200, 200, 150])
    progress = controller.record_teaching_point()
    print(f"  Point 3 recorded, State: {progress.state.value}")

    frame = controller.finish_teaching()
    if frame:
        print(f"\nCreated frame: {frame.name}")
        print(f"  Position: {frame.position}")
        print(f"  Orientation (deg): {frame.euler_degrees}")

    print("\nWorkpiece frames:", controller.get_workpieces())
    print("Selectable frames:", controller.get_selectable_frames())

    print("\nAll tests completed!")
