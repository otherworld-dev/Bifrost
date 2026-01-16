"""
Coordinate Frame Management for Thor Robot Arm

Provides multi-frame support including:
- World frame (absolute reference)
- Base frame (robot mounting position/orientation)
- Tool frames (multiple end-effector offsets)
- Workpiece frames (user-defined fixtures/objects)

Uses 4x4 homogeneous matrices internally for efficient transformation chaining.
Stores frames as position + quaternion in JSON for compactness.
"""

import json
import logging
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


class FrameType(Enum):
    """Types of coordinate frames supported"""
    WORLD = "world"
    BASE = "base"
    TOOL = "tool"
    WORKPIECE = "workpiece"


@dataclass
class CoordinateFrame:
    """
    Represents a coordinate frame with position and orientation.

    Attributes:
        name: Unique identifier for the frame
        frame_type: Category of frame (WORLD, BASE, TOOL, WORKPIECE)
        transform: 4x4 homogeneous transformation matrix (frame relative to parent)
        parent_frame: Name of parent frame in hierarchy
        description: Optional human-readable description
    """
    name: str
    frame_type: FrameType
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))
    parent_frame: str = "world"
    description: str = ""

    def __post_init__(self):
        """Ensure transform is proper 4x4 matrix"""
        self.transform = np.asarray(self.transform, dtype=np.float64)
        if self.transform.shape != (4, 4):
            raise ValueError(f"Transform must be 4x4 matrix, got {self.transform.shape}")

    @property
    def position(self) -> np.ndarray:
        """Get position (translation) component as (x, y, z)"""
        return self.transform[:3, 3].copy()

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Get rotation component as 3x3 matrix"""
        return self.transform[:3, :3].copy()

    @property
    def quaternion(self) -> np.ndarray:
        """Get orientation as quaternion (w, x, y, z)"""
        rot = Rotation.from_matrix(self.rotation_matrix)
        # scipy uses (x, y, z, w), we want (w, x, y, z)
        q = rot.as_quat()
        return np.array([q[3], q[0], q[1], q[2]])

    @property
    def euler_degrees(self) -> np.ndarray:
        """Get orientation as Euler angles (roll, pitch, yaw) in degrees"""
        rot = Rotation.from_matrix(self.rotation_matrix)
        # ZYX extrinsic = XYZ intrinsic
        rpy = rot.as_euler('ZYX', degrees=True)
        return np.array([rpy[2], rpy[1], rpy[0]])  # roll, pitch, yaw

    def to_dict(self) -> dict:
        """Serialize to dictionary for JSON storage"""
        return {
            "name": self.name,
            "frame_type": self.frame_type.value,
            "position": self.position.tolist(),
            "orientation_quaternion": self.quaternion.tolist(),
            "parent_frame": self.parent_frame,
            "description": self.description
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'CoordinateFrame':
        """Create from dictionary (JSON deserialization)"""
        # Build transform from position + quaternion
        position = np.array(data["position"])
        quat = np.array(data["orientation_quaternion"])  # (w, x, y, z)

        transform = transform_from_position_quaternion(position, quat)

        return cls(
            name=data["name"],
            frame_type=FrameType(data["frame_type"]),
            transform=transform,
            parent_frame=data.get("parent_frame", "world"),
            description=data.get("description", "")
        )

    @classmethod
    def from_position_orientation(
        cls,
        name: str,
        frame_type: FrameType,
        position: np.ndarray,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0,
        parent_frame: str = "world",
        description: str = ""
    ) -> 'CoordinateFrame':
        """
        Create frame from position and Euler angles.

        Args:
            name: Frame name
            frame_type: Type of frame
            position: (x, y, z) in mm
            roll, pitch, yaw: Euler angles in degrees (ZYX convention)
            parent_frame: Parent frame name
            description: Optional description
        """
        transform = transform_from_xyz_rpy(
            position[0], position[1], position[2],
            roll, pitch, yaw
        )
        return cls(
            name=name,
            frame_type=frame_type,
            transform=transform,
            parent_frame=parent_frame,
            description=description
        )


def transform_from_position_quaternion(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """
    Build 4x4 transform from position and quaternion.

    Args:
        position: (x, y, z) translation
        quaternion: (w, x, y, z) orientation

    Returns:
        4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = position

    # Convert (w, x, y, z) to scipy's (x, y, z, w)
    q_scipy = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    T[:3, :3] = Rotation.from_quat(q_scipy).as_matrix()

    return T


def transform_from_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build 4x4 transform from position and Euler angles.

    Args:
        x, y, z: Position in mm
        roll, pitch, yaw: Euler angles in degrees (ZYX extrinsic convention)

    Returns:
        4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = [x, y, z]

    # ZYX extrinsic rotation (yaw around Z, then pitch around Y, then roll around X)
    rot = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True)
    T[:3, :3] = rot.as_matrix()

    return T


def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract Euler angles (roll, pitch, yaw) from rotation matrix.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Tuple (roll, pitch, yaw) in radians
    """
    rot = Rotation.from_matrix(R)
    rpy = rot.as_euler('ZYX')  # Returns [yaw, pitch, roll]
    return rpy[2], rpy[1], rpy[0]  # roll, pitch, yaw


class FrameManager:
    """
    Manages coordinate frames and transformations.

    Responsibilities:
    - Store and retrieve frame definitions
    - Transform points/poses between frames
    - Handle frame hierarchy traversal
    - Persist to/load from JSON

    Frame hierarchy:
        WORLD (identity, fixed)
          └── BASE (robot mounting)
                └── TCP (computed from FK, managed externally)
                      └── TOOL_1, TOOL_2, ... (tool offsets)
                └── WORKPIECE_1, WORKPIECE_2, ... (fixture frames)
    """

    def __init__(self):
        self._frames: Dict[str, CoordinateFrame] = {}
        self._active_frame: str = "base"
        self._active_tool: str = "default_tool"
        self._tcp_transform: np.ndarray = np.eye(4)  # Updated externally from FK
        self._setup_default_frames()

    def _setup_default_frames(self):
        """Initialise with default frame hierarchy"""
        # World frame - always identity, immutable
        self._frames["world"] = CoordinateFrame(
            name="world",
            frame_type=FrameType.WORLD,
            transform=np.eye(4),
            parent_frame="",  # No parent
            description="World origin (fixed)"
        )

        # Base frame - robot mounting position
        self._frames["base"] = CoordinateFrame(
            name="base",
            frame_type=FrameType.BASE,
            transform=np.eye(4),  # Default: coincident with world
            parent_frame="world",
            description="Robot base mounting frame"
        )

        # Default tool - no offset
        self._frames["default_tool"] = CoordinateFrame(
            name="default_tool",
            frame_type=FrameType.TOOL,
            transform=np.eye(4),  # No offset from TCP
            parent_frame="tcp",
            description="Default tool (no offset)"
        )

    # ========== Frame CRUD ==========

    def add_frame(self, frame: CoordinateFrame) -> None:
        """
        Add or update a frame.

        Args:
            frame: CoordinateFrame to add

        Raises:
            ValueError: If trying to modify world frame
        """
        if frame.name == "world":
            raise ValueError("Cannot modify world frame")

        self._frames[frame.name] = frame
        logger.info(f"Added/updated frame: {frame.name} ({frame.frame_type.value})")

    def remove_frame(self, name: str) -> None:
        """
        Remove a frame.

        Args:
            name: Frame name to remove

        Raises:
            ValueError: If trying to remove protected frames
        """
        protected = ["world", "base", "default_tool"]
        if name in protected:
            raise ValueError(f"Cannot remove protected frame: {name}")

        if name not in self._frames:
            raise KeyError(f"Frame not found: {name}")

        del self._frames[name]
        logger.info(f"Removed frame: {name}")

        # Reset active selections if removed
        if self._active_frame == name:
            self._active_frame = "base"
        if self._active_tool == name:
            self._active_tool = "default_tool"

    def get_frame(self, name: str) -> CoordinateFrame:
        """
        Get a frame by name.

        Args:
            name: Frame name

        Returns:
            CoordinateFrame object

        Raises:
            KeyError: If frame not found
        """
        if name not in self._frames:
            raise KeyError(f"Frame not found: {name}")
        return self._frames[name]

    def list_frames(self, frame_type: Optional[FrameType] = None) -> List[str]:
        """
        List all frames, optionally filtered by type.

        Args:
            frame_type: Optional filter by frame type

        Returns:
            List of frame names
        """
        if frame_type is None:
            return list(self._frames.keys())
        return [name for name, frame in self._frames.items() if frame.frame_type == frame_type]

    def frame_exists(self, name: str) -> bool:
        """Check if a frame exists"""
        return name in self._frames

    # ========== Active Frame Management ==========

    def set_active_frame(self, name: str) -> None:
        """
        Set the active working frame.

        Args:
            name: Frame name to make active

        Raises:
            KeyError: If frame not found
        """
        if name not in self._frames and name != "tcp":
            raise KeyError(f"Frame not found: {name}")
        self._active_frame = name
        logger.info(f"Active frame set to: {name}")

    def get_active_frame(self) -> str:
        """Get current active frame name"""
        return self._active_frame

    def set_active_tool(self, name: str) -> None:
        """
        Set the active tool frame.

        Args:
            name: Tool frame name

        Raises:
            KeyError: If tool not found
            ValueError: If frame is not a tool type
        """
        if name not in self._frames:
            raise KeyError(f"Tool frame not found: {name}")
        if self._frames[name].frame_type != FrameType.TOOL:
            raise ValueError(f"Frame '{name}' is not a tool frame")
        self._active_tool = name
        logger.info(f"Active tool set to: {name}")

    def get_active_tool(self) -> str:
        """Get current active tool name"""
        return self._active_tool

    def get_active_tool_offset(self) -> np.ndarray:
        """
        Get 4x4 transform of active tool relative to TCP.

        Returns:
            4x4 homogeneous transform matrix
        """
        tool = self._frames.get(self._active_tool)
        if tool is None:
            return np.eye(4)
        return tool.transform.copy()

    # ========== TCP Management ==========

    def update_tcp_transform(self, tcp_transform: np.ndarray) -> None:
        """
        Update TCP transform from FK computation.

        This should be called whenever joint positions change.

        Args:
            tcp_transform: 4x4 TCP pose in base frame (from FK)
        """
        self._tcp_transform = np.asarray(tcp_transform, dtype=np.float64)

    def get_tcp_transform(self) -> np.ndarray:
        """Get current TCP transform in base frame"""
        return self._tcp_transform.copy()

    def get_tool_tip_transform(self) -> np.ndarray:
        """
        Get current tool tip transform in base frame.

        Combines TCP transform with active tool offset.

        Returns:
            4x4 homogeneous transform of tool tip in base frame
        """
        tool_offset = self.get_active_tool_offset()
        return self._tcp_transform @ tool_offset

    # ========== Transformations ==========

    def _get_transform_to_world(self, frame_name: str) -> np.ndarray:
        """
        Get cumulative transform from frame to world.

        Traverses parent chain and accumulates transforms.

        Args:
            frame_name: Starting frame name

        Returns:
            4x4 transform: frame -> world
        """
        if frame_name == "world":
            return np.eye(4)

        if frame_name == "tcp":
            # TCP is special: base_transform @ tcp_transform
            base_to_world = self._get_transform_to_world("base")
            return base_to_world @ self._tcp_transform

        frame = self._frames.get(frame_name)
        if frame is None:
            raise KeyError(f"Frame not found: {frame_name}")

        # Handle tool frames specially - they're relative to TCP
        if frame.parent_frame == "tcp":
            tcp_to_world = self._get_transform_to_world("tcp")
            return tcp_to_world @ frame.transform

        # Normal parent chain traversal
        parent_to_world = self._get_transform_to_world(frame.parent_frame)
        return parent_to_world @ frame.transform

    def get_transform(self, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Get 4x4 transform from one frame to another.

        Args:
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            4x4 transformation matrix: from_frame -> to_frame

        Example:
            T = get_transform("workpiece1", "base")
            # point_in_base = T @ point_in_workpiece1
        """
        if from_frame == to_frame:
            return np.eye(4)

        # Get both transforms to world
        from_to_world = self._get_transform_to_world(from_frame)
        to_to_world = self._get_transform_to_world(to_frame)

        # from -> world -> to
        # T_from_to = T_world_to^-1 @ T_from_world
        return np.linalg.inv(to_to_world) @ from_to_world

    def transform_point(self, point: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Transform a 3D point between frames.

        Args:
            point: (x, y, z) or (3,) array
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            Transformed point as (3,) array
        """
        point = np.asarray(point, dtype=np.float64)
        if point.shape != (3,):
            raise ValueError(f"Point must be (3,) array, got {point.shape}")

        T = self.get_transform(from_frame, to_frame)

        # Convert to homogeneous, transform, extract
        p_homo = np.array([point[0], point[1], point[2], 1.0])
        p_transformed = T @ p_homo

        return p_transformed[:3]

    def transform_pose(self, pose: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Transform a 4x4 pose matrix between frames.

        Args:
            pose: 4x4 homogeneous transformation matrix
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            Transformed 4x4 pose matrix
        """
        pose = np.asarray(pose, dtype=np.float64)
        if pose.shape != (4, 4):
            raise ValueError(f"Pose must be (4,4) array, got {pose.shape}")

        T = self.get_transform(from_frame, to_frame)
        return T @ pose

    def get_frame_in_frame(self, frame_name: str, reference_frame: str) -> np.ndarray:
        """
        Get a frame's pose expressed in another frame's coordinates.

        Args:
            frame_name: Frame to express
            reference_frame: Frame to express it in

        Returns:
            4x4 pose of frame_name in reference_frame coordinates
        """
        return self.get_transform(frame_name, reference_frame)

    # ========== Persistence ==========

    def save_to_file(self, filepath: Path) -> None:
        """
        Save frame definitions to JSON file.

        Args:
            filepath: Path to save to
        """
        filepath = Path(filepath)

        data = {
            "version": "1.0",
            "description": "Bifrost coordinate frame definitions",
            "base_frame": self._frames["base"].to_dict(),
            "tools": [
                frame.to_dict()
                for frame in self._frames.values()
                if frame.frame_type == FrameType.TOOL
            ],
            "workpieces": [
                frame.to_dict()
                for frame in self._frames.values()
                if frame.frame_type == FrameType.WORKPIECE
            ],
            "active_frame": self._active_frame,
            "active_tool": self._active_tool
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"Saved frames to: {filepath}")

    def load_from_file(self, filepath: Path) -> None:
        """
        Load frame definitions from JSON file.

        Args:
            filepath: Path to load from

        Raises:
            FileNotFoundError: If file doesn't exist
            json.JSONDecodeError: If file is invalid JSON
        """
        filepath = Path(filepath)

        with open(filepath, 'r') as f:
            data = json.load(f)

        # Reset to defaults first
        self._frames.clear()
        self._setup_default_frames()

        # Load base frame
        if "base_frame" in data:
            base_data = data["base_frame"]
            base_data["name"] = "base"
            base_data["frame_type"] = "base"
            base_data["parent_frame"] = "world"
            self._frames["base"] = CoordinateFrame.from_dict(base_data)

        # Load tools
        for tool_data in data.get("tools", []):
            tool_data["parent_frame"] = "tcp"
            tool_data["frame_type"] = "tool"
            frame = CoordinateFrame.from_dict(tool_data)
            self._frames[frame.name] = frame

        # Load workpieces
        for wp_data in data.get("workpieces", []):
            wp_data["parent_frame"] = "base"
            wp_data["frame_type"] = "workpiece"
            frame = CoordinateFrame.from_dict(wp_data)
            self._frames[frame.name] = frame

        # Restore active selections
        self._active_frame = data.get("active_frame", "base")
        self._active_tool = data.get("active_tool", "default_tool")

        # Validate active selections exist
        if self._active_frame not in self._frames and self._active_frame != "tcp":
            self._active_frame = "base"
        if self._active_tool not in self._frames:
            self._active_tool = "default_tool"

        logger.info(f"Loaded frames from: {filepath}")

    # ========== Convenience Methods ==========

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
    ) -> CoordinateFrame:
        """
        Create a new tool frame with offset from TCP.

        Args:
            name: Tool name
            offset_x, offset_y, offset_z: Position offset from TCP in mm
            roll, pitch, yaw: Orientation offset in degrees
            description: Optional description

        Returns:
            Created CoordinateFrame
        """
        frame = CoordinateFrame.from_position_orientation(
            name=name,
            frame_type=FrameType.TOOL,
            position=np.array([offset_x, offset_y, offset_z]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            parent_frame="tcp",
            description=description
        )
        self.add_frame(frame)
        return frame

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
    ) -> CoordinateFrame:
        """
        Create a new workpiece frame in base coordinates.

        Args:
            name: Workpiece frame name
            x, y, z: Position in base frame (mm)
            roll, pitch, yaw: Orientation in degrees
            description: Optional description

        Returns:
            Created CoordinateFrame
        """
        frame = CoordinateFrame.from_position_orientation(
            name=name,
            frame_type=FrameType.WORKPIECE,
            position=np.array([x, y, z]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            parent_frame="base",
            description=description
        )
        self.add_frame(frame)
        return frame

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
        Update base frame (robot mounting position/orientation).

        Args:
            x, y, z: Position offset from world origin in mm
            roll, pitch, yaw: Orientation offset in degrees
        """
        self._frames["base"] = CoordinateFrame.from_position_orientation(
            name="base",
            frame_type=FrameType.BASE,
            position=np.array([x, y, z]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            parent_frame="world",
            description="Robot base mounting frame"
        )
        logger.info(f"Updated base frame: pos=({x}, {y}, {z}), rpy=({roll}, {pitch}, {yaw})")


# Module-level convenience functions

def pose_from_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Create 4x4 pose matrix from position and Euler angles.

    Args:
        x, y, z: Position in mm
        roll, pitch, yaw: Euler angles in radians (ZYX convention)

    Returns:
        4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    rot = Rotation.from_euler('ZYX', [yaw, pitch, roll])
    T[:3, :3] = rot.as_matrix()
    return T


def pose_to_xyz_rpy(pose: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    """
    Extract position and Euler angles from 4x4 pose matrix.

    Args:
        pose: 4x4 homogeneous transformation matrix

    Returns:
        Tuple (x, y, z, roll, pitch, yaw) with angles in radians
    """
    x, y, z = pose[:3, 3]
    rot = Rotation.from_matrix(pose[:3, :3])
    yaw, pitch, roll = rot.as_euler('ZYX')
    return x, y, z, roll, pitch, yaw


if __name__ == "__main__":
    # Test the module
    logging.basicConfig(level=logging.DEBUG)

    print("Coordinate Frames Module Test")
    print("=" * 50)

    # Create manager
    fm = FrameManager()

    # List default frames
    print("\nDefault frames:")
    for name in fm.list_frames():
        frame = fm.get_frame(name)
        print(f"  {name}: type={frame.frame_type.value}, parent={frame.parent_frame}")

    # Create a tool with offset
    print("\nCreating tool 'gripper' with 50mm Z offset...")
    fm.create_tool_frame("gripper", offset_z=50, description="Parallel gripper")

    # Create a workpiece frame
    print("Creating workpiece 'pallet' at (200, 100, 0) rotated 45 deg...")
    fm.create_workpiece_frame("pallet", 200, 100, 0, yaw=45, description="Parts pallet")

    # Test transformations
    print("\nTransformation tests:")

    # Set a TCP position (simulating FK output)
    tcp_pose = transform_from_xyz_rpy(300, 0, 200, 0, 0, 0)
    fm.update_tcp_transform(tcp_pose)
    print(f"  TCP position: {fm.get_tcp_transform()[:3, 3]}")

    # Get tool tip position
    fm.set_active_tool("gripper")
    tool_tip = fm.get_tool_tip_transform()
    print(f"  Tool tip position (with gripper): {tool_tip[:3, 3]}")

    # Transform a point from workpiece to base
    point_in_pallet = np.array([10, 20, 0])
    point_in_base = fm.transform_point(point_in_pallet, "pallet", "base")
    print(f"  Point (10, 20, 0) in pallet -> base: {point_in_base}")

    # Save and reload
    print("\nPersistence test:")
    test_file = Path("test_frames.json")
    fm.save_to_file(test_file)
    print(f"  Saved to {test_file}")

    fm2 = FrameManager()
    fm2.load_from_file(test_file)
    print(f"  Loaded frames: {fm2.list_frames()}")

    # Cleanup
    test_file.unlink()
    print("  Cleaned up test file")

    print("\nAll tests passed!")
