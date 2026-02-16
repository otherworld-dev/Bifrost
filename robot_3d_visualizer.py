"""
3D Robot Arm Visualizer for Thor Robot
Displays robot skeleton and end effector trajectory in interactive 3D view
PYQTGRAPH VERSION: Uses OpenGL for smooth interactive controls and better performance

Realistic visualization using geometric primitives to approximate Thor robot geometry.
"""

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import threading
from pathlib import Path
import json

import forward_kinematics as fk

logger = logging.getLogger(__name__)

# Try to import numpy-stl for STL loading
try:
    from stl import mesh as stl_mesh
    STL_AVAILABLE = True
except ImportError:
    STL_AVAILABLE = False
    logger.warning("numpy-stl not installed. STL visualization disabled. "
                   "Install with: pip install numpy-stl")


# =============================================================================
# STL Mesh Loading for Realistic Robot Visualisation
# =============================================================================

# STL file mapping: joint index -> filename
STL_FILES = {
    'base': 'Thor - base.stl',
    'art1': 'Thor - Art1.stl',
    'art2': 'Thor - Art2.stl',
    'art3': 'Thor - Art3.stl',
    'art4': 'Thor - Art4.stl',
    'art5': 'Thor - Art5.stl',
    'gripper': 'Thor - Gripper.stl',
}

# Cache for loaded STL meshes
_stl_cache = {}


def get_stl_directory():
    """Get the STL files directory path"""
    return Path(__file__).parent / 'STLs'


def load_stl_mesh(name):
    """
    Load an STL file and convert to PyQtGraph mesh format.

    Args:
        name: Key from STL_FILES dict ('base', 'art1', etc.)

    Returns:
        Tuple (vertices, faces) or None if loading fails.
        vertices: Nx3 numpy array of vertex positions
        faces: Mx3 numpy array of face indices
    """
    global _stl_cache

    if not STL_AVAILABLE:
        return None

    if name in _stl_cache:
        return _stl_cache[name]

    if name not in STL_FILES:
        logger.warning(f"Unknown STL name: {name}")
        return None

    stl_path = get_stl_directory() / STL_FILES[name]

    if not stl_path.exists():
        logger.warning(f"STL file not found: {stl_path}")
        return None

    try:
        # Load STL using numpy-stl
        mesh_data = stl_mesh.Mesh.from_file(str(stl_path))

        # Extract vertices - STL stores 3 vertices per face
        # mesh_data.vectors has shape (n_faces, 3, 3) - 3 vertices per face, 3 coords per vertex
        n_faces = len(mesh_data.vectors)

        # Flatten to get all vertices (with duplicates)
        all_vertices = mesh_data.vectors.reshape(-1, 3)

        # Create unique vertices and face indices
        # For efficiency with large meshes, we deduplicate vertices
        unique_vertices, inverse_indices = np.unique(
            all_vertices, axis=0, return_inverse=True
        )

        # Reshape inverse indices to get face indices
        faces = inverse_indices.reshape(n_faces, 3)

        # Convert to float32 for OpenGL
        vertices = unique_vertices.astype(np.float32)
        faces = faces.astype(np.uint32)

        logger.info(f"Loaded STL '{name}': {len(vertices)} vertices, {len(faces)} faces")

        # Cache the result
        _stl_cache[name] = (vertices, faces)
        return vertices, faces

    except Exception as e:
        logger.error(f"Error loading STL '{name}': {e}")
        return None


def load_all_stl_meshes():
    """Pre-load all STL meshes into cache"""
    if not STL_AVAILABLE:
        logger.warning("STL loading not available")
        return False

    success = True
    for name in STL_FILES.keys():
        result = load_stl_mesh(name)
        if result is None:
            success = False

    return success


def get_stl_calibration():
    """
    Load STL calibration offsets from config file.

    Returns:
        Dict mapping STL name to calibration dict with:
        - offset: [x, y, z] translation offset in mm
        - rotation: [rx, ry, rz] rotation in degrees (applied as Euler XYZ)
        - scale: float scale factor (default 1.0)
    """
    config_path = get_stl_directory() / 'stl_calibration.json'

    # Default calibration (identity - no offset)
    default_calibration = {
        'base': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'art1': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'art2': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'art3': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'art4': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'art5': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
        'gripper': {'offset': [0, 0, 0], 'rotation': [0, 0, 0], 'scale': 1.0},
    }

    if config_path.exists():
        try:
            with open(config_path, 'r') as f:
                loaded = json.load(f)
            # Merge with defaults
            for key in default_calibration:
                if key in loaded:
                    default_calibration[key].update(loaded[key])
            logger.info("Loaded STL calibration from config")
        except Exception as e:
            logger.warning(f"Error loading STL calibration: {e}")

    return default_calibration


def euler_to_rotation_matrix(rx, ry, rz):
    """
    Create rotation matrix from Euler angles (XYZ order, degrees).

    Args:
        rx, ry, rz: Rotation angles in degrees

    Returns:
        3x3 rotation matrix
    """
    rx, ry, rz = np.radians([rx, ry, rz])

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    # Rotation matrices
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    # Combined rotation (XYZ order)
    return Rz @ Ry @ Rx


def transform_stl_vertices(vertices, transform_4x4, calibration=None):
    """
    Apply transformation to STL vertices.

    Args:
        vertices: Nx3 array of vertex positions
        transform_4x4: 4x4 homogeneous transformation matrix (from FK)
        calibration: Optional dict with 'offset', 'rotation', 'scale'

    Returns:
        Transformed Nx3 vertex array
    """
    verts = vertices.copy()

    # Apply calibration first (in original STL local frame)
    if calibration:
        # Scale first
        scale = calibration.get('scale', 1.0)
        verts = verts * scale

        # Offset in original STL coordinates (before rotation)
        offset = calibration.get('offset', [0, 0, 0])
        verts = verts + np.array(offset)

        # Then rotate to align with robot frame
        rot = calibration.get('rotation', [0, 0, 0])
        if any(r != 0 for r in rot):
            R_local = euler_to_rotation_matrix(*rot)
            verts = verts @ R_local.T

    # Apply FK transformation
    R = transform_4x4[:3, :3]
    t = transform_4x4[:3, 3]

    verts = verts @ R.T + t

    return verts.astype(np.float32)


# =============================================================================
# Mesh Generation Helpers for Realistic Robot Visualisation
# =============================================================================

def create_cylinder_mesh(radius, height, segments=16):
    """
    Create a cylinder mesh centered at origin, extending along Z-axis.

    Args:
        radius: Cylinder radius
        height: Cylinder height (extends from -height/2 to +height/2 on Z)
        segments: Number of segments around circumference

    Returns:
        verts: Nx3 array of vertices
        faces: Mx3 array of face indices
    """
    theta = np.linspace(0, 2*np.pi, segments, endpoint=False)

    # Bottom and top circle vertices
    bottom_verts = np.zeros((segments, 3))
    bottom_verts[:, 0] = radius * np.cos(theta)
    bottom_verts[:, 1] = radius * np.sin(theta)
    bottom_verts[:, 2] = -height / 2

    top_verts = np.zeros((segments, 3))
    top_verts[:, 0] = radius * np.cos(theta)
    top_verts[:, 1] = radius * np.sin(theta)
    top_verts[:, 2] = height / 2

    # Centre points for caps
    bottom_center = np.array([[0, 0, -height/2]])
    top_center = np.array([[0, 0, height/2]])

    # Combine vertices: bottom ring, top ring, bottom centre, top centre
    verts = np.vstack([bottom_verts, top_verts, bottom_center, top_center])

    faces = []
    bottom_center_idx = 2 * segments
    top_center_idx = 2 * segments + 1

    for i in range(segments):
        next_i = (i + 1) % segments

        # Side faces (two triangles per quad)
        # Bottom-left, bottom-right, top-right
        faces.append([i, next_i, segments + next_i])
        # Bottom-left, top-right, top-left
        faces.append([i, segments + next_i, segments + i])

        # Bottom cap
        faces.append([bottom_center_idx, next_i, i])

        # Top cap
        faces.append([top_center_idx, segments + i, segments + next_i])

    return np.array(verts, dtype=np.float32), np.array(faces, dtype=np.uint32)


def create_box_mesh(width, depth, height):
    """
    Create a box mesh centered at origin.

    Args:
        width: Size along X-axis
        depth: Size along Y-axis
        height: Size along Z-axis

    Returns:
        verts: 8x3 array of vertices
        faces: 12x3 array of face indices (triangles)
    """
    w, d, h = width/2, depth/2, height/2

    verts = np.array([
        [-w, -d, -h],  # 0: bottom-back-left
        [ w, -d, -h],  # 1: bottom-back-right
        [ w,  d, -h],  # 2: bottom-front-right
        [-w,  d, -h],  # 3: bottom-front-left
        [-w, -d,  h],  # 4: top-back-left
        [ w, -d,  h],  # 5: top-back-right
        [ w,  d,  h],  # 6: top-front-right
        [-w,  d,  h],  # 7: top-front-left
    ], dtype=np.float32)

    faces = np.array([
        # Bottom
        [0, 2, 1], [0, 3, 2],
        # Top
        [4, 5, 6], [4, 6, 7],
        # Front
        [3, 7, 6], [3, 6, 2],
        # Back
        [0, 1, 5], [0, 5, 4],
        # Left
        [0, 4, 7], [0, 7, 3],
        # Right
        [1, 2, 6], [1, 6, 5],
    ], dtype=np.uint32)

    return verts, faces


def create_dome_mesh(radius, segments=16, rings=8):
    """
    Create a hemisphere/dome mesh (top half of sphere).

    Args:
        radius: Dome radius
        segments: Number of segments around circumference
        rings: Number of rings from base to top

    Returns:
        verts: Nx3 array of vertices
        faces: Mx3 array of face indices
    """
    verts = []

    # Generate vertices ring by ring from bottom to top
    for i in range(rings + 1):
        phi = (np.pi / 2) * (i / rings)  # 0 to pi/2
        z = radius * np.sin(phi)
        ring_radius = radius * np.cos(phi)

        for j in range(segments):
            theta = 2 * np.pi * j / segments
            x = ring_radius * np.cos(theta)
            y = ring_radius * np.sin(theta)
            verts.append([x, y, z])

    verts = np.array(verts, dtype=np.float32)

    faces = []
    for i in range(rings):
        for j in range(segments):
            next_j = (j + 1) % segments

            # Current ring indices
            curr = i * segments + j
            curr_next = i * segments + next_j

            # Next ring indices
            upper = (i + 1) * segments + j
            upper_next = (i + 1) * segments + next_j

            # Two triangles per quad (except at the pole)
            if i < rings - 1:
                faces.append([curr, curr_next, upper_next])
                faces.append([curr, upper_next, upper])
            else:
                # At the top, triangles converge
                faces.append([curr, curr_next, upper])

    return verts, np.array(faces, dtype=np.uint32)


def transform_mesh(verts, position, rotation_matrix=None):
    """
    Transform mesh vertices by rotation and translation.

    Args:
        verts: Nx3 array of vertices
        position: [x, y, z] translation
        rotation_matrix: 3x3 rotation matrix (optional)

    Returns:
        Transformed vertices
    """
    transformed = verts.copy()

    if rotation_matrix is not None:
        transformed = transformed @ rotation_matrix.T

    transformed += np.array(position)

    return transformed


def rotation_matrix_from_vectors(vec1, vec2):
    """
    Create rotation matrix that rotates vec1 to vec2.

    Args:
        vec1: Source vector (will be normalized)
        vec2: Target vector (will be normalized)

    Returns:
        3x3 rotation matrix
    """
    a = np.array(vec1, dtype=np.float64)
    b = np.array(vec2, dtype=np.float64)

    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)

    if a_norm < 1e-10 or b_norm < 1e-10:
        return np.eye(3)

    a = a / a_norm
    b = b / b_norm

    # Check if vectors are parallel
    dot = np.dot(a, b)
    if dot > 0.9999:
        return np.eye(3)
    if dot < -0.9999:
        # 180 degree rotation - find perpendicular axis
        perp = np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0])
        perp = perp - np.dot(perp, a) * a
        perp = perp / np.linalg.norm(perp)
        return 2 * np.outer(perp, perp) - np.eye(3)

    # Rodrigues' rotation formula
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = dot

    vx = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

    R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s + 1e-10))

    return R


class Robot3DCanvas(gl.GLViewWidget):
    """
    PyQtGraph OpenGL canvas for visualizing robot arm and trajectory
    Provides smooth mouse controls and better visual clarity
    """

    def __init__(self, parent=None, width=800, height=700, dpi=100):
        """
        Initialize 3D robot visualization canvas with OpenGL rendering

        Args:
            parent: Parent Qt widget
            width: Widget width (if < 100, treated as inches and multiplied by dpi; otherwise pixels)
            height: Widget height (if < 100, treated as inches and multiplied by dpi; otherwise pixels)
            dpi: Dots per inch for converting inches to pixels (default: 100)
        """
        super().__init__(parent)
        self.setParent(parent)

        # Convert inches to pixels if needed (for matplotlib API compatibility)
        # If width/height < 100, assume it's in inches (like matplotlib figsize)
        if width < 100:
            width_px = int(width * dpi)
        else:
            width_px = int(width)

        if height < 100:
            height_px = int(height * dpi)
        else:
            height_px = int(height)

        # Set widget size
        self.resize(width_px, height_px)

        # Visualisation state
        self.current_joint_positions = None
        self.trajectory_points = []
        self.trajectory_timestamps = []

        # Display options
        self.show_robot = True
        self.show_trajectory = True
        self.show_base_frame = True
        self.show_workspace = False
        self.show_grid = True
        self.show_labels = False
        self.auto_rotate = False
        self.rotation_angle = 45  # Current azimuth for auto-rotate

        # STL visualisation options
        self.use_stl = STL_AVAILABLE  # Use STL meshes if available
        self.stl_loaded = False
        self.stl_calibration = get_stl_calibration()

        # Colour schemes (RGBA format for PyQtGraph)
        # Thor robot colours - orange body with contrasting accents
        self.colors_active = {
            'body': (1.0, 0.55, 0.2, 1.0),       # Thor orange
            'body_dark': (0.95, 0.48, 0.12, 1.0),  # Darker orange
            'joint': (0.47, 0.47, 0.47, 1.0),   # Medium grey for joint rings
            'accent': (0.38, 0.38, 0.38, 1.0),  # Dark grey accents
            'tcp': (1.0, 0.25, 0.25, 1.0),      # Red TCP
            'gripper': (0.57, 0.57, 0.57, 1.0), # Medium grey gripper
            # Legacy keys for compatibility
            'link': (1.0, 0.55, 0.2, 1.0),
            'base': (0.42, 0.42, 0.42, 1.0)
        }
        self.colors_inactive = {
            'body': (0.85, 0.55, 0.35, 0.8),    # Muted orange
            'body_dark': (0.78, 0.48, 0.28, 0.8),
            'joint': (0.5, 0.5, 0.5, 0.7),
            'accent': (0.4, 0.4, 0.4, 0.7),
            'tcp': (0.75, 0.45, 0.45, 0.7),
            'gripper': (0.6, 0.6, 0.6, 0.7),
            # Legacy keys
            'link': (0.85, 0.55, 0.35, 0.8),
            'base': (0.45, 0.45, 0.45, 0.7)
        }

        # PERFORMANCE OPTIMIZATION: Dirty flag pattern
        self._is_dirty = True  # Needs redraw
        self._last_joint_angles = None  # Cache last angles to detect changes
        self._last_trajectory_length = 0  # Cache trajectory length
        self._render_lock = threading.Lock()  # Thread-safe rendering
        self._pending_update = False  # Debounce rapid updates

        # FK calculation cache (avoid redundant calculations)
        self._fk_cache = {}
        self._fk_cache_lock = threading.Lock()

        # PERFORMANCE OPTIMIZATION: Cache workspace envelope (only changes with DH params)
        self._workspace_envelope_cache = None

        # PERFORMANCE OPTIMIZATION: Persistent mesh items (reuse instead of recreate)
        self._persistent_mesh_items = {}  # name -> GLMeshItem
        self._persistent_meshes_initialized = False
        self._grid_initialized = False
        self._tcp_mesh_item = None  # Separate TCP indicator
        self._base_frame_initialized = False  # Base frame is static

        # Force reload DH parameters at startup to ensure latest values
        fk.reload_dh_parameters()
        logger.info("DH parameters loaded at visualizer startup")

        # Graphics items (will be created/updated as needed)
        self.robot_arm_item = None
        self.robot_joints_item = None
        self.tcp_item = None
        self.base_item = None
        self.trajectory_item = None
        self.grid_item = None
        self.grid_label_items = []  # Coordinate labels on grid
        self.base_frame_items = []
        self.tcp_frame_items = []
        self.workspace_item = None
        self.tool_direction_item = None
        self.base_front_item = None
        self.robot_mesh_items = []  # List of 3D mesh items for realistic robot

        # Setup initial view
        self.setup_3d_view()

        # Pre-load STL meshes if available
        if self.use_stl:
            self.stl_loaded = load_all_stl_meshes()
            if self.stl_loaded:
                logger.info("STL meshes loaded successfully")
            else:
                logger.warning("Some STL meshes failed to load, falling back to primitives")
                self.use_stl = False

        # Show home position immediately
        self.show_home_position()

        logger.info("3D robot canvas initialised with PyQtGraph OpenGL rendering")

    def setup_3d_view(self):
        """Setup 3D view with camera position, grid, and axes"""
        # Get workspace envelope for setting limits
        workspace = fk.compute_workspace_envelope()
        max_reach = workspace['radius']
        z_max = workspace['z_max']

        # Set camera distance to see whole workspace
        # PyQtGraph uses distance from centre
        distance = max_reach * 2.5

        # Set camera centre point above the base (not at origin)
        # This shifts the view focus up so the robot isn't centred vertically
        center_z = z_max * 0.35  # Focus at ~35% of max height
        self.camera_center = pg.Vector(0, 0, center_z)

        # Set camera position (isometric view)
        # elevation=30°, azimuth=45°, centered above base
        self.setCameraPosition(distance=distance, elevation=30, azimuth=45)
        self.opts['center'] = self.camera_center

        # Add grid at Z=0 (base plane)
        if self.show_grid:
            if self.grid_item:
                self.removeItem(self.grid_item)

            # Clear old grid labels
            for item in self.grid_label_items:
                if item is not None:
                    try:
                        self.removeItem(item)
                    except Exception:
                        pass
            self.grid_label_items = []

            # Create grid
            grid_size = max_reach * 2.5
            self.grid_item = gl.GLGridItem()
            self.grid_item.setSize(grid_size, grid_size)
            self.grid_item.setSpacing(50, 50)  # 50mm grid spacing
            self.grid_item.setColor(pg.mkColor(50, 50, 50, 200))  # Dark grey grid
            self.grid_item.translate(0, 0, 0)  # At Z=0
            self.addItem(self.grid_item)

            # Add coordinate labels along axes
            label_spacing = 100  # Label every 100mm
            label_range = int(grid_size / 2)

            # X-axis labels (along positive and negative X)
            for x in range(-label_range, label_range + 1, label_spacing):
                if x == 0:
                    continue  # Skip origin
                text_item = gl.GLTextItem(
                    pos=np.array([x, -30, 0]),
                    text=f'{x}',
                    color=pg.mkColor(0, 0, 0, 255),
                    font=QtGui.QFont('Arial', 8)
                )
                self.addItem(text_item)
                self.grid_label_items.append(text_item)

            # Y-axis labels (along positive and negative Y)
            for y in range(-label_range, label_range + 1, label_spacing):
                if y == 0:
                    continue  # Skip origin
                text_item = gl.GLTextItem(
                    pos=np.array([-30, y, 0]),
                    text=f'{y}',
                    color=pg.mkColor(0, 0, 0, 255),
                    font=QtGui.QFont('Arial', 8)
                )
                self.addItem(text_item)
                self.grid_label_items.append(text_item)

            # Origin label
            origin_label = gl.GLTextItem(
                pos=np.array([-40, -40, 0]),
                text='0',
                color=pg.mkColor(0, 0, 0, 255),
                font=QtGui.QFont('Arial', 8)
            )
            self.addItem(origin_label)
            self.grid_label_items.append(origin_label)

            # Axis labels
            x_label = gl.GLTextItem(
                pos=np.array([label_range + 20, 0, 0]),
                text='X+',
                color=pg.mkColor(200, 0, 0, 255),
                font=QtGui.QFont('Arial', 10, QtGui.QFont.Bold)
            )
            self.addItem(x_label)
            self.grid_label_items.append(x_label)

            y_label = gl.GLTextItem(
                pos=np.array([0, label_range + 20, 0]),
                text='Y+',
                color=pg.mkColor(0, 150, 0, 255),
                font=QtGui.QFont('Arial', 10, QtGui.QFont.Bold)
            )
            self.addItem(y_label)
            self.grid_label_items.append(y_label)

        # Set background colour to white
        self.setBackgroundColor('w')

    def show_home_position(self):
        """Display robot at home position (all joints at 0)"""
        # Force reload DH parameters to ensure latest values are used
        fk.reload_dh_parameters()
        # Invalidate all caches since DH parameters may have changed
        self.invalidate_workspace_cache()
        with self._fk_cache_lock:
            self._fk_cache.clear()
        logger.info("DH parameters reloaded and caches cleared")

        # Use persistent grid
        if self.show_grid:
            self._ensure_grid_initialized()

        # Show robot at home position (all joints at 0)
        home_angles = (0, 0, 0, 0, 0, 0)
        home_positions = fk.compute_all_joint_positions(*home_angles)
        self.current_joint_positions = home_positions

        # DEBUG: Log computed positions
        logger.info(f"Home positions computed:")
        for i, pos in enumerate(home_positions):
            logger.info(f"  [{i}]: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")

        # Draw robot using fast path if STL available
        if self.use_stl and self.stl_loaded:
            self._initialize_persistent_meshes()
            self._ensure_meshes_in_scene()
            self._update_robot_transforms(home_angles)
        else:
            self.draw_robot_arm(home_positions, active=False, joint_angles=home_angles)

        # Draw base coordinate frame (XYZ axes at origin)
        self.draw_base_frame(length=100)

        # Draw TCP frame at home position
        self.draw_tcp_frame(0, 0, 0, 0, 0, 0, length=50)

    def preview_dh_parameters(self, dh_params):
        """
        Preview robot with custom DH parameters (for live editing).

        Args:
            dh_params: List of dicts with DH parameters for each link
        """
        # Apply DH parameters to FK module temporarily
        fk.apply_dh_parameters(dh_params)

        # Invalidate all caches since parameters changed
        self.invalidate_workspace_cache()
        with self._fk_cache_lock:
            self._fk_cache.clear()

        # Remove old persistent meshes and force re-creation with new DH transforms
        self._remove_meshes_from_scene()
        self._persistent_meshes_initialized = False
        self._base_frame_initialized = False

        # Re-initialize and update with home position using new DH parameters
        home_angles = (0, 0, 0, 0, 0, 0)
        if self.use_stl and self.stl_loaded:
            self._initialize_persistent_meshes()
            self._ensure_meshes_in_scene()
            self._update_robot_transforms(home_angles)
            self.draw_tcp_frame(*home_angles, length=50)
        else:
            home_positions = fk.compute_all_joint_positions(*home_angles)
            self.current_joint_positions = home_positions
            self._draw_robot_primitives(home_positions, active=False)

        if self.show_base_frame:
            self.draw_base_frame()

        logger.debug("DH parameters preview updated")

    def update_robot(self, joint_angles):
        """
        Update robot visualization directly from joint angles (no position history needed).

        Args:
            joint_angles: List/tuple of 6 joint angles [q1, q2, q3, q4, q5, q6] in degrees
        """
        angles = tuple(joint_angles[:6])

        if self.show_robot and self.use_stl and self.stl_loaded:
            self._initialize_persistent_meshes()
            self._ensure_meshes_in_scene()
            self._update_robot_transforms(angles)
            self.draw_tcp_frame(*angles, length=50)
        elif self.show_robot:
            current_positions = self._compute_fk_cached(*angles)
            self.current_joint_positions = current_positions
            self._draw_robot_primitives(current_positions, active=True)

    def clear_all_items(self):
        """Remove all graphics items from the view"""
        # Clear all items (including grid)
        items_to_remove = [
            self.robot_arm_item,
            self.robot_joints_item,
            self.tcp_item,
            self.base_item,
            self.trajectory_item,
            self.tool_direction_item,
            self.base_front_item,
            self.workspace_item,
            self.grid_item
        ]

        for item in items_to_remove:
            if item is not None:
                self.removeItem(item)

        # Clear robot mesh items (3D body parts)
        for item in self.robot_mesh_items:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass
        self.robot_mesh_items = []

        # Clear grid label items
        for item in self.grid_label_items:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass
        self.grid_label_items = []

        # Clear base frame items
        for item in self.base_frame_items:
            if item is not None:
                self.removeItem(item)
        self.base_frame_items = []

        # Clear TCP frame items
        for item in self.tcp_frame_items:
            if item is not None:
                self.removeItem(item)
        self.tcp_frame_items = []

        # Reset references
        self.robot_arm_item = None
        self.robot_joints_item = None
        self.tcp_item = None
        self.base_item = None
        self.trajectory_item = None
        self.tool_direction_item = None
        self.base_front_item = None
        self.workspace_item = None
        self.grid_item = None

    def draw_robot_arm(self, joint_positions, active=True, joint_angles=None):
        """
        Draw Thor robot arm - dispatches to STL or primitive rendering.

        Args:
            joint_positions: List of 7 tuples [(x,y,z), ...] for joints
            active: If True, use active colors; if False, use inactive (gray)
            joint_angles: Optional tuple (q1,q2,q3,q4,q5,q6) for STL rendering
        """
        if joint_positions is None or len(joint_positions) < 7:
            return

        # Try STL rendering if available
        if self.use_stl and self.stl_loaded and joint_angles is not None:
            self._draw_robot_stl(joint_angles, active)
            return

        # Fall back to primitive rendering
        self._draw_robot_primitives(joint_positions, active)

    def _draw_robot_stl(self, joint_angles, active=True):
        """
        Draw Thor robot using actual STL mesh files.

        Args:
            joint_angles: Tuple (q1, q2, q3, q4, q5, q6) in degrees
            active: If True, use active colors; if False, use inactive (gray)
        """
        q1, q2, q3, q4, q5, q6 = joint_angles

        # Get cumulative transforms for each joint
        transforms = fk.compute_all_joint_transforms(q1, q2, q3, q4, q5, q6)

        # Clear existing mesh items
        for item in self.robot_mesh_items:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass
        self.robot_mesh_items = []

        # Select colour scheme
        colors = self.colors_active if active else self.colors_inactive

        # STL to transform mapping:
        # base    -> transforms[0] (identity - fixed at origin)
        # art1    -> transforms[1] (after J1)
        # art2    -> transforms[2] (after J1+J2)
        # art3    -> transforms[3] (after J1+J2+J3)
        # art4    -> transforms[4] (after J1+J2+J3+J4)
        # art5    -> transforms[5] (after J1+J2+J3+J4+J5)
        # gripper -> transforms[6] (after all joints - TCP frame)

        stl_mapping = [
            ('base', 0, colors['accent']),
            ('art1', 1, colors['body']),
            ('art2', 2, colors['body']),
            ('art3', 3, colors['body_dark']),
            ('art4', 4, colors['body']),
            ('art5', 5, colors['body_dark']),
            ('gripper', 6, colors['gripper']),
        ]

        for stl_name, transform_idx, color in stl_mapping:
            mesh_data = load_stl_mesh(stl_name)
            if mesh_data is None:
                continue

            vertices, faces = mesh_data
            transform = transforms[transform_idx]
            calibration = self.stl_calibration.get(stl_name)

            # Transform vertices
            transformed_verts = transform_stl_vertices(vertices, transform, calibration)

            # Create mesh item
            mesh_item = gl.GLMeshItem(
                vertexes=transformed_verts,
                faces=faces,
                color=color,
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(mesh_item)
            self.robot_mesh_items.append(mesh_item)

        # Add TCP indicator sphere
        tcp_pos = transforms[6][:3, 3]
        tcp_verts, tcp_faces = create_dome_mesh(10, segments=12, rings=6)
        tcp_bottom = tcp_verts.copy()
        tcp_bottom[:, 2] = -tcp_bottom[:, 2]
        tcp_sphere_verts = np.vstack([tcp_verts, tcp_bottom])
        n_top = len(tcp_verts)
        tcp_sphere_faces = np.vstack([tcp_faces, tcp_faces + n_top])
        tcp_sphere_verts = transform_mesh(tcp_sphere_verts, tcp_pos)

        tcp_mesh = gl.GLMeshItem(
            vertexes=tcp_sphere_verts,
            faces=tcp_sphere_faces,
            color=colors['tcp'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(tcp_mesh)
        self.robot_mesh_items.append(tcp_mesh)

    def _draw_robot_primitives(self, joint_positions, active=True):
        """
        Draw Thor robot arm using geometric primitives (cylinders, boxes).
        Fallback when STL files are not available.

        Joint positions from FK (7 positions):
        [0] Base (origin)
        [1] J1 - After base rotation (top of base column)
        [2] J2 - After shoulder pitch
        [3] J3 - After elbow
        [4] J4 - After wrist roll
        [5] J5 - After wrist pitch
        [6] TCP - After J6 (wrist yaw) / tool center point

        Args:
            joint_positions: List of 7 tuples [(x,y,z), ...] for joints
            active: If True, use active colours; if False, use inactive (grey)
        """
        # DEBUG: Print received positions
        logger.info(f"_draw_robot_primitives received positions:")
        for i, pos in enumerate(joint_positions):
            logger.info(f"  [{i}]: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")

        # Select colour scheme
        colors = self.colors_active if active else self.colors_inactive

        # Clear existing mesh items
        for item in self.robot_mesh_items:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass
        self.robot_mesh_items = []

        # Also clear legacy items
        for item in [self.robot_arm_item, self.robot_joints_item, self.tcp_item,
                     self.base_item, self.tool_direction_item, self.base_front_item]:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass

        self.robot_arm_item = None
        self.robot_joints_item = None
        self.tcp_item = None
        self.base_item = None
        self.tool_direction_item = None
        self.base_front_item = None

        # Convert positions to numpy arrays
        pos = [np.array(p) for p in joint_positions]

        # =====================================================================
        # Thor Robot Geometry (approximate dimensions from image)
        # =====================================================================

        # 1. BASE PLATFORM - Rectangular box at ground level
        base_width = 120   # X dimension
        base_depth = 100   # Y dimension
        base_height = 35   # Z dimension

        base_verts, base_faces = create_box_mesh(base_width, base_depth, base_height)
        base_verts = transform_mesh(base_verts, [0, 0, base_height/2])

        base_mesh = gl.GLMeshItem(
            vertexes=base_verts,
            faces=base_faces,
            color=colors['accent'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(base_mesh)
        self.robot_mesh_items.append(base_mesh)

        # 2. BASE COLUMN - Cylinder from base to J1
        # This is the main rotating column (J1 rotates around Z)
        j1_pos = pos[1]
        column_height = j1_pos[2] - base_height  # Height from base top to J1
        column_radius = 45

        if column_height > 5:
            col_verts, col_faces = create_cylinder_mesh(column_radius, column_height, segments=20)
            col_center_z = base_height + column_height / 2
            col_verts = transform_mesh(col_verts, [0, 0, col_center_z])

            column_mesh = gl.GLMeshItem(
                vertexes=col_verts,
                faces=col_faces,
                color=colors['body'],
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(column_mesh)
            self.robot_mesh_items.append(column_mesh)

        # 3. SHOULDER HOUSING - Motor housing at J1
        shoulder_radius = 45
        shoulder_height = 60

        shoulder_verts, shoulder_faces = create_cylinder_mesh(shoulder_radius, shoulder_height, segments=20)
        shoulder_verts = transform_mesh(shoulder_verts, j1_pos + [0, 0, shoulder_height/2])

        shoulder_mesh = gl.GLMeshItem(
            vertexes=shoulder_verts,
            faces=shoulder_faces,
            color=colors['body'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(shoulder_mesh)
        self.robot_mesh_items.append(shoulder_mesh)

        # Z-axis reference for cylinder orientation
        z_axis = np.array([0, 0, 1])

        # 4. UPPER ARM - Link from J1 to J2 (this is the actual upper arm!)
        # Note: J2 and J3 are at the same position (elbow pivot)
        j2_pos = pos[2]
        j3_pos = pos[3]  # Same as J2
        upper_arm_vec = j2_pos - j1_pos
        upper_arm_length = np.linalg.norm(upper_arm_vec)

        if upper_arm_length > 5:
            arm_radius = 35
            arm_verts, arm_faces = create_cylinder_mesh(arm_radius, upper_arm_length, segments=16)

            rot_matrix_arm = rotation_matrix_from_vectors(z_axis, upper_arm_vec)
            arm_center = (j1_pos + j2_pos) / 2
            arm_verts = transform_mesh(arm_verts, [0, 0, 0], rot_matrix_arm)
            arm_verts = transform_mesh(arm_verts, arm_center)

            upper_arm_mesh = gl.GLMeshItem(
                vertexes=arm_verts,
                faces=arm_faces,
                color=colors['body'],
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(upper_arm_mesh)
            self.robot_mesh_items.append(upper_arm_mesh)

        # 5. ELBOW HOUSING - Motor housing at J2/J3 (elbow pivot point)
        elbow_radius = 40
        elbow_height = 55

        elbow_verts, elbow_faces = create_cylinder_mesh(elbow_radius, elbow_height, segments=16)
        # Orient elbow along the upper arm direction
        if upper_arm_length > 5:
            elbow_verts = transform_mesh(elbow_verts, [0, 0, 0], rot_matrix_arm)
        elbow_verts = transform_mesh(elbow_verts, j2_pos)

        elbow_mesh = gl.GLMeshItem(
            vertexes=elbow_verts,
            faces=elbow_faces,
            color=colors['body_dark'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(elbow_mesh)
        self.robot_mesh_items.append(elbow_mesh)

        # 6. FOREARM - Link from J3 to J4 (J4 and J5 are at the same position)
        j4_pos = pos[4]
        j5_pos = pos[5]  # Same as J4
        forearm_vec = j4_pos - j3_pos
        forearm_length = np.linalg.norm(forearm_vec)

        if forearm_length > 5:
            forearm_radius = 28
            forearm_verts, forearm_faces = create_cylinder_mesh(forearm_radius, forearm_length, segments=16)

            # Rotate to align with forearm direction
            rot_matrix_forearm = rotation_matrix_from_vectors(z_axis, forearm_vec)

            forearm_center = (j3_pos + j4_pos) / 2
            forearm_verts = transform_mesh(forearm_verts, [0, 0, 0], rot_matrix_forearm)
            forearm_verts = transform_mesh(forearm_verts, forearm_center)

            forearm_mesh = gl.GLMeshItem(
                vertexes=forearm_verts,
                faces=forearm_faces,
                color=colors['body'],
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(forearm_mesh)
            self.robot_mesh_items.append(forearm_mesh)

        # 7. WRIST ASSEMBLY - J5 to TCP
        tcp_pos = pos[6]  # TCP is the last position
        wrist_radius = 22
        wrist_height = 40

        # Wrist housing at J5, oriented toward TCP
        wrist_vec = tcp_pos - j5_pos
        wrist_length = np.linalg.norm(wrist_vec)

        wrist_verts, wrist_faces = create_cylinder_mesh(wrist_radius, wrist_height, segments=14)
        if wrist_length > 2:
            rot_matrix_wrist = rotation_matrix_from_vectors(z_axis, wrist_vec)
            wrist_verts = transform_mesh(wrist_verts, [0, 0, 0], rot_matrix_wrist)
        wrist_verts = transform_mesh(wrist_verts, j5_pos)

        wrist_mesh = gl.GLMeshItem(
            vertexes=wrist_verts,
            faces=wrist_faces,
            color=colors['body_dark'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(wrist_mesh)
        self.robot_mesh_items.append(wrist_mesh)

        # 8. TCP/GRIPPER - End effector (from J5 to TCP)
        if wrist_length > 2:
            gripper_radius = 18
            gripper_verts, gripper_faces = create_cylinder_mesh(gripper_radius, wrist_length, segments=12)

            rot_matrix_gripper = rotation_matrix_from_vectors(z_axis, wrist_vec)
            gripper_center = (j5_pos + tcp_pos) / 2
            gripper_verts = transform_mesh(gripper_verts, [0, 0, 0], rot_matrix_gripper)
            gripper_verts = transform_mesh(gripper_verts, gripper_center)

            gripper_mesh = gl.GLMeshItem(
                vertexes=gripper_verts,
                faces=gripper_faces,
                color=colors['gripper'],
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(gripper_mesh)
            self.robot_mesh_items.append(gripper_mesh)

        # 9. JOINT RINGS - Dark accent rings at each joint
        joint_ring_radius = 12
        joint_ring_height = 8

        for i, joint_pos in enumerate(pos[1:6]):  # J1 through J5
            ring_verts, ring_faces = create_cylinder_mesh(joint_ring_radius, joint_ring_height, segments=12)
            ring_verts = transform_mesh(ring_verts, joint_pos)

            ring_mesh = gl.GLMeshItem(
                vertexes=ring_verts,
                faces=ring_faces,
                color=colors['joint'],
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self.addItem(ring_mesh)
            self.robot_mesh_items.append(ring_mesh)

        # 10. TCP INDICATOR - Red sphere at TCP
        tcp_indicator_verts, tcp_indicator_faces = create_dome_mesh(10, segments=12, rings=6)
        # Create full sphere by mirroring dome
        tcp_bottom_verts = tcp_indicator_verts.copy()
        tcp_bottom_verts[:, 2] = -tcp_bottom_verts[:, 2]
        tcp_sphere_verts = np.vstack([tcp_indicator_verts, tcp_bottom_verts])

        # Adjust faces for bottom half
        n_top_verts = len(tcp_indicator_verts)
        tcp_bottom_faces = tcp_indicator_faces + n_top_verts
        tcp_sphere_faces = np.vstack([tcp_indicator_faces, tcp_bottom_faces])

        tcp_sphere_verts = transform_mesh(tcp_sphere_verts, tcp_pos)

        tcp_mesh = gl.GLMeshItem(
            vertexes=tcp_sphere_verts,
            faces=tcp_sphere_faces,
            color=colors['tcp'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )
        self.addItem(tcp_mesh)
        self.robot_mesh_items.append(tcp_mesh)

    def draw_tcp_trajectory(self, tcp_points, timestamps):
        """
        Draw TCP trajectory as gradient colored line

        Args:
            tcp_points: List of (x, y, z) tuples
            timestamps: List of timestamps for each point
        """
        if len(tcp_points) < 2:
            return

        # Remove old trajectory
        if self.trajectory_item:
            self.removeItem(self.trajectory_item)

        # Convert to numpy array
        positions = np.array(tcp_points)

        # Create gradient colors (blue to red based on time)
        n_points = len(positions)
        # Gradient from blue (0,0,1) to red (1,0,0)
        colors = np.zeros((n_points, 4))
        for i in range(n_points):
            t = i / (n_points - 1)  # 0 to 1
            colors[i] = [t, 0, 1-t, 0.8]  # Red increases, blue decreases

        # Create line plot with gradient
        self.trajectory_item = gl.GLLinePlotItem(
            pos=positions,
            color=colors,
            width=2,
            antialias=True,
            mode='line_strip'
        )
        self.addItem(self.trajectory_item)

    def draw_base_frame(self, length=80):
        """
        Draw coordinate frame at base (XYZ axes) with forward direction arrow.
        OPTIMIZED: Only creates items once (base frame is static).

        Args:
            length: Length of each axis arrow in mm
        """
        # OPTIMIZATION: Base frame is static, only create once
        if self._base_frame_initialized and len(self.base_frame_items) > 0:
            # Ensure items are in scene
            for item in self.base_frame_items:
                if item is not None and item not in self.items:
                    self.addItem(item)
            return

        # Remove old base frame items (if any from previous state)
        for item in self.base_frame_items:
            if item is not None:
                try:
                    self.removeItem(item)
                except Exception:
                    pass
        self.base_frame_items = []

        origin = np.array([0, 0, 0])
        arrow_width = 4  # Thicker lines for visibility

        # X axis - Red (FORWARD DIRECTION)
        x_axis_points = np.array([origin, origin + [length, 0, 0]])
        x_axis = gl.GLLinePlotItem(
            pos=x_axis_points,
            color=(1, 0, 0, 1.0),
            width=arrow_width,
            antialias=True
        )
        self.addItem(x_axis)
        self.base_frame_items.append(x_axis)

        # Y axis - Green
        y_axis_points = np.array([origin, origin + [0, length, 0]])
        y_axis = gl.GLLinePlotItem(
            pos=y_axis_points,
            color=(0, 0.8, 0, 1.0),
            width=arrow_width,
            antialias=True
        )
        self.addItem(y_axis)
        self.base_frame_items.append(y_axis)

        # Z axis - Blue
        z_axis_points = np.array([origin, origin + [0, 0, length]])
        z_axis = gl.GLLinePlotItem(
            pos=z_axis_points,
            color=(0, 0, 1, 1.0),
            width=arrow_width,
            antialias=True
        )
        self.addItem(z_axis)
        self.base_frame_items.append(z_axis)

        # FORWARD DIRECTION ARROW (along X+) - prominent indicator
        # Arrow shaft - starts from base front, points along X+
        arrow_start = np.array([60, 0, 5])  # Slightly in front of base, raised a bit
        arrow_end = np.array([150, 0, 5])   # Points forward along X+
        arrow_shaft = gl.GLLinePlotItem(
            pos=np.array([arrow_start, arrow_end]),
            color=(0.9, 0.1, 0.1, 1.0),  # Bright red
            width=6,
            antialias=True
        )
        self.addItem(arrow_shaft)
        self.base_frame_items.append(arrow_shaft)

        # Arrowhead - two angled lines forming a ">" shape
        arrowhead_size = 20
        arrowhead_left = np.array([arrow_end[0] - arrowhead_size, arrowhead_size * 0.5, 5])
        arrowhead_right = np.array([arrow_end[0] - arrowhead_size, -arrowhead_size * 0.5, 5])

        arrowhead1 = gl.GLLinePlotItem(
            pos=np.array([arrow_end, arrowhead_left]),
            color=(0.9, 0.1, 0.1, 1.0),
            width=6,
            antialias=True
        )
        self.addItem(arrowhead1)
        self.base_frame_items.append(arrowhead1)

        arrowhead2 = gl.GLLinePlotItem(
            pos=np.array([arrow_end, arrowhead_right]),
            color=(0.9, 0.1, 0.1, 1.0),
            width=6,
            antialias=True
        )
        self.addItem(arrowhead2)
        self.base_frame_items.append(arrowhead2)

        self._base_frame_initialized = True
        logger.debug("Base frame initialized (persistent)")

    def draw_tcp_frame(self, q1, q2, q3, q4, q5, q6, length=40):
        """
        Draw coordinate frame at TCP showing end effector orientation.
        Applies gripper calibration rotation to match the actual gripper STL orientation.

        Args:
            q1-q6: Joint angles in degrees
            length: Length of each axis arrow in mm
        """
        # Remove old TCP frame items
        for item in self.tcp_frame_items:
            if item is not None:
                self.removeItem(item)
        self.tcp_frame_items = []

        # Get all joint transforms
        transforms = fk.compute_all_joint_transforms(q1, q2, q3, q4, q5, q6)

        # Use TCP position and orientation (transforms[6] = after all joints)
        # This is the raw DH joint 6 frame - no calibration applied
        tcp_pos = transforms[6][0:3, 3]
        tcp_rot = transforms[6][0:3, 0:3]

        # Create frame axes (X, Y, Z in TCP frame)
        x_axis_tcp = tcp_rot @ np.array([length, 0, 0])
        y_axis_tcp = tcp_rot @ np.array([0, length, 0])
        z_axis_tcp = tcp_rot @ np.array([0, 0, length])

        # TCP X axis - Red (thicker for clarity)
        x_points = np.array([tcp_pos, tcp_pos + x_axis_tcp])
        x_axis = gl.GLLinePlotItem(
            pos=x_points,
            color=(1, 0, 0, 1.0),
            width=4,
            antialias=True
        )
        self.addItem(x_axis)
        self.tcp_frame_items.append(x_axis)

        # TCP Y axis - Green (thicker for clarity)
        y_points = np.array([tcp_pos, tcp_pos + y_axis_tcp])
        y_axis = gl.GLLinePlotItem(
            pos=y_points,
            color=(0, 1, 0, 1.0),
            width=4,
            antialias=True
        )
        self.addItem(y_axis)
        self.tcp_frame_items.append(y_axis)

        # TCP Z axis - Blue (thicker for clarity)
        z_points = np.array([tcp_pos, tcp_pos + z_axis_tcp])
        z_axis = gl.GLLinePlotItem(
            pos=z_points,
            color=(0, 0, 1, 1.0),
            width=4,
            antialias=True
        )
        self.addItem(z_axis)
        self.tcp_frame_items.append(z_axis)

    def draw_coordinate_frame(
        self,
        transform: np.ndarray,
        name: str,
        length: float = 50,
        width: float = 3,
        colors: dict = None
    ) -> list:
        """
        Draw a coordinate frame at specified pose.

        Args:
            transform: 4x4 homogeneous transformation matrix
            name: Frame name (for tracking/removal)
            length: Length of each axis arrow in mm
            width: Line width
            colors: Optional dict with 'x', 'y', 'z' RGBA tuples

        Returns:
            List of GLLinePlotItem objects created
        """
        if colors is None:
            colors = {
                'x': (1.0, 0.0, 0.0, 1.0),  # Red
                'y': (0.0, 0.8, 0.0, 1.0),  # Green
                'z': (0.0, 0.0, 1.0, 1.0),  # Blue
            }

        # Extract position and rotation
        origin = transform[:3, 3]
        rotation = transform[:3, :3]

        # Create frame axes
        x_axis_vec = rotation @ np.array([length, 0, 0])
        y_axis_vec = rotation @ np.array([0, length, 0])
        z_axis_vec = rotation @ np.array([0, 0, length])

        items = []

        # X axis
        x_points = np.array([origin, origin + x_axis_vec])
        x_axis = gl.GLLinePlotItem(
            pos=x_points,
            color=colors['x'],
            width=width,
            antialias=True
        )
        self.addItem(x_axis)
        items.append(x_axis)

        # Y axis
        y_points = np.array([origin, origin + y_axis_vec])
        y_axis = gl.GLLinePlotItem(
            pos=y_points,
            color=colors['y'],
            width=width,
            antialias=True
        )
        self.addItem(y_axis)
        items.append(y_axis)

        # Z axis
        z_points = np.array([origin, origin + z_axis_vec])
        z_axis = gl.GLLinePlotItem(
            pos=z_points,
            color=colors['z'],
            width=width,
            antialias=True
        )
        self.addItem(z_axis)
        items.append(z_axis)

        return items

    def update_custom_frames(self, frames: dict):
        """
        Update display of custom coordinate frames (tools, workpieces).

        Args:
            frames: Dict mapping frame names to 4x4 transform matrices
                    e.g., {'workpiece1': transform_matrix, 'tool1': transform_matrix}
        """
        # Remove old custom frame items
        if not hasattr(self, 'custom_frame_items'):
            self.custom_frame_items = {}

        for name, items in list(self.custom_frame_items.items()):
            for item in items:
                if item is not None:
                    self.removeItem(item)
            del self.custom_frame_items[name]

        # Draw new frames
        # Colour schemes for different frame types
        workpiece_colors = {
            'x': (1.0, 0.3, 0.7, 0.9),  # Magenta-ish
            'y': (0.3, 1.0, 0.7, 0.9),  # Cyan-ish
            'z': (0.7, 0.3, 1.0, 0.9),  # Purple-ish
        }
        tool_colors = {
            'x': (1.0, 0.6, 0.0, 0.9),  # Orange
            'y': (0.6, 1.0, 0.0, 0.9),  # Lime
            'z': (0.0, 0.6, 1.0, 0.9),  # Sky blue
        }

        for name, transform in frames.items():
            # Determine colours based on name pattern
            if 'tool' in name.lower():
                colors = tool_colors
                length = 35
            else:
                colors = workpiece_colors
                length = 50

            items = self.draw_coordinate_frame(
                transform=transform,
                name=name,
                length=length,
                width=3,
                colors=colors
            )
            self.custom_frame_items[name] = items

    def clear_custom_frames(self):
        """Remove all custom frame visualisations."""
        if hasattr(self, 'custom_frame_items'):
            for name, items in list(self.custom_frame_items.items()):
                for item in items:
                    if item is not None:
                        self.removeItem(item)
            self.custom_frame_items = {}

    def draw_workspace_limits(self):
        """Draw workspace envelope as semi-transparent cylinder"""
        workspace = fk.compute_workspace_envelope()

        # Remove old workspace item
        if self.workspace_item:
            self.removeItem(self.workspace_item)

        # Create cylinder mesh
        radius = workspace['radius']
        z_min = workspace['z_min']
        z_max = workspace['z_max']

        # Cylinder parameters
        n_segments = 30
        theta = np.linspace(0, 2*np.pi, n_segments)

        # Create cylinder surface points
        n_z = 20
        z_levels = np.linspace(z_min, z_max, n_z)

        # Create mesh data
        verts = []
        faces = []

        for i, z in enumerate(z_levels):
            for t in theta:
                x = radius * np.cos(t)
                y = radius * np.sin(t)
                verts.append([x, y, z])

        verts = np.array(verts)

        # Create faces (triangles connecting adjacent rings)
        for i in range(n_z - 1):
            for j in range(n_segments - 1):
                # Two triangles per quad
                idx1 = i * n_segments + j
                idx2 = i * n_segments + (j + 1)
                idx3 = (i + 1) * n_segments + j
                idx4 = (i + 1) * n_segments + (j + 1)

                faces.append([idx1, idx2, idx3])
                faces.append([idx2, idx4, idx3])

        faces = np.array(faces)

        # Create mesh item
        mesh_data = gl.MeshData(vertexes=verts, faces=faces)
        self.workspace_item = gl.GLMeshItem(
            meshdata=mesh_data,
            color=(0, 0, 1, 0.05),  # Very transparent blue
            shader='edgeHilight',
            smooth=True,
            drawEdges=True,
            edgeColor=(0, 0, 1, 0.2)
        )
        self.addItem(self.workspace_item)

    def _compute_fk_cached(self, q1, q2, q3, q4, q5, q6):
        """
        Compute forward kinematics with caching to avoid redundant calculations

        Args:
            q1-q6: Joint angles in degrees

        Returns:
            List of joint positions
        """
        # Create cache key (round to 0.1 degree precision)
        cache_key = (round(q1, 1), round(q2, 1), round(q3, 1),
                     round(q4, 1), round(q5, 1), round(q6, 1))

        with self._fk_cache_lock:
            if cache_key in self._fk_cache:
                return self._fk_cache[cache_key]

            # Compute FK if not cached
            positions = fk.compute_all_joint_positions(q1, q2, q3, q4, q5, q6)

            # Cache result (limit cache size to prevent memory bloat)
            if len(self._fk_cache) > 100:
                # Remove oldest entry
                self._fk_cache.pop(next(iter(self._fk_cache)))

            self._fk_cache[cache_key] = positions
            return positions

    def _get_workspace_envelope_cached(self):
        """
        Get workspace envelope with caching (only changes when DH parameters change).

        Returns:
            Dict with 'radius', 'z_min', 'z_max'
        """
        if self._workspace_envelope_cache is None:
            self._workspace_envelope_cache = fk.compute_workspace_envelope()
        return self._workspace_envelope_cache

    def invalidate_workspace_cache(self):
        """Invalidate workspace cache (call when DH parameters change)"""
        self._workspace_envelope_cache = None
        self._grid_initialized = False  # Force grid recreation

    def _numpy_to_qmatrix4x4(self, np_matrix):
        """
        Convert numpy 4x4 matrix to QMatrix4x4 for OpenGL transforms.

        Args:
            np_matrix: 4x4 numpy array

        Returns:
            QtGui.QMatrix4x4
        """
        # QMatrix4x4 constructor takes row-major values
        m = QtGui.QMatrix4x4(
            np_matrix[0, 0], np_matrix[0, 1], np_matrix[0, 2], np_matrix[0, 3],
            np_matrix[1, 0], np_matrix[1, 1], np_matrix[1, 2], np_matrix[1, 3],
            np_matrix[2, 0], np_matrix[2, 1], np_matrix[2, 2], np_matrix[2, 3],
            np_matrix[3, 0], np_matrix[3, 1], np_matrix[3, 2], np_matrix[3, 3]
        )
        return m

    def _initialize_persistent_meshes(self):
        """
        Create persistent mesh items for STL rendering (called once).
        Meshes are created with identity transform and updated via setTransform().
        """
        if self._persistent_meshes_initialized:
            return

        if not self.use_stl or not self.stl_loaded:
            logger.debug("STL not available, skipping persistent mesh initialization")
            return

        colors = self.colors_active

        stl_mapping = [
            ('base', colors['accent']),
            ('art1', colors['body']),
            ('art2', colors['body']),
            ('art3', colors['body_dark']),
            ('art4', colors['body']),
            ('art5', colors['body_dark']),
            ('gripper', colors['gripper']),
        ]

        for stl_name, color in stl_mapping:
            mesh_data = load_stl_mesh(stl_name)
            if mesh_data is None:
                continue

            vertices, faces = mesh_data
            calibration = self.stl_calibration.get(stl_name)

            # Apply calibration to vertices (this is static, done once)
            calibrated_verts = self._apply_calibration(vertices, calibration)

            # Create mesh item with calibrated vertices (no FK transform yet)
            mesh_item = gl.GLMeshItem(
                vertexes=calibrated_verts,
                faces=faces,
                color=color,
                shader='edgeHilight',
                smooth=True,
                drawEdges=False
            )
            self._persistent_mesh_items[stl_name] = mesh_item
            # Don't add to scene yet - done in update

        # Create TCP indicator sphere (persistent)
        tcp_verts, tcp_faces = create_dome_mesh(10, segments=12, rings=6)
        tcp_bottom = tcp_verts.copy()
        tcp_bottom[:, 2] = -tcp_bottom[:, 2]
        tcp_sphere_verts = np.vstack([tcp_verts, tcp_bottom])
        n_top = len(tcp_verts)
        tcp_sphere_faces = np.vstack([tcp_faces, tcp_faces + n_top])

        self._tcp_mesh_item = gl.GLMeshItem(
            vertexes=tcp_sphere_verts,
            faces=tcp_sphere_faces,
            color=colors['tcp'],
            shader='edgeHilight',
            smooth=True,
            drawEdges=False
        )

        self._persistent_meshes_initialized = True
        logger.info(f"Initialized {len(self._persistent_mesh_items)} persistent mesh items")

    def _apply_calibration(self, vertices, calibration):
        """
        Apply calibration offset/rotation/scale to vertices (static, done once).

        Args:
            vertices: Nx3 array of vertex positions
            calibration: Optional dict with 'offset', 'rotation', 'scale'

        Returns:
            Calibrated Nx3 vertex array
        """
        if calibration is None:
            return vertices.astype(np.float32)

        verts = vertices.copy()

        # Scale first
        scale = calibration.get('scale', 1.0)
        verts = verts * scale

        # Offset in original STL coordinates
        offset = calibration.get('offset', [0, 0, 0])
        verts = verts + np.array(offset)

        # Rotate to align with robot frame
        rot = calibration.get('rotation', [0, 0, 0])
        if any(r != 0 for r in rot):
            R_local = euler_to_rotation_matrix(*rot)
            verts = verts @ R_local.T

        return verts.astype(np.float32)

    def _update_robot_transforms(self, joint_angles):
        """
        Update robot mesh transforms using GPU-side setTransform() (fast path).

        Args:
            joint_angles: Tuple (q1, q2, q3, q4, q5, q6) in degrees
        """
        if not self._persistent_meshes_initialized:
            self._initialize_persistent_meshes()

        if not self._persistent_mesh_items:
            return  # Fall back handled elsewhere

        q1, q2, q3, q4, q5, q6 = joint_angles

        # Get cumulative transforms for each joint
        transforms = fk.compute_all_joint_transforms(q1, q2, q3, q4, q5, q6)

        # Transform index mapping for each STL part
        transform_indices = {
            'base': 0,
            'art1': 1,
            'art2': 2,
            'art3': 3,
            'art4': 4,
            'art5': 5,
            'gripper': 6,
        }

        # Update each mesh transform
        for stl_name, mesh_item in self._persistent_mesh_items.items():
            transform_idx = transform_indices.get(stl_name, 0)
            transform_4x4 = transforms[transform_idx]

            # Convert to QMatrix4x4 and apply
            qmatrix = self._numpy_to_qmatrix4x4(transform_4x4)
            mesh_item.resetTransform()
            mesh_item.applyTransform(qmatrix, local=False)

        # Update TCP position
        if self._tcp_mesh_item:
            tcp_transform = transforms[6].copy()
            qmatrix = self._numpy_to_qmatrix4x4(tcp_transform)
            self._tcp_mesh_item.resetTransform()
            self._tcp_mesh_item.applyTransform(qmatrix, local=False)

    def _ensure_meshes_in_scene(self):
        """Ensure all persistent mesh items are added to the scene."""
        for stl_name, mesh_item in self._persistent_mesh_items.items():
            if mesh_item not in self.items:
                self.addItem(mesh_item)

        if self._tcp_mesh_item and self._tcp_mesh_item not in self.items:
            self.addItem(self._tcp_mesh_item)

    def _remove_meshes_from_scene(self):
        """Remove all persistent mesh items from the scene."""
        for stl_name, mesh_item in self._persistent_mesh_items.items():
            if mesh_item in self.items:
                try:
                    self.removeItem(mesh_item)
                except Exception:
                    pass

        if self._tcp_mesh_item and self._tcp_mesh_item in self.items:
            try:
                self.removeItem(self._tcp_mesh_item)
            except Exception:
                pass

    def _ensure_grid_initialized(self):
        """
        Initialize grid once (persistent). Only recreates if workspace changes.
        """
        if self._grid_initialized and self.grid_item is not None:
            # Grid exists - just ensure it's in the scene
            if self.grid_item not in self.items:
                self.addItem(self.grid_item)
            return

        # Create grid using cached workspace
        workspace = self._get_workspace_envelope_cached()
        max_reach = workspace['radius']
        grid_size = max_reach * 2.5

        # Remove old grid if exists
        if self.grid_item is not None:
            try:
                self.removeItem(self.grid_item)
            except Exception:
                pass

        self.grid_item = gl.GLGridItem()
        self.grid_item.setSize(grid_size, grid_size)
        self.grid_item.setSpacing(50, 50)
        self.grid_item.setColor(pg.mkColor(50, 50, 50, 200))
        self.grid_item.translate(0, 0, 0)
        self.addItem(self.grid_item)

        self._grid_initialized = True
        logger.debug("Grid initialized (persistent)")

    def _has_data_changed(self, current_angles, trajectory_length):
        """
        Check if visualization data has actually changed

        Args:
            current_angles: Dict of current joint angles
            trajectory_length: Number of trajectory points

        Returns:
            True if data changed, False otherwise
        """
        if self._last_joint_angles is None:
            return True

        # Check if joint angles changed significantly (>0.5 degrees)
        for key in ['art1', 'art2', 'art3', 'art4', 'art5', 'art6']:
            old_val = self._last_joint_angles.get(key, 0)
            new_val = current_angles.get(key, 0)
            if abs(new_val - old_val) > 0.5:
                return True

        # Check if trajectory changed
        if trajectory_length != self._last_trajectory_length:
            return True

        return False

    def update_visualization(self, position_history, window_size=60, options=None):
        """
        Update the 3D visualization with current robot state and trajectory.
        OPTIMIZED: Uses GPU transforms for mesh updates - no recreation.

        Args:
            position_history: PositionHistory object
            window_size: Time window in seconds
            options: Dictionary of display options
        """
        # Safety check: ensure widget is still valid
        try:
            if not self.isVisible():
                logger.debug("Skipping update - widget not visible")
                return
        except RuntimeError:
            # Widget has been deleted
            logger.debug("Skipping update - widget destroyed")
            return

        # Prevent concurrent updates (thread-safe)
        if not self._render_lock.acquire(blocking=False):
            logger.debug("Skipping update - render in progress")
            return

        try:
            if options is None:
                options = {}

            # Update display options
            self.show_robot = options.get('show_robot', True)
            self.show_trajectory = options.get('show_trajectory', True)
            self.show_base_frame = options.get('show_base_frame', True)
            self.show_workspace = options.get('show_workspace', False)
            self.show_grid = options.get('show_grid', True)
            self.show_labels = options.get('show_labels', False)
            self.auto_rotate = options.get('auto_rotate', False)

            # If no data, show home position (only once)
            if len(position_history) == 0:
                if self._is_dirty:
                    self.show_home_position()
                    self._is_dirty = False
                return

            # Get current joint angles (most recent)
            current_angles = position_history.get_current_joint_angles()
            if current_angles is None:
                if self._is_dirty:
                    self.show_home_position()
                    self._is_dirty = False
                return

            # OPTIMIZATION: Check if data actually changed
            tcp_trajectory = position_history.get_tcp_trajectory(window_size) if self.show_trajectory else []
            if not self._has_data_changed(current_angles, len(tcp_trajectory)) and not self.auto_rotate:
                logger.debug("No data change - skipping redraw")
                return

            # Data changed - update cache
            self._last_joint_angles = current_angles.copy()
            self._last_trajectory_length = len(tcp_trajectory)

            # Extract joint angles as tuple
            joint_angles = (
                current_angles.get('art1', 0),
                current_angles.get('art2', 0),
                current_angles.get('art3', 0),
                current_angles.get('art4', 0),
                current_angles.get('art5', 0),
                current_angles.get('art6', 0)
            )

            # OPTIMIZATION: Ensure grid is initialized (persistent, not recreated)
            if self.show_grid:
                self._ensure_grid_initialized()

            # OPTIMIZATION: Use persistent meshes with GPU transforms (fast path)
            if self.show_robot:
                if self.use_stl and self.stl_loaded:
                    # Fast path: Update transforms on existing mesh items
                    self._initialize_persistent_meshes()  # No-op if already done
                    self._ensure_meshes_in_scene()
                    self._update_robot_transforms(joint_angles)
                else:
                    # Fallback: Use primitive rendering (slower, recreates meshes)
                    current_positions = self._compute_fk_cached(*joint_angles)
                    self.current_joint_positions = current_positions
                    self._draw_robot_primitives(current_positions, active=True)

                # Draw TCP coordinate frame
                self.draw_tcp_frame(*joint_angles, length=50)
            else:
                # Robot hidden - remove meshes from scene
                self._remove_meshes_from_scene()

            # Draw TCP trajectory if enabled (this still needs recreation for now)
            if self.show_trajectory and len(tcp_trajectory) > 1:
                # Remove old trajectory
                if self.trajectory_item is not None:
                    try:
                        self.removeItem(self.trajectory_item)
                    except Exception:
                        pass
                    self.trajectory_item = None

                tcp_points = [(x, y, z) for x, y, z, _ in tcp_trajectory]
                timestamps = [t for _, _, _, t in tcp_trajectory]
                self.draw_tcp_trajectory(tcp_points, timestamps)

            # Draw base frame if enabled
            if self.show_base_frame:
                self.draw_base_frame()

            # Draw workspace limits if enabled
            if self.show_workspace:
                self.draw_workspace_limits()

            # Auto-rotate if enabled
            if self.auto_rotate:
                self.rotation_angle += 0.5
                if self.rotation_angle >= 360:
                    self.rotation_angle = 0
                self.setCameraPosition(azimuth=self.rotation_angle)

            logger.debug("3D visualization updated (fast path)")

        finally:
            self._render_lock.release()

    def reset_view(self):
        """Reset view to default isometric angle"""
        workspace = self._get_workspace_envelope_cached()
        max_reach = workspace['radius']
        z_max = workspace['z_max']
        distance = max_reach * 2.5

        # Reset camera centre above base
        center_z = z_max * 0.35
        self.camera_center = pg.Vector(0, 0, center_z)

        self.rotation_angle = 45
        self.setCameraPosition(distance=distance, elevation=30, azimuth=45)
        self.opts['center'] = self.camera_center
        logger.info("View reset to isometric")

    def draw(self):
        """
        Compatibility method for matplotlib API
        PyQtGraph updates automatically, so this is a no-op
        """
        pass


# Example usage and testing
if __name__ == '__main__':
    import sys
    from PyQt5 import QtWidgets

    logging.basicConfig(level=logging.DEBUG)

    print("Thor Robot 3D Visualizer Test (PyQtGraph)")
    print("=" * 50)

    app = QtWidgets.QApplication(sys.argv)

    # Create main window
    window = QtWidgets.QMainWindow()
    window.setWindowTitle("Thor Robot 3D Visualizer Test (PyQtGraph)")
    window.setGeometry(100, 100, 900, 700)

    # Create central widget
    central_widget = QtWidgets.QWidget()
    window.setCentralWidget(central_widget)

    # Create layout
    layout = QtWidgets.QVBoxLayout(central_widget)

    # Create 3D canvas
    canvas = Robot3DCanvas(central_widget, width=800, height=600)
    layout.addWidget(canvas)

    # Show window
    window.show()

    print("\nShowing home position by default")
    print("Mouse controls:")
    print("  - Left drag: Rotate")
    print("  - Right drag: Pan")
    print("  - Wheel: Zoom")
    print("Close window to exit")

    sys.exit(app.exec_())
