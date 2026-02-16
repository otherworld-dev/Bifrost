"""
Forward Kinematics module for Thor Robot Arm - CONFIGURABLE VERSION
Uses DH parameters loaded from dh_parameters.json

DH Parameters format:
- theta_offset: Joint angle offset in degrees
- d: Link offset along z-axis in mm
- a: Link length along x-axis in mm
- alpha: Twist angle in degrees
"""

from typing import List, Tuple
import numpy as np
import numpy.typing as npt
import logging
import json
from pathlib import Path

logger = logging.getLogger(__name__)

# Default Thor Robot Link Lengths (in mm) - used if config file not found
L1 = 202.00  # Base height (d1)
L2 = 160.00  # Upper arm length (a2)
L3 = 195.00  # Forearm length (d4)
L4 = 67.15   # Wrist to TCP length (d6)

# DH Parameters - will be loaded from file
_dh_params = None


def load_dh_parameters():
    """Load DH parameters from JSON file"""
    global _dh_params, L1, L2, L3, L4

    dh_file = Path(__file__).parent / 'dh_parameters.json'

    try:
        if dh_file.exists():
            with open(dh_file, 'r') as f:
                data = json.load(f)
            _dh_params = data['links']

            # Update link lengths from DH parameters
            L1 = _dh_params[0]['d']  # Link 1: d
            L2 = _dh_params[1]['a']  # Link 2: a
            L3 = _dh_params[3]['d']  # Link 4: d
            L4 = _dh_params[5]['d']  # Link 6: d

            logger.info(f"Loaded DH parameters: L1={L1}, L2={L2}, L3={L3}, L4={L4}")
            return _dh_params
        else:
            logger.warning("DH parameters file not found, using defaults")
            return None
    except Exception as e:
        logger.error(f"Error loading DH parameters: {e}")
        return None


def get_dh_params():
    """Get current DH parameters, loading if necessary"""
    global _dh_params
    if _dh_params is None:
        load_dh_parameters()
    return _dh_params


def reload_dh_parameters():
    """Force reload of DH parameters from file"""
    global _dh_params
    _dh_params = None
    return load_dh_parameters()


def apply_dh_parameters(params):
    """
    Apply DH parameters directly (for live preview, without file I/O)

    Args:
        params: List of dicts with keys: link, theta_offset, d, a, alpha
    """
    global _dh_params, L1, L2, L3, L4

    _dh_params = params

    # Update link lengths from parameters
    if params and len(params) >= 6:
        L1 = params[0]['d']  # Link 1: d
        L2 = params[1]['a']  # Link 2: a
        L3 = params[3]['d']  # Link 4: d
        L4 = params[5]['d']  # Link 6: d

    logger.debug(f"Applied DH parameters: L1={L1}, L2={L2}, L3={L3}, L4={L4}")


def get_theta_offset(link_index):
    """Get theta offset for a link (0-indexed)"""
    params = get_dh_params()
    if params and link_index < len(params):
        return np.radians(params[link_index]['theta_offset'])
    return 0.0


def get_direction(link_index):
    """Get direction multiplier for a link (0-indexed). Returns 1 or -1."""
    params = get_dh_params()
    if params and link_index < len(params):
        return params[link_index].get('direction', 1)
    return 1


def get_link_lengths() -> dict:
    """
    Get robot link lengths from DH parameters.

    Returns:
        dict with keys L1, L2, L3, L4 (in mm)
    """
    params = get_dh_params()
    if params and len(params) >= 6:
        return {
            'L1': params[0]['d'],   # Base height
            'L2': params[1]['a'],   # Upper arm length
            'L3': params[3]['d'],   # Forearm length
            'L4': params[5]['d']    # Wrist to TCP
        }
    return {'L1': 202.0, 'L2': 160.0, 'L3': 195.0, 'L4': 67.15}


# Load parameters on module import
load_dh_parameters()


def compute_all_joint_positions(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> List[Tuple[float, float, float]]:
    """
    Compute positions of all joints and TCP using forward kinematics
    Uses Thor-specific transformation matrices with DH parameters from config

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        List of 8 tuples [(x, y, z), ...] representing:
        [Base, J1, J2, J3, J4, J5, J6, TCP]

    Note:
        Base is always at (0, 0, 0)
        All positions in mm
    """
    # Get DH parameters for link lengths
    params = get_dh_params()
    if params is None:
        logger.error("DH parameters not loaded")
        return [(0, 0, 0)] * 8

    # Extract link parameters
    d1 = params[0]['d']   # Base height (Link 1)
    a2 = params[1]['a']   # Upper arm length (Link 2)
    d4 = params[3]['d']   # Forearm length (Link 4)
    d6 = params[5]['d']   # Wrist to TCP (Link 6)

    # Extract theta offsets and direction from DH parameters
    theta_offset1 = np.radians(params[0]['theta_offset'])
    theta_offset2 = np.radians(params[1]['theta_offset'])
    theta_offset3 = np.radians(params[2]['theta_offset'])
    theta_offset4 = np.radians(params[3]['theta_offset'])
    theta_offset5 = np.radians(params[4]['theta_offset'])
    theta_offset6 = np.radians(params[5]['theta_offset'])

    # Direction multipliers (default to 1 for backward compatibility)
    dir1 = params[0].get('direction', 1)
    dir2 = params[1].get('direction', 1)
    dir3 = params[2].get('direction', 1)
    dir4 = params[3].get('direction', 1)
    dir5 = params[4].get('direction', 1)
    dir6 = params[5].get('direction', 1)

    # Convert joint angles to radians: apply direction, then add theta_offset
    # Formula: q_rad = radians(q * direction) + theta_offset
    q1_rad = np.radians(q1 * dir1) + theta_offset1
    q2_rad = np.radians(q2 * dir2) + theta_offset2
    q3_rad = np.radians(q3 * dir3) + theta_offset3
    q4_rad = np.radians(q4 * dir4) + theta_offset4
    q5_rad = np.radians(q5 * dir5) + theta_offset5
    q6_rad = np.radians(q6 * dir6) + theta_offset6

    # Compute trig values
    c1, s1 = np.cos(q1_rad), np.sin(q1_rad)
    c2, s2 = np.cos(q2_rad), np.sin(q2_rad)
    c3, s3 = np.cos(q3_rad), np.sin(q3_rad)
    c4, s4 = np.cos(q4_rad), np.sin(q4_rad)
    c5, s5 = np.cos(q5_rad), np.sin(q5_rad)
    c6, s6 = np.cos(q6_rad), np.sin(q6_rad)

    # Get alpha values from parameters
    alpha1 = np.radians(params[0]['alpha'])  # 90
    alpha2 = np.radians(params[1]['alpha'])  # 0
    alpha3 = np.radians(params[2]['alpha'])  # -90
    alpha4 = np.radians(params[3]['alpha'])  # -90
    alpha5 = np.radians(params[4]['alpha'])  # 90
    alpha6 = np.radians(params[5]['alpha'])  # 0

    ca1, sa1 = np.cos(alpha1), np.sin(alpha1)
    ca3, sa3 = np.cos(alpha3), np.sin(alpha3)
    ca4, sa4 = np.cos(alpha4), np.sin(alpha4)
    ca5, sa5 = np.cos(alpha5), np.sin(alpha5)

    # Standard DH transformation matrices
    # A = Rz(θ) * Tz(d) * Tx(a) * Rx(α)

    # Link 1
    A1 = np.array([
        [c1, -s1 * ca1,  s1 * sa1, 0],
        [s1,  c1 * ca1, -c1 * sa1, 0],
        [0,   sa1,       ca1,      d1],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    # Link 2: α=0°, translation along x
    A2 = np.array([
        [c2, -s2, 0, a2 * c2],
        [s2,  c2, 0, a2 * s2],
        [0,   0,  1, 0],
        [0,   0,  0, 1]
    ], dtype=np.float64)

    # Link 3
    A3 = np.array([
        [c3, -s3 * ca3,  s3 * sa3, 0],
        [s3,  c3 * ca3, -c3 * sa3, 0],
        [0,   sa3,       ca3,      0],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    # Link 4
    A4 = np.array([
        [c4, -s4 * ca4,  s4 * sa4, 0],
        [s4,  c4 * ca4, -c4 * sa4, 0],
        [0,   sa4,       ca4,      d4],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    # Link 5
    A5 = np.array([
        [c5, -s5 * ca5,  s5 * sa5, 0],
        [s5,  c5 * ca5, -c5 * sa5, 0],
        [0,   sa5,       ca5,      0],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    # Link 6: uses alpha from config for TCP frame orientation
    d6 = params[5]['d']
    ca6, sa6 = np.cos(np.radians(params[5]['alpha'])), np.sin(np.radians(params[5]['alpha']))
    A6 = np.array([
        [c6, -s6*ca6,  s6*sa6, 0],
        [s6,  c6*ca6, -c6*sa6, 0],
        [0,   sa6,     ca6,    d6],
        [0,   0,       0,      1]
    ], dtype=np.float64)

    # Compute cumulative transformations
    positions = [(0.0, 0.0, 0.0)]  # Base at origin

    T = np.eye(4)
    for A in [A1, A2, A3, A4, A5, A6]:
        T = T @ A
        pos = T[0:3, 3]
        positions.append(tuple(pos))

    logger.debug(f"FK: Computed {len(positions)} joint positions")

    return positions


def compute_all_joint_transforms(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> List[np.ndarray]:
    """
    Compute cumulative transformation matrices for all joints.
    Used for STL mesh visualization where we need full pose (position + orientation).

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        List of 7 4x4 transformation matrices:
        [T_base, T_1, T_12, T_123, T_1234, T_12345, T_123456]
        Where T_base is identity (base frame), T_1 is after joint 1, etc.
    """
    params = get_dh_params()
    if params is None:
        logger.error("DH parameters not loaded")
        return [np.eye(4)] * 7

    # Extract link parameters
    d1 = params[0]['d']
    a2 = params[1]['a']
    d4 = params[3]['d']

    # Extract theta offsets and direction from DH parameters
    theta_offset1 = np.radians(params[0]['theta_offset'])
    theta_offset2 = np.radians(params[1]['theta_offset'])
    theta_offset3 = np.radians(params[2]['theta_offset'])
    theta_offset4 = np.radians(params[3]['theta_offset'])
    theta_offset5 = np.radians(params[4]['theta_offset'])
    theta_offset6 = np.radians(params[5]['theta_offset'])

    dir1 = params[0].get('direction', 1)
    dir2 = params[1].get('direction', 1)
    dir3 = params[2].get('direction', 1)
    dir4 = params[3].get('direction', 1)
    dir5 = params[4].get('direction', 1)
    dir6 = params[5].get('direction', 1)

    q1_rad = np.radians(q1 * dir1) + theta_offset1
    q2_rad = np.radians(q2 * dir2) + theta_offset2
    q3_rad = np.radians(q3 * dir3) + theta_offset3
    q4_rad = np.radians(q4 * dir4) + theta_offset4
    q5_rad = np.radians(q5 * dir5) + theta_offset5
    q6_rad = np.radians(q6 * dir6) + theta_offset6

    c1, s1 = np.cos(q1_rad), np.sin(q1_rad)
    c2, s2 = np.cos(q2_rad), np.sin(q2_rad)
    c3, s3 = np.cos(q3_rad), np.sin(q3_rad)
    c4, s4 = np.cos(q4_rad), np.sin(q4_rad)
    c5, s5 = np.cos(q5_rad), np.sin(q5_rad)
    c6, s6 = np.cos(q6_rad), np.sin(q6_rad)

    alpha1 = np.radians(params[0]['alpha'])
    alpha3 = np.radians(params[2]['alpha'])
    alpha4 = np.radians(params[3]['alpha'])
    alpha5 = np.radians(params[4]['alpha'])

    ca1, sa1 = np.cos(alpha1), np.sin(alpha1)
    ca3, sa3 = np.cos(alpha3), np.sin(alpha3)
    ca4, sa4 = np.cos(alpha4), np.sin(alpha4)
    ca5, sa5 = np.cos(alpha5), np.sin(alpha5)

    # Build transformation matrices (same as compute_all_joint_positions)
    A1 = np.array([
        [c1, -s1 * ca1,  s1 * sa1, 0],
        [s1,  c1 * ca1, -c1 * sa1, 0],
        [0,   sa1,       ca1,      d1],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    A2 = np.array([
        [c2, -s2, 0, a2 * c2],
        [s2,  c2, 0, a2 * s2],
        [0,   0,  1, 0],
        [0,   0,  0, 1]
    ], dtype=np.float64)

    A3 = np.array([
        [c3, -s3 * ca3,  s3 * sa3, 0],
        [s3,  c3 * ca3, -c3 * sa3, 0],
        [0,   sa3,       ca3,      0],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    A4 = np.array([
        [c4, -s4 * ca4,  s4 * sa4, 0],
        [s4,  c4 * ca4, -c4 * sa4, 0],
        [0,   sa4,       ca4,      d4],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    A5 = np.array([
        [c5, -s5 * ca5,  s5 * sa5, 0],
        [s5,  c5 * ca5, -c5 * sa5, 0],
        [0,   sa5,       ca5,      0],
        [0,   0,         0,        1]
    ], dtype=np.float64)

    d6 = params[5]['d']
    ca6, sa6 = np.cos(np.radians(params[5]['alpha'])), np.sin(np.radians(params[5]['alpha']))
    A6 = np.array([
        [c6, -s6*ca6,  s6*sa6, 0],
        [s6,  c6*ca6, -c6*sa6, 0],
        [0,   sa6,     ca6,    d6],
        [0,   0,       0,      1]
    ], dtype=np.float64)

    # Compute cumulative transforms
    transforms = [np.eye(4)]  # Base transform (identity)

    T = np.eye(4)
    for A in [A1, A2, A3, A4, A5, A6]:
        T = T @ A
        transforms.append(T.copy())

    return transforms


def compute_tcp_position_only(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> Tuple[float, float, float]:
    """
    Compute only TCP position (faster than computing all joints)

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        Tuple (x, y, z) representing TCP position in mm
    """
    # Just get last position from full FK
    positions = compute_all_joint_positions(q1, q2, q3, q4, q5, q6)
    return positions[-1]


def compute_tcp_transform(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> np.ndarray:
    """
    Compute TCP transformation matrix (position and orientation)
    Uses Thor-specific DH convention matching compute_all_joint_positions

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        4x4 transformation matrix representing TCP pose in base frame
    """
    # Get DH parameters for link lengths
    params = get_dh_params()
    if params is None:
        logger.error("DH parameters not loaded")
        return np.eye(4)

    # Extract link parameters
    d1 = params[0]['d']
    a2 = params[1]['a']
    d4 = params[3]['d']
    d6 = params[5]['d']  # TCP offset (d parameter)

    # Extract theta offsets and direction, then compute joint angles
    # Direction multipliers (default to 1 for backward compatibility)
    dir1 = params[0].get('direction', 1)
    dir2 = params[1].get('direction', 1)
    dir3 = params[2].get('direction', 1)
    dir4 = params[3].get('direction', 1)
    dir5 = params[4].get('direction', 1)
    dir6 = params[5].get('direction', 1)

    # Formula: q_rad = radians(q * direction) + theta_offset
    q1_rad = np.radians(q1 * dir1) + np.radians(params[0]['theta_offset'])
    q2_rad = np.radians(q2 * dir2) + np.radians(params[1]['theta_offset'])
    q3_rad = np.radians(q3 * dir3) + np.radians(params[2]['theta_offset'])
    q4_rad = np.radians(q4 * dir4) + np.radians(params[3]['theta_offset'])
    q5_rad = np.radians(q5 * dir5) + np.radians(params[4]['theta_offset'])
    q6_rad = np.radians(q6 * dir6) + np.radians(params[5]['theta_offset'])

    c1, s1 = np.cos(q1_rad), np.sin(q1_rad)
    c2, s2 = np.cos(q2_rad), np.sin(q2_rad)
    c3, s3 = np.cos(q3_rad), np.sin(q3_rad)
    c4, s4 = np.cos(q4_rad), np.sin(q4_rad)
    c5, s5 = np.cos(q5_rad), np.sin(q5_rad)
    c6, s6 = np.cos(q6_rad), np.sin(q6_rad)

    ca1, sa1 = np.cos(np.radians(params[0]['alpha'])), np.sin(np.radians(params[0]['alpha']))
    ca3, sa3 = np.cos(np.radians(params[2]['alpha'])), np.sin(np.radians(params[2]['alpha']))
    ca4, sa4 = np.cos(np.radians(params[3]['alpha'])), np.sin(np.radians(params[3]['alpha']))
    ca5, sa5 = np.cos(np.radians(params[4]['alpha'])), np.sin(np.radians(params[4]['alpha']))
    ca6, sa6 = np.cos(np.radians(params[5]['alpha'])), np.sin(np.radians(params[5]['alpha']))

    # Build transformation matrices (same as compute_all_joint_positions)
    A1 = np.array([[c1, -s1*ca1, s1*sa1, 0], [s1, c1*ca1, -c1*sa1, 0], [0, sa1, ca1, d1], [0, 0, 0, 1]], dtype=np.float64)
    A2 = np.array([[c2, -s2, 0, a2*c2], [s2, c2, 0, a2*s2], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
    A3 = np.array([[c3, -s3*ca3, s3*sa3, 0], [s3, c3*ca3, -c3*sa3, 0], [0, sa3, ca3, 0], [0, 0, 0, 1]], dtype=np.float64)
    A4 = np.array([[c4, -s4*ca4, s4*sa4, 0], [s4, c4*ca4, -c4*sa4, 0], [0, sa4, ca4, d4], [0, 0, 0, 1]], dtype=np.float64)
    A5 = np.array([[c5, -s5*ca5, s5*sa5, 0], [s5, c5*ca5, -c5*sa5, 0], [0, sa5, ca5, 0], [0, 0, 0, 1]], dtype=np.float64)
    A6 = np.array([[c6, -s6*ca6, s6*sa6, 0], [s6, c6*ca6, -c6*sa6, 0], [0, sa6, ca6, d6], [0, 0, 0, 1]], dtype=np.float64)

    return A1 @ A2 @ A3 @ A4 @ A5 @ A6


def compute_tool_transform(
    q1: float, q2: float, q3: float, q4: float, q5: float, q6: float,
    tool_offset: np.ndarray = None
) -> np.ndarray:
    """
    Compute tool tip transformation (TCP + tool offset).

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees
        tool_offset: Optional 4x4 tool offset matrix relative to TCP

    Returns:
        4x4 transformation matrix of tool tip in base frame
    """
    tcp_transform = compute_tcp_transform(q1, q2, q3, q4, q5, q6)

    if tool_offset is not None:
        return tcp_transform @ tool_offset
    return tcp_transform


def compute_workspace_envelope() -> dict:
    """
    Compute workspace envelope parameters for visualization

    Returns:
        Dictionary with workspace parameters
    """
    # Maximum horizontal reach (fully extended in XY plane)
    max_horizontal_reach = L2 + L3 + L4

    # Vertical reach
    max_z = L1 + L2 + L3 + L4
    min_z = max(L1 - L2 - L3 - L4, 0)

    return {
        'type': 'cylinder',
        'radius': max_horizontal_reach,
        'height': max_z - min_z,
        'z_min': min_z,
        'z_max': max_z,
        'center_x': 0,
        'center_y': 0
    }


def get_home_position():
    """
    Get robot positions for home configuration (all joints at 0°)

    Returns:
        List of 8 tuples [(x, y, z), ...] for home position
    """
    return compute_all_joint_positions(0, 0, 0, 0, 0, 0)


def get_joint_names():
    """
    Get descriptive names for each joint

    Returns:
        List of joint names
    """
    return [
        'Base',
        'J1 (Shoulder Rotation)',
        'J2 (Shoulder Pitch)',
        'J3 (Elbow)',
        'J4 (Wrist Roll)',
        'J5 (Wrist Pitch)',
        'J6 (Wrist Yaw)',
        'TCP'
    ]


# Example usage and testing
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    print("Thor Robot Forward Kinematics Test - CORRECTED VERSION")
    print("=" * 50)

    # Test home position
    print("\nHome Position (all joints at 0 deg):")
    home_pos = get_home_position()
    joint_names = get_joint_names()
    for name, (x, y, z) in zip(joint_names, home_pos):
        print(f"  {name:25s}: ({x:7.2f}, {y:7.2f}, {z:7.2f}) mm")

    # Test vertical position
    print("\nVertical Position (q2=90 deg):")
    vertical_pos = compute_all_joint_positions(0, 90, 0, 0, 0, 0)
    for name, (x, y, z) in zip(joint_names, vertical_pos):
        print(f"  {name:25s}: ({x:7.2f}, {y:7.2f}, {z:7.2f}) mm")

    tcp = vertical_pos[-1]
    expected_z = L1 + L2 + L3 + L4
    print(f"\nExpected Z for vertical: {expected_z:.2f} mm")
    print(f"Actual TCP Z: {tcp[2]:.2f} mm")
    if abs(tcp[2] - expected_z) < 5:
        print("SUCCESS: Vertical position is correct!")
    else:
        print(f"ERROR: Z difference is {tcp[2] - expected_z:.2f} mm")
