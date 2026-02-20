"""
Inverse Kinematics module for Thor Robot Arm

Analytical solver using kinematic decoupling: position (q1-q3) then orientation (q4-q6).
Uses FK DH parameters as single source of truth for robot geometry.

Wrist orientation extraction derived from the DH structure of links 4-5-6:
    R_3_6 = [[-c4*s5*c6 - s4*s6,  c4*s5*s6 - s4*c6,  c4*c5],
             [-s4*s5*c6 + c4*s6,  s4*s5*s6 + c4*c6,  s4*c5],
             [-c5*c6,              c5*s6,              -s5  ]]
"""

from typing import Tuple
import numpy as np
import logging
from scipy.spatial.transform import Rotation

from forward_kinematics import (
    get_dh_params, compute_all_joint_transforms, compute_tcp_transform
)
from coordinate_frames import pose_to_xyz_rpy

logger = logging.getLogger(__name__)

# Joint limits in degrees (must match GUI spinbox ranges and firmware M208)
JOINT_LIMITS = {
    1: (-97, 97),      # Art1 base rotation
    2: (-90, 90),       # Art2 shoulder
    3: (-90, 90),       # Art3 elbow
    4: (-180, 180),     # Art4 wrist roll
    5: (-90, 90),       # Art5 wrist pitch
    6: (-180, 180),     # Art6 wrist yaw
}


def check_joint_limits(q1, q2, q3, q4, q5, q6):
    """
    Check all joint angles against limits.

    Returns:
        (all_within, error_msg) - True if all joints within limits, else error string
    """
    angles = {1: q1, 2: q2, 3: q3, 4: q4, 5: q5, 6: q6}
    for joint, angle in angles.items():
        lo, hi = JOINT_LIMITS[joint]
        if angle < lo - 0.01 or angle > hi + 0.01:  # small tolerance for float
            return False, f"Joint {joint} = {angle:.1f} deg exceeds limits [{lo}, {hi}]"
    return True, ""


class IKSolution:
    """Container for inverse kinematics solution"""
    def __init__(self, q1: float = 0, q2: float = 0, q3: float = 0,
                 q4: float = 0, q5: float = 0, q6: float = 0,
                 valid: bool = True, error_msg: str = ""):
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.q5 = q5
        self.q6 = q6
        self.valid = valid
        self.error_msg = error_msg

    def __str__(self) -> str:
        if self.valid:
            return (f"IK Solution: q1={self.q1:.2f}, q2={self.q2:.2f}, "
                    f"q3={self.q3:.2f}, q4={self.q4:.2f}, "
                    f"q5={self.q5:.2f}, q6={self.q6:.2f}")
        return f"IK Solution: INVALID - {self.error_msg}"


def _get_geometry():
    """
    Extract robot geometry from DH parameters.

    Returns:
        (d1, a2, a3, d4, d6) - base height, upper arm, elbow offset, forearm, TCP offset
    """
    params = get_dh_params()
    if params is None or len(params) < 6:
        return 202.0, 160.0, -6.0, 195.0, 67.15
    return (
        params[0]['d'],   # d1: base height (202)
        params[1]['a'],   # a2: upper arm length (160)
        params[2]['a'],   # a3: elbow offset (-6)
        params[3]['d'],   # d4: forearm length (195)
        params[5]['d'],   # d6: TCP offset (67.15)
    )


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert ZYX Euler angles (radians) to 3x3 rotation matrix."""
    return Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()


def _solve_position_joints(p_wx: float, p_wy: float, p_wz: float,
                           elbow_up: bool = True) -> Tuple[float, float, float, bool, str]:
    """
    Solve q1, q2, q3 from wrist center position.

    Uses 2-link IK in the arm plane with effective forearm length
    L_eff = sqrt(d4^2 + a3^2) and angular offset delta = atan2(-a3, d4)
    to account for the a3 elbow offset.

    Args:
        p_wx, p_wy, p_wz: Wrist center position in base frame (mm)
        elbow_up: True for elbow-up solution

    Returns:
        (q1_deg, q2_deg, q3_deg, is_valid, error_msg)
    """
    d1, a2, a3, d4, _ = _get_geometry()

    # q1: base rotation from wrist center XY projection
    q1_deg = np.degrees(np.arctan2(p_wy, p_wx))

    # Arm plane distances
    r = np.sqrt(p_wx**2 + p_wy**2)   # horizontal distance from base axis
    s = p_wz - d1                     # vertical distance from shoulder
    D_sq = r**2 + s**2
    D = np.sqrt(D_sq)

    # Effective forearm: combines d4 (along z3) and a3 (along x3)
    L_eff = np.sqrt(d4**2 + a3**2)
    delta = np.arctan2(-a3, d4)       # angular offset (~1.76 deg for a3=-6)

    # Reachability
    max_reach = a2 + L_eff
    min_reach = abs(a2 - L_eff)
    TOLERANCE = 1.0

    if D > max_reach + TOLERANCE:
        return 0, 0, 0, False, f"Distance {D:.1f}mm exceeds max reach {max_reach:.1f}mm"
    if D < max(min_reach - TOLERANCE, 0):
        return 0, 0, 0, False, f"Distance {D:.1f}mm below min reach {min_reach:.1f}mm"
    D = np.clip(D, min_reach, max_reach)
    D_sq = D**2

    # q3: elbow angle via law of cosines
    # D^2 = a2^2 + L_eff^2 + 2*a2*L_eff*cos(q3 - delta)
    cos_elbow = (D_sq - a2**2 - L_eff**2) / (2 * a2 * L_eff)
    cos_elbow = np.clip(cos_elbow, -1.0, 1.0)

    if elbow_up:
        q3_rad = delta - np.arccos(cos_elbow)
    else:
        q3_rad = delta + np.arccos(cos_elbow)
    q3_deg = np.degrees(q3_rad)

    # q2: shoulder angle
    # theta2 = atan2(s, r) - atan2(L_eff*sin(q3-delta), a2 + L_eff*cos(q3-delta))
    # q2 = theta2 - 90 deg  (removing theta_offset2)
    elbow_angle = q3_rad - delta
    beta = np.arctan2(
        L_eff * np.sin(elbow_angle),
        a2 + L_eff * np.cos(elbow_angle)
    )
    alpha = np.arctan2(s, r)
    theta2 = alpha - beta
    q2_deg = np.degrees(theta2) - 90.0

    return q1_deg, q2_deg, q3_deg, True, ""


def _extract_wrist_angles(R36: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract q4, q5, q6 from the wrist rotation matrix R_3_6.

    Derived from DH links 4-5-6 (with theta_offset5=90 deg):
        R36[2][2] = -sin(q5)
        R36[0][2] = cos(q4)*cos(q5)
        R36[1][2] = sin(q4)*cos(q5)
        R36[2][0] = -cos(q5)*cos(q6)
        R36[2][1] = cos(q5)*sin(q6)

    Returns:
        (q4_deg, q5_deg, q6_deg)
    """
    r02 = R36[0, 2]
    r12 = R36[1, 2]
    r22 = R36[2, 2]
    r20 = R36[2, 0]
    r21 = R36[2, 1]

    # q5 from R36[2][2] = -sin(q5)
    sin_q5 = -r22
    cos_q5 = np.sqrt(r02**2 + r12**2)   # positive root (preferred solution)

    SINGULARITY_THRESH = 1e-6

    if cos_q5 > SINGULARITY_THRESH:
        # Normal case
        q5_deg = np.degrees(np.arctan2(sin_q5, cos_q5))
        q4_deg = np.degrees(np.arctan2(r12, r02))
        q6_deg = np.degrees(np.arctan2(r21, -r20))
    else:
        # Wrist singularity (q5 near +/-90 deg)
        q5_deg = 90.0 if sin_q5 > 0 else -90.0
        q6_deg = 0.0   # convention: assign all redundant freedom to q4

        if sin_q5 > 0:
            # q5=90: R36 degenerates, only (q4-q6) is determined
            # R36[0][0] = -cos(q4-q6), R36[0][1] = -sin(q4-q6)
            q4_deg = np.degrees(np.arctan2(-R36[0, 1], -R36[0, 0]))
        else:
            # q5=-90: only (q4+q6) is determined
            # R36[1][0] = sin(q4+q6), R36[1][1] = cos(q4+q6)
            q4_deg = np.degrees(np.arctan2(R36[1, 0], R36[1, 1]))

        logger.warning(f"IK: Wrist singularity at q5={q5_deg:.1f} deg")

    return q4_deg, q5_deg, q6_deg


def solve_ik_full(x: float, y: float, z: float,
                  roll: float = 0, pitch: float = -np.pi/2, yaw: float = 0) -> IKSolution:
    """
    Solve full 6-DOF inverse kinematics using kinematic decoupling.

    Args:
        x, y, z: Target TCP position in mm
        roll, pitch, yaw: Target TCP orientation in radians (ZYX Euler convention)
                         Default: tool pointing down (-Z direction)

    Returns:
        IKSolution with 6 joint angles in degrees, or error
    """
    _, _, _, _, d6 = _get_geometry()

    logger.info(f"IK 6-DOF: Target pos=({x:.1f}, {y:.1f}, {z:.1f}), "
                f"ori=({np.degrees(roll):.1f}, {np.degrees(pitch):.1f}, "
                f"{np.degrees(yaw):.1f}) deg")

    # Step 1: Target orientation matrix
    R_desired = euler_to_rotation_matrix(roll, pitch, yaw)

    # Step 2: Wrist center = TCP - d6 * z_tcp
    z_tcp = R_desired[:, 2]
    p_w = np.array([x, y, z]) - d6 * z_tcp

    logger.debug(f"IK 6-DOF: Wrist center at ({p_w[0]:.2f}, {p_w[1]:.2f}, {p_w[2]:.2f})")

    # Step 3: Solve position joints (q1, q2, q3)
    q1, q2, q3, valid, error = _solve_position_joints(p_w[0], p_w[1], p_w[2])

    if not valid:
        logger.error(f"IK 6-DOF: Position solve failed: {error}")
        return IKSolution(valid=False, error_msg=error)

    logger.debug(f"IK 6-DOF: Position joints q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}")

    # Step 4: Compute R_0_3 from FK (guaranteed consistent with FK chain)
    transforms = compute_all_joint_transforms(q1, q2, q3, 0, 0, 0)
    R03 = transforms[3][:3, :3]

    # Step 5: Wrist rotation matrix
    R36 = R03.T @ R_desired

    # Step 6: Extract wrist angles from R_3_6 structure
    q4, q5, q6 = _extract_wrist_angles(R36)

    # Step 7: Validate joint limits
    within, limit_err = check_joint_limits(q1, q2, q3, q4, q5, q6)
    if not within:
        logger.warning(f"IK 6-DOF: {limit_err}")
        return IKSolution(valid=False, error_msg=limit_err)

    logger.info(f"IK 6-DOF: Solution q=({q1:.2f}, {q2:.2f}, {q3:.2f}, "
                f"{q4:.2f}, {q5:.2f}, {q6:.2f})")

    return IKSolution(q1, q2, q3, q4, q5, q6, valid=True)


def solve_ik_position(x: float, y: float, z: float) -> IKSolution:
    """
    Solve 3-DOF positioning IK (q1, q2, q3).
    Positions the wrist center at the target, assuming tool points down.

    Args:
        x, y, z: Target position in mm

    Returns:
        IKSolution with q1, q2, q3 (q4-q6 = 0)
    """
    _, _, _, _, d6 = _get_geometry()

    # Tool pointing down: z_tcp = [0, 0, -1]
    # Wrist center = target - d6 * [0, 0, -1] = [x, y, z + d6]
    q1, q2, q3, valid, error = _solve_position_joints(x, y, z + d6)

    if not valid:
        return IKSolution(valid=False, error_msg=error)

    within, limit_err = check_joint_limits(q1, q2, q3, 0, 0, 0)
    if not within:
        return IKSolution(valid=False, error_msg=limit_err)

    return IKSolution(q1, q2, q3, valid=True)


def verify_ik_solution(solution: IKSolution,
                       target_x: float, target_y: float, target_z: float,
                       target_roll: float = 0, target_pitch: float = -np.pi/2,
                       target_yaw: float = 0,
                       pos_tolerance: float = 1.0,
                       ori_tolerance_deg: float = 5.0) -> Tuple[bool, float, float]:
    """
    Verify IK solution via FK round-trip.

    Args:
        solution: IKSolution to verify
        target_*: Original target pose
        pos_tolerance: Max position error in mm
        ori_tolerance_deg: Max orientation error in degrees

    Returns:
        (is_valid, pos_error_mm, ori_error_deg)
    """
    if not solution.valid:
        return False, float('inf'), float('inf')

    # FK round-trip
    T = compute_tcp_transform(solution.q1, solution.q2, solution.q3,
                              solution.q4, solution.q5, solution.q6)
    fk_x, fk_y, fk_z, fk_roll, fk_pitch, fk_yaw = pose_to_xyz_rpy(T)

    # Position error
    pos_error = np.sqrt(
        (fk_x - target_x)**2 + (fk_y - target_y)**2 + (fk_z - target_z)**2
    )

    # Orientation error (geodesic angle between rotations)
    R_target = euler_to_rotation_matrix(target_roll, target_pitch, target_yaw)
    R_actual = T[:3, :3]
    R_diff = R_target.T @ R_actual
    angle_error = np.degrees(
        np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    )

    is_valid = pos_error < pos_tolerance and angle_error < ori_tolerance_deg
    return is_valid, pos_error, angle_error


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    d1, a2, a3, d4, d6 = _get_geometry()
    L_eff = np.sqrt(d4**2 + a3**2)

    print("Thor Robot Inverse Kinematics Test")
    print("=" * 60)
    print(f"Geometry: d1={d1}, a2={a2}, a3={a3}, d4={d4}, d6={d6}")
    print(f"Effective forearm: L_eff={L_eff:.3f}mm")
    print(f"Max reach: {a2 + L_eff:.1f}mm, Min reach: {abs(a2 - L_eff):.1f}mm")
    print()

    # Test: FK round-trip at multiple configurations
    print("FK Round-Trip Tests")
    print("-" * 60)

    test_configs = [
        (0, 0, 0, 0, 0, 0, "Home position"),
        (30, -20, 15, 10, -5, 20, "Arbitrary pose 1"),
        (-45, 10, -30, 25, 15, -10, "Arbitrary pose 2"),
        (0, -45, 45, 0, 0, 0, "Elbow bent, wrist straight"),
        (90, 0, 0, 0, 0, 0, "Base rotated 90 deg"),
        (0, 0, 0, 0, 45, 0, "Wrist pitched 45 deg"),
        (0, 0, 0, 45, 0, 30, "Wrist roll+yaw"),
        (0, -30, 20, 0, 30, 0, "Mid-range all"),
        (-60, 20, -20, -30, 10, 45, "Arbitrary pose 3"),
    ]

    pass_count = 0
    for q1, q2, q3, q4, q5, q6, name in test_configs:
        # FK: compute TCP from joint angles
        T = compute_tcp_transform(q1, q2, q3, q4, q5, q6)
        x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(T)

        # IK: recover joint angles from TCP pose
        sol = solve_ik_full(x, y, z, roll, pitch, yaw)

        if not sol.valid:
            print(f"  FAIL [{name}]: IK failed - {sol.error_msg}")
            continue

        # Verify via FK round-trip
        ok, pos_err, ori_err = verify_ik_solution(
            sol, x, y, z, roll, pitch, yaw, pos_tolerance=0.5, ori_tolerance_deg=1.0
        )

        status = "PASS" if ok else "FAIL"
        if ok:
            pass_count += 1
        print(f"  {status} [{name}]: pos_err={pos_err:.4f}mm, ori_err={ori_err:.4f} deg")

        if not ok:
            print(f"        Input:  q=({q1}, {q2}, {q3}, {q4}, {q5}, {q6})")
            print(f"        Output: q=({sol.q1:.2f}, {sol.q2:.2f}, {sol.q3:.2f}, "
                  f"{sol.q4:.2f}, {sol.q5:.2f}, {sol.q6:.2f})")

    print(f"\n{pass_count}/{len(test_configs)} tests passed")
