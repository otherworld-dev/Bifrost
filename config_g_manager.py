"""
Config.g manager for reading and writing M569 motor direction commands.

Handles parsing sys/config.g to extract and update M569 drive direction
settings, providing the bridge between the calibration UI and firmware.
"""

import re
import logging
from pathlib import Path
from typing import Dict, Optional

logger = logging.getLogger(__name__)

# Joint-to-physical-drive mapping
# Art2 is coupled (drives 1+2), Art5/Art6 are differential (V=drive5, W=drive6)
JOINT_TO_DRIVES = {
    'Art1': [0],
    'Art2': [1, 2],
    'Art3': [3],
    'Art4': [4],
    'Art5': [5],
    'Art6': [6],
}

# Default config.g path
DEFAULT_CONFIG_G_PATH = Path(__file__).parent / 'sys' / 'config.g'

# Regex to match M569 lines: M569 P<drive> S<direction> [rest of line]
_M569_PATTERN = re.compile(r'^(\s*M569\s+P)(\d+)(\s+S)(\d+)(.*)', re.MULTILINE)


def read_m569_directions(config_path: Path = None) -> Dict[int, int]:
    """
    Parse all M569 lines from config.g and extract drive directions.

    Args:
        config_path: Path to config.g file. Defaults to sys/config.g.

    Returns:
        Dict mapping drive number to S value (0=forward, 1=reverse).
        e.g. {0: 0, 1: 0, 2: 0, 3: 0, 4: 1, 5: 1, 6: 1}
    """
    if config_path is None:
        config_path = DEFAULT_CONFIG_G_PATH

    directions = {}
    try:
        content = config_path.read_text(encoding='utf-8')
        for match in _M569_PATTERN.finditer(content):
            drive = int(match.group(2))
            s_value = int(match.group(4))
            directions[drive] = s_value
    except FileNotFoundError:
        logger.error(f"Config file not found: {config_path}")
    except Exception as e:
        logger.error(f"Error reading config.g: {e}")

    return directions


def write_m569_direction(config_path: Path, drive_number: int, s_value: int) -> bool:
    """
    Update the S parameter for a specific M569 drive line in config.g.

    Preserves all other parameters (T values, comments) on the line.

    Args:
        config_path: Path to config.g file.
        drive_number: Physical drive number (P parameter).
        s_value: Direction value (0=forward, 1=reverse).

    Returns:
        True if the line was found and updated, False otherwise.
    """
    try:
        content = config_path.read_text(encoding='utf-8')

        # Pattern to match the specific drive's M569 line
        pattern = re.compile(
            rf'^(\s*M569\s+P{drive_number}\s+S)(\d+)(.*)',
            re.MULTILINE
        )

        new_content, count = pattern.subn(rf'\g<1>{s_value}\3', content)

        if count == 0:
            logger.warning(f"M569 P{drive_number} not found in {config_path}")
            return False

        config_path.write_text(new_content, encoding='utf-8')
        logger.info(f"Updated config.g: M569 P{drive_number} S{s_value}")
        return True

    except Exception as e:
        logger.error(f"Error writing config.g: {e}")
        return False


def get_joint_directions(config_path: Path = None) -> Dict[str, int]:
    """
    Get motor directions for each joint by reading config.g.

    Args:
        config_path: Path to config.g file.

    Returns:
        Dict mapping joint name to direction (+1=forward, -1=reverse).
        e.g. {'Art1': 1, 'Art2': 1, 'Art3': 1, 'Art4': -1, 'Art5': -1, 'Art6': -1}
    """
    if config_path is None:
        config_path = DEFAULT_CONFIG_G_PATH

    drive_directions = read_m569_directions(config_path)
    joint_directions = {}

    for joint_name, drives in JOINT_TO_DRIVES.items():
        # Use first drive's direction as the joint direction
        first_drive = drives[0]
        s_value = drive_directions.get(first_drive, 0)
        joint_directions[joint_name] = 1 if s_value == 0 else -1

    return joint_directions


def set_joint_direction(config_path: Path, joint_name: str, direction: int) -> bool:
    """
    Set motor direction for a joint by updating M569 lines in config.g.

    For coupled joints (Art2), updates all associated drives.

    Args:
        config_path: Path to config.g file.
        joint_name: Joint name (Art1-Art6).
        direction: +1 for forward (S0), -1 for reverse (S1).

    Returns:
        True if all drives were updated successfully.
    """
    drives = JOINT_TO_DRIVES.get(joint_name)
    if drives is None:
        logger.error(f"Unknown joint: {joint_name}")
        return False

    s_value = 0 if direction == 1 else 1
    success = True

    for drive in drives:
        if not write_m569_direction(config_path, drive, s_value):
            success = False

    return success
