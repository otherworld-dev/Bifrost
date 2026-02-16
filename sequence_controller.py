"""
Sequence Controller Module
Orchestrates sequence recording and playback with GUI callbacks

This module provides:
- Sequence recording coordination
- Playback control with GUI state management
- Movement command building for sequence execution
- Callbacks for GUI updates (decoupled from Qt widgets)
"""

import logging
import csv
from typing import Dict, Optional, Callable, List, Tuple
from dataclasses import dataclass

import config
import differential_kinematics as diff_kin
from command_builder import CommandBuilder
import sequence_recorder as seq_rec

logger = logging.getLogger(__name__)


@dataclass
class JointPositions:
    """Container for joint positions"""
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    q4: float = 0.0
    q5: float = 0.0
    q6: float = 0.0
    gripper: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary"""
        return {
            'q1': self.q1, 'q2': self.q2, 'q3': self.q3,
            'q4': self.q4, 'q5': self.q5, 'q6': self.q6,
            'gripper': self.gripper
        }


@dataclass
class PlaybackState:
    """Container for playback state"""
    is_playing: bool = False
    is_paused: bool = False
    current_index: int = 0
    total_points: int = 0
    speed: float = 1.0
    loop: bool = False


class SequenceController:
    """
    Controls sequence recording and playback.

    This class coordinates between SequenceRecorder/SequencePlayer and the GUI,
    using callbacks for all GUI updates to maintain decoupling.
    """

    def __init__(
        self,
        command_sender=None,
        list_update_callback: Optional[Callable[[str], None]] = None,
        list_clear_callback: Optional[Callable[[], None]] = None,
        list_remove_callback: Optional[Callable[[int], None]] = None,
        button_state_callback: Optional[Callable[[str, bool], None]] = None,
        pause_text_callback: Optional[Callable[[str], None]] = None,
        simulation_move_callback: Optional[Callable[[float, float, float, float, float, float, float, float], None]] = None
    ):
        """
        Initialize sequence controller.

        Args:
            command_sender: Command sender for executing movements
            list_update_callback: Callback(text) to add item to sequence list
            list_clear_callback: Callback() to clear sequence list
            list_remove_callback: Callback(index) to remove item from list
            button_state_callback: Callback(button_name, enabled) to update button states
            pause_text_callback: Callback(text) to update pause button text
            simulation_move_callback: Callback(q1..q6, gripper, duration_s) for simulation mode movement
        """
        self.command_sender = command_sender

        # Callbacks
        self.list_update_callback = list_update_callback
        self.list_clear_callback = list_clear_callback
        self.list_remove_callback = list_remove_callback
        self.button_state_callback = button_state_callback
        self.pause_text_callback = pause_text_callback
        self.simulation_move_callback = simulation_move_callback

        # Internal state
        self.recorder = seq_rec.SequenceRecorder()
        self.player: Optional[seq_rec.SequencePlayer] = None
        self.playback_state = PlaybackState()
        self.simulation_mode = False

        # Movement parameters (can be updated from GUI)
        self.movement_type = "G0"
        self.feedrate = ""

    @property
    def is_playing(self) -> bool:
        """Check if sequence is currently playing"""
        return self.playback_state.is_playing

    @property
    def is_paused(self) -> bool:
        """Check if playback is paused"""
        return self.playback_state.is_paused

    @property
    def current_sequence(self) -> seq_rec.Sequence:
        """Get current sequence"""
        return self.recorder.current_sequence

    @property
    def sequence_length(self) -> int:
        """Get number of points in current sequence"""
        return len(self.recorder.current_sequence)

    def set_movement_params(self, movement_type: str, feedrate: str) -> None:
        """
        Set movement parameters for sequence execution.

        Args:
            movement_type: "G0" (rapid) or "G1" (linear)
            feedrate: Feedrate string (e.g., " F1000") or empty
        """
        self.movement_type = movement_type
        self.feedrate = feedrate

    def record_point(self, positions: JointPositions, delay: float = 1.0) -> str:
        """
        Record a point to the sequence.

        Args:
            positions: Joint positions to record
            delay: Delay before this point during playback

        Returns:
            Display text for the recorded point
        """
        # Start recording if not already
        if not self.recorder.is_recording:
            self.recorder.start_recording("Current Sequence")

        # Record the point
        self.recorder.record_point(
            positions.q1, positions.q2, positions.q3,
            positions.q4, positions.q5, positions.q6,
            positions.gripper, delay
        )

        # Generate display text
        point_num = len(self.recorder.current_sequence)
        point_text = (
            f"Point {point_num}: "
            f"q1={positions.q1:.1f}° q2={positions.q2:.1f}° q3={positions.q3:.1f}° "
            f"delay={delay:.1f}s"
        )

        # Update GUI via callback
        if self.list_update_callback:
            self.list_update_callback(point_text)

        logger.info(f"Recorded point: {point_text}")
        return point_text

    def delete_point(self, index: int) -> bool:
        """
        Delete a point from the sequence.

        Args:
            index: Index of point to delete

        Returns:
            True if deleted, False otherwise
        """
        if index < 0:
            return False

        if self.recorder.current_sequence.remove_point(index):
            # Update GUI via callback
            if self.list_remove_callback:
                self.list_remove_callback(index)

            logger.info(f"Deleted point {index + 1}")
            return True

        return False

    def clear_sequence(self) -> None:
        """Clear all points from the sequence."""
        self.recorder.current_sequence.clear()

        # Update GUI via callback
        if self.list_clear_callback:
            self.list_clear_callback()

        logger.info("Cleared all sequence points")

    def add_manual_point(self, point_data: Dict[str, float]) -> str:
        """
        Add a manually entered point to the sequence.

        Args:
            point_data: Dict with q1-q6, gripper, delay values

        Returns:
            Display text for the point
        """
        # Start recording if not already
        if not self.recorder.is_recording:
            self.recorder.start_recording("Current Sequence")

        # Create and add point
        point = seq_rec.SequencePoint(
            q1=point_data.get('q1', 0.0),
            q2=point_data.get('q2', 0.0),
            q3=point_data.get('q3', 0.0),
            q4=point_data.get('q4', 0.0),
            q5=point_data.get('q5', 0.0),
            q6=point_data.get('q6', 0.0),
            gripper=point_data.get('gripper', 0.0),
            delay=point_data.get('delay', 1.0)
        )
        self.recorder.current_sequence.add_point(point)

        # Generate display text
        point_num = len(self.recorder.current_sequence)
        point_text = format_point_display(
            point_num,
            point.q1, point.q2, point.q3,
            point.q4, point.q5, point.q6,
            point.delay
        )

        # Update GUI via callback
        if self.list_update_callback:
            self.list_update_callback(point_text)

        logger.info(f"Added manual point: {point_text}")
        return point_text

    def get_point_data(self, index: int) -> Optional[Dict[str, float]]:
        """
        Get point data for editing.

        Args:
            index: Index of point to get

        Returns:
            Dict with q1-q6, gripper, delay or None if invalid index
        """
        point = self.recorder.current_sequence.get_point(index)
        if point is None:
            return None

        return {
            'q1': point.q1,
            'q2': point.q2,
            'q3': point.q3,
            'q4': point.q4,
            'q5': point.q5,
            'q6': point.q6,
            'gripper': point.gripper,
            'delay': point.delay
        }

    def update_point(self, index: int, point_data: Dict[str, float]) -> bool:
        """
        Update an existing point.

        Args:
            index: Index of point to update
            point_data: Dict with q1-q6, gripper, delay values

        Returns:
            True if updated successfully
        """
        point = seq_rec.SequencePoint(
            q1=point_data.get('q1', 0.0),
            q2=point_data.get('q2', 0.0),
            q3=point_data.get('q3', 0.0),
            q4=point_data.get('q4', 0.0),
            q5=point_data.get('q5', 0.0),
            q6=point_data.get('q6', 0.0),
            gripper=point_data.get('gripper', 0.0),
            delay=point_data.get('delay', 1.0)
        )

        if self.recorder.current_sequence.update_point(index, point):
            # Refresh display
            self._refresh_list_display(self.recorder.current_sequence)
            logger.info(f"Updated point {index + 1}")
            return True
        return False

    def import_csv(self, filepath: str) -> Tuple[bool, str]:
        """
        Import points from a CSV file.

        Supports CSV with or without header row. Expected columns:
        q1, q2, q3, q4, q5, q6, gripper (optional), delay (optional)

        Args:
            filepath: Path to CSV file

        Returns:
            Tuple of (success, message)
        """
        try:
            with open(filepath, 'r', newline='') as f:
                # Sniff to detect if there's a header
                sample = f.read(1024)
                f.seek(0)
                has_header = csv.Sniffer().has_header(sample)

                reader = csv.reader(f)
                if has_header:
                    next(reader)  # Skip header row

                # Start recording if not already
                if not self.recorder.is_recording:
                    self.recorder.start_recording("Imported Sequence")

                count = 0
                for row in reader:
                    if len(row) < 6:
                        continue  # Skip invalid rows

                    try:
                        point_data = {
                            'q1': float(row[0]),
                            'q2': float(row[1]),
                            'q3': float(row[2]),
                            'q4': float(row[3]),
                            'q5': float(row[4]),
                            'q6': float(row[5]),
                            'gripper': float(row[6]) if len(row) > 6 else 0.0,
                            'delay': float(row[7]) if len(row) > 7 else 1.0
                        }
                        self.add_manual_point(point_data)
                        count += 1
                    except (ValueError, IndexError) as e:
                        logger.warning(f"Skipped invalid CSV row: {row} - {e}")
                        continue

                logger.info(f"Imported {count} points from {filepath}")
                return True, f"Imported {count} points"

        except Exception as e:
            logger.error(f"Failed to import CSV: {e}")
            return False, f"Import failed: {str(e)}"

    def start_playback(self, speed: float = 1.0, loop: bool = False) -> bool:
        """
        Start sequence playback.

        Args:
            speed: Playback speed multiplier
            loop: Whether to loop the sequence

        Returns:
            True if playback started, False if empty sequence
        """
        if len(self.recorder.current_sequence) == 0:
            logger.warning("Cannot play empty sequence")
            return False

        if self.playback_state.is_playing:
            logger.warning("Sequence already playing")
            return False

        # Create player with movement callback
        self.player = seq_rec.SequencePlayer(self._execute_movement)

        # Start playback
        self.player.start_playback(
            self.recorder.current_sequence,
            speed=speed,
            loop=loop
        )

        # Update state
        self.playback_state = PlaybackState(
            is_playing=True,
            is_paused=False,
            current_index=0,
            total_points=len(self.recorder.current_sequence),
            speed=speed,
            loop=loop
        )

        # Update button states via callback
        self._update_button_states(playing=True)

        logger.info(f"Started sequence playback (speed={speed}x, loop={loop})")
        return True

    def update_playback(self) -> Tuple[bool, int, int]:
        """
        Advance playback by one tick (called by timer).

        Returns:
            Tuple of (should_continue, current_index, total_points)
        """
        if not self.player:
            return (False, 0, 0)

        should_continue, current, total = self.player.playNextPoint()

        if not should_continue:
            self.stop_playback()
            logger.info("Sequence playback completed")

        return (should_continue, current, total)

    def pause_playback(self) -> None:
        """Toggle pause/resume playback."""
        if not self.player:
            return

        if self.player.is_paused:
            self.player.resume()
            self.playback_state.is_paused = False
            if self.pause_text_callback:
                self.pause_text_callback("Pause")
        else:
            self.player.pause()
            self.playback_state.is_paused = True
            if self.pause_text_callback:
                self.pause_text_callback("Resume")

    def stop_playback(self) -> None:
        """Stop sequence playback."""
        if self.player:
            self.player.stop()

        # Update state
        self.playback_state = PlaybackState()

        # Update button states via callback
        self._update_button_states(playing=False)

        logger.info("Stopped sequence playback")

    def save_sequence(self, filepath: str) -> bool:
        """
        Save current sequence to file.

        Args:
            filepath: Path to save file

        Returns:
            True if saved successfully
        """
        if len(self.recorder.current_sequence) == 0:
            logger.warning("Cannot save empty sequence")
            return False

        success = self.recorder.save_sequence(filepath)
        if success:
            logger.info(f"Saved sequence to {filepath}")
        return success

    def load_sequence(self, filepath: str) -> Optional[seq_rec.Sequence]:
        """
        Load sequence from file.

        Args:
            filepath: Path to load file

        Returns:
            Loaded sequence or None if failed
        """
        sequence = self.recorder.load_sequence(filepath)
        if sequence:
            self.recorder.set_current_sequence(sequence)
            self._refresh_list_display(sequence)
            logger.info(f"Loaded sequence '{sequence.name}' with {len(sequence)} points")
        return sequence

    def _execute_movement(
        self,
        q1: float, q2: float, q3: float,
        q4: float, q5: float, q6: float,
        gripper: float
    ) -> None:
        """
        Execute a single movement during sequence playback.

        Args:
            q1-q6: Joint angles
            gripper: Gripper position
        """
        logger.info(
            f"Executing sequence move: q1={q1:.1f}°, q2={q2:.1f}°, q3={q3:.1f}°, "
            f"q4={q4:.1f}°, q5={q5:.1f}°, q6={q6:.1f}°, grip={gripper}"
        )

        # In simulation mode, delegate to simulation callback instead of serial
        if self.simulation_mode and self.simulation_move_callback:
            # Calculate animation duration from next point's delay
            duration = 0.5  # default fallback in seconds
            if self.player and self.player.current_sequence:
                next_idx = self.player.current_point_index + 1
                seq = self.player.current_sequence
                if next_idx < len(seq):
                    duration = seq.points[next_idx].delay / self.player.speed_multiplier
            self.simulation_move_callback(q1, q2, q3, q4, q5, q6, gripper, duration)
            return

        # Calculate differential motor positions
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(q5, q6)

        # Build movement command
        command = CommandBuilder.build_axis_command(
            self.movement_type,
            {"X": q1, "Y": q2, "Z": q3, "U": q4, "V": motor_v, "W": motor_w},
            self.feedrate
        )

        # Send command (don't spam console during playback)
        if self.command_sender:
            self.command_sender.send(command, show_in_console=False)

        # Move gripper if needed
        if gripper > 0:
            self._execute_gripper_move(gripper)

    def _execute_gripper_move(self, gripper_percent: float) -> None:
        """
        Execute gripper movement.

        Args:
            gripper_percent: Gripper position 0-100%
        """
        # Convert percent to PWM then to servo angle
        pwm_value = (config.GRIPPER_PWM_MAX / config.GRIPPER_PERCENT_MAX) * gripper_percent
        servo_angle = int((pwm_value / 255.0) * 180.0)

        command = f"M280 P0 S{servo_angle}"

        if self.command_sender:
            self.command_sender.send(command, show_in_console=False)

    def _update_button_states(self, playing: bool) -> None:
        """Update button states via callback."""
        if not self.button_state_callback:
            return

        if playing:
            self.button_state_callback("play", False)
            self.button_state_callback("pause", True)
            self.button_state_callback("stop", True)
        else:
            self.button_state_callback("play", True)
            self.button_state_callback("pause", False)
            self.button_state_callback("stop", False)

        # Reset pause text
        if self.pause_text_callback:
            self.pause_text_callback("Pause")

    def _refresh_list_display(self, sequence: seq_rec.Sequence) -> None:
        """Refresh the list display with sequence points."""
        if self.list_clear_callback:
            self.list_clear_callback()

        if self.list_update_callback:
            for i, point in enumerate(sequence.points):
                point_text = (
                    f"Point {i+1}: q1={point.q1:.1f}° q2={point.q2:.1f}° "
                    f"q3={point.q3:.1f}° delay={point.delay:.1f}s"
                )
                self.list_update_callback(point_text)

    def get_sequence_info(self) -> Dict[str, any]:
        """
        Get information about current sequence.

        Returns:
            Dictionary with sequence info
        """
        seq = self.recorder.current_sequence
        return {
            'name': seq.name,
            'point_count': len(seq),
            'duration': seq.get_duration(),
            'is_recording': self.recorder.is_recording,
            'is_playing': self.playback_state.is_playing,
            'is_paused': self.playback_state.is_paused
        }


def format_point_display(
    index: int,
    q1: float, q2: float, q3: float,
    q4: float = 0, q5: float = 0, q6: float = 0,
    delay: float = 0
) -> str:
    """
    Format a sequence point for display.

    Args:
        index: 1-based point index
        q1-q6: Joint angles
        delay: Delay in seconds

    Returns:
        Formatted display string
    """
    return (
        f"Point {index}: q1={q1:.1f}° q2={q2:.1f}° q3={q3:.1f}° "
        f"q4={q4:.1f}° q5={q5:.1f}° q6={q6:.1f}° delay={delay:.1f}s"
    )
