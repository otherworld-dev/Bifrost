"""
Simulated Hardware Module
Mock serial communication for offline robot control and testing

This module provides:
- SimulatedSerialManager: Drop-in replacement for SerialManager
- SimulatedSerialThread: Drop-in replacement for SerialThread
- Instant position updates (no interpolation)
- RepRapFirmware command compatibility
"""

import re
import time
import logging
import threading
from typing import Optional, Dict
from collections import deque

from PyQt5.QtCore import QThread, pyqtSignal, QTimer

import config

logger = logging.getLogger(__name__)


class SimulatedSerialManager:
    """
    Simulated serial manager that mimics SerialManager interface.

    Maintains internal robot state and generates RRF-compatible responses
    without requiring real hardware.
    """

    def __init__(self):
        """Initialize simulated serial manager with default robot state."""
        # Mimic SerialManager interface
        self._port = "SIMULATION"
        self._baudrate = config.SERIAL_BAUDRATE_DEFAULT
        self._timeout = config.SERIAL_TIMEOUT
        self._is_open = False

        # Thread safety (same as SerialManager)
        self.lock = threading.Lock()
        self.queue_lock = threading.Lock()
        self.command_queue = []

        # Internal robot state (degrees)
        self.positions = {
            'X': 0.0,  # Art1
            'Y': 0.0,  # Art2
            'Z': 0.0,  # Art3
            'U': 0.0,  # Art4
            'V': 0.0,  # Motor V (differential)
            'W': 0.0,  # Motor W (differential)
        }

        # Gripper state (PWM 0-255)
        self.gripper_pwm = 0

        # Endstop states
        self.endstops = {
            'X': 'not stopped',
            'Y': 'not stopped',
            'Z': 'not stopped',
            'U': 'not stopped',
            'V': 'not stopped',
            'W': 'not stopped',
        }

        # Response buffer (FIFO queue for readline())
        self.response_buffer = deque()

        # G-code parsing patterns
        self.gcode_pattern = re.compile(r'([GM])(\d+)')
        self.axis_pattern = re.compile(r'([XYZUVW])([+-]?\d+\.?\d*)')

        logger.info("SimulatedSerialManager initialized")

    def write(self, data: bytes, priority: bool = False) -> None:
        """
        Queue command for processing (non-blocking).

        Args:
            data: Command bytes
            priority: If True, add to front of queue
        """
        with self.queue_lock:
            if priority:
                self.command_queue.insert(0, data)
            else:
                self.command_queue.append(data)

    def _write_internal(self, data: bytes) -> bool:
        """
        Internal write that processes command and generates response.

        Args:
            data: Command bytes

        Returns:
            True if command processed successfully
        """
        try:
            command_str = data.decode('UTF-8', errors='replace').strip()
            logger.debug(f"Simulated command: {command_str}")

            # Parse and execute command
            response = self._parse_and_execute_command(command_str)

            # Add response(s) to buffer
            if response:
                if isinstance(response, list):
                    for resp in response:
                        self.response_buffer.append(resp.encode('UTF-8') + b'\n')
                else:
                    self.response_buffer.append(response.encode('UTF-8') + b'\n')

            return True

        except Exception as e:
            logger.error(f"Error processing simulated command: {e}")
            return False

    def _parse_and_execute_command(self, command: str) -> Optional[str]:
        """
        Parse G-code command and update internal state.

        Args:
            command: Command string (e.g., "G0 X10.5 Y20.0")

        Returns:
            Response string(s) or None
        """
        command_upper = command.upper().strip()

        # Handle empty commands
        if not command_upper:
            return None

        # Match G-code or M-code
        match = self.gcode_pattern.match(command_upper)
        if not match:
            return "ok"  # Unknown commands just get "ok"

        cmd_type = match.group(1)  # 'G' or 'M'
        cmd_num = int(match.group(2))

        # G-codes (movement)
        if cmd_type == 'G':
            if cmd_num in [0, 1]:  # G0 (rapid) or G1 (linear)
                return self._handle_movement(command_upper)
            elif cmd_num == 28:  # G28 (homing)
                return self._handle_homing()
            elif cmd_num == 29:  # G29 (bed leveling - not applicable but respond ok)
                return "ok"

        # M-codes (various)
        elif cmd_type == 'M':
            if cmd_num == 114:  # M114 (position report)
                return self._generate_m114_response()
            elif cmd_num == 119:  # M119 (endstop status)
                return self._generate_m119_response()
            elif cmd_num == 42:  # M42 (GPIO/gripper control)
                return self._handle_gripper(command_upper)
            elif cmd_num == 112:  # M112 (emergency stop)
                return self._handle_emergency_stop()
            elif cmd_num == 410:  # M410 (motor freeze)
                return self._handle_motor_freeze()
            elif cmd_num == 999:  # M999 (reset)
                return self._handle_reset()
            elif cmd_num == 115:  # M115 (firmware version)
                return "FIRMWARE_NAME: Simulated RRF FIRMWARE_VERSION: 3.5-sim"

        # Default response
        return "ok"

    def _handle_movement(self, command: str) -> str:
        """
        Handle G0/G1 movement command with instant position update.

        Args:
            command: Movement command (e.g., "G0 X10.5 Y20.0")

        Returns:
            "ok" response
        """
        # Extract axis movements
        matches = self.axis_pattern.findall(command)

        for axis, value_str in matches:
            try:
                value = float(value_str)
                if axis in self.positions:
                    self.positions[axis] = value
                    logger.debug(f"Simulated move: {axis} = {value}")
            except ValueError:
                logger.warning(f"Invalid axis value: {axis}={value_str}")

        return "ok"

    def _handle_homing(self) -> str:
        """
        Handle G28 homing command.

        Returns:
            "ok" response after setting all positions to zero
        """
        logger.info("Simulated homing: setting all axes to zero")
        for axis in self.positions:
            self.positions[axis] = 0.0

        # Update endstops to "at min stop" during homing
        for axis in self.endstops:
            self.endstops[axis] = 'at min stop'

        return "ok"

    def _handle_gripper(self, command: str) -> str:
        """
        Handle M42 gripper PWM command.

        Args:
            command: Gripper command (e.g., "M42 P5 S128")

        Returns:
            "ok" response
        """
        # Extract PWM value (S parameter)
        pwm_match = re.search(r'S(\d+)', command)
        if pwm_match:
            try:
                pwm = int(pwm_match.group(1))
                self.gripper_pwm = max(0, min(255, pwm))
                logger.debug(f"Simulated gripper: PWM = {self.gripper_pwm}")
            except ValueError:
                pass

        return "ok"

    def _handle_emergency_stop(self) -> str:
        """Handle M112 emergency stop - clear queue."""
        logger.warning("Simulated emergency stop!")
        self.clear_command_queue()
        return "ok"

    def _handle_motor_freeze(self) -> str:
        """Handle M410 motor freeze - clear queue."""
        logger.info("Simulated motor freeze")
        self.clear_command_queue()
        return "ok"

    def _handle_reset(self) -> str:
        """Handle M999 reset - clear alarms."""
        logger.info("Simulated reset")
        # Reset endstops to normal state
        for axis in self.endstops:
            self.endstops[axis] = 'not stopped'
        return "ok"

    def _generate_m114_response(self) -> str:
        """
        Generate M114 position response in RRF format.

        Returns:
            Position string (e.g., "X:10.000 Y:20.000 Z:0.000 ...")
        """
        response = " ".join([
            f"{axis}:{value:.3f}"
            for axis, value in self.positions.items()
        ])
        logger.debug(f"M114 response: {response}")
        return response

    def _generate_m119_response(self) -> str:
        """
        Generate M119 endstop response in RRF format.

        Returns:
            Endstop status string
        """
        status_parts = [f"{axis}: {status}" for axis, status in self.endstops.items()]
        response = "Endstops - " + ", ".join(status_parts)
        logger.debug(f"M119 response: {response}")
        return response

    def get_next_command(self) -> Optional[bytes]:
        """
        Get next command from queue (thread-safe).

        Returns:
            Command bytes or None if queue empty
        """
        with self.queue_lock:
            if len(self.command_queue) > 0:
                return self.command_queue.pop(0)
        return None

    def readline(self) -> bytes:
        """
        Read next response from buffer.

        Returns:
            Response bytes or empty if buffer empty
        """
        with self.lock:
            if self.response_buffer:
                return self.response_buffer.popleft()
        return b''

    def isOpen(self) -> bool:
        """Check if simulated port is open."""
        with self.lock:
            return self._is_open

    def open(self) -> None:
        """Open simulated port."""
        with self.lock:
            self._is_open = True
            logger.info("Simulated serial port opened")

    def close(self) -> None:
        """Close simulated port."""
        with self.lock:
            if self._is_open:
                self._is_open = False
                logger.info("Simulated serial port closed")

    def inWaiting(self) -> int:
        """Get number of bytes waiting in response buffer."""
        with self.lock:
            # Estimate bytes (each response ~50 chars average)
            return len(self.response_buffer) * 50

    def reset_input_buffer(self) -> None:
        """Clear response buffer."""
        with self.lock:
            self.response_buffer.clear()
            logger.debug("Simulated buffer cleared")

    def clear_command_queue(self) -> None:
        """Clear pending commands."""
        with self.queue_lock:
            self.command_queue.clear()

    def get_queue_size(self) -> int:
        """Get number of pending commands."""
        with self.queue_lock:
            return len(self.command_queue)

    @property
    def port(self) -> str:
        """Get port name (always "SIMULATION")."""
        return self._port

    @port.setter
    def port(self, value: str) -> None:
        """Set port name (ignored in simulation)."""
        self._port = "SIMULATION"

    @property
    def baudrate(self) -> int:
        """Get baudrate."""
        return self._baudrate

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        """Set baudrate (stored but not used)."""
        self._baudrate = value

    @property
    def timeout(self) -> Optional[float]:
        """Get timeout."""
        return self._timeout

    @timeout.setter
    def timeout(self, value: Optional[float]) -> None:
        """Set timeout."""
        self._timeout = value


class SimulatedSerialThread(QThread):
    """
    Simulated serial thread that mimics SerialThread interface.

    Uses QTimer for periodic polling instead of blocking time.sleep().
    Emits position updates without requiring real hardware.
    """

    serialSignal = pyqtSignal(str)

    # Blocking commands that pause status polling
    BLOCKING_COMMANDS = ['G28', 'G29', 'M999']

    def __init__(self, serial_manager=None, gui_instance=None, parent=None):
        """
        Initialize simulated serial thread.

        Args:
            serial_manager: SimulatedSerialManager instance
            gui_instance: Optional GUI instance (for compatibility)
            parent: Optional Qt parent
        """
        super().__init__(parent)

        # Get global simulated serial manager if not provided
        if serial_manager is None:
            # Import here to avoid circular dependency
            import bifrost
            serial_manager = bifrost.simulated_serial_manager

        self.serial_manager = serial_manager
        self.running = True
        self.status_polling_paused = False
        self.blocking_command_start_time = 0.0

        # Timers for periodic requests (use QTimer instead of time.time())
        self.position_timer = None
        self.endstop_timer = None

        logger.info("SimulatedSerialThread initialized")

    def stop(self) -> None:
        """Gracefully stop the thread."""
        self.running = False

    def run(self) -> None:
        """Main thread loop with command processing and periodic polling."""
        logger.info("SimulatedSerialThread started")

        # Track time for periodic requests (no QTimer - use manual timing)
        last_position_request = time.time()
        last_endstop_request = time.time()

        # Main loop: process commands and read responses
        while self.running:
            if not self.serial_manager.isOpen():
                time.sleep(0.1)
                continue

            try:
                current_time = time.time()

                # Process command queue
                command = self.serial_manager.get_next_command()
                if command:
                    success = self.serial_manager._write_internal(command)
                    if not success:
                        self.serialSignal.emit("SERIAL-DISCONNECTED")
                        break

                    # Check for blocking commands
                    command_str = command.decode('UTF-8', errors='replace').strip().upper()
                    for block_cmd in self.BLOCKING_COMMANDS:
                        if command_str.startswith(block_cmd):
                            self.status_polling_paused = True
                            self.blocking_command_start_time = current_time
                            logger.info(f"Pausing status polling for: {command_str}")
                            break

                # Send periodic position requests (if not paused)
                if not self.status_polling_paused:
                    if current_time - last_position_request >= config.SERIAL_STATUS_REQUEST_INTERVAL:
                        self.serial_manager.write(b"M114\n", priority=True)
                        last_position_request = current_time

                    if current_time - last_endstop_request >= config.SERIAL_ENDSTOP_REQUEST_INTERVAL:
                        self.serial_manager.write(b"M119\n", priority=True)
                        last_endstop_request = current_time

                # Read available responses
                bytes_available = self.serial_manager.inWaiting()
                if bytes_available > 0:
                    # Process multiple responses if available
                    for _ in range(min(10, bytes_available // 30)):
                        data_bytes = self.serial_manager.readline()
                        if data_bytes:
                            data_str = data_bytes.decode('UTF-8', errors='replace').strip()
                            if data_str:
                                self.serialSignal.emit(data_str)
                                self._check_blocking_command_complete(data_str)

                # Check for timeout on paused polling
                if self.status_polling_paused:
                    elapsed = current_time - self.blocking_command_start_time
                    if elapsed >= config.BLOCKING_COMMAND_MAX_PAUSE:
                        self.status_polling_paused = False
                        logger.warning(f"Forcing resume after {elapsed:.1f}s timeout")
                        self._request_immediate_status()

                # Small sleep to prevent busy-waiting
                time.sleep(config.SERIAL_THREAD_SLEEP)

            except Exception as e:
                logger.exception("Error in simulated serial thread")
                time.sleep(0.1)

        logger.info("SimulatedSerialThread stopped")


    def _check_blocking_command_complete(self, data: str) -> None:
        """Check if blocking command completed and resume polling."""
        if not self.status_polling_paused:
            return

        if "ok" not in data.lower():
            return

        elapsed = time.time() - self.blocking_command_start_time
        if elapsed >= config.BLOCKING_COMMAND_MIN_PAUSE:
            self.status_polling_paused = False
            logger.info(f"Resuming status polling ({elapsed:.1f}s elapsed)")
            self._request_immediate_status()

    def _request_immediate_status(self) -> None:
        """Request immediate position and endstop status."""
        self.serial_manager.write(b"M114\n", priority=True)
        self.serial_manager.write(b"M119\n", priority=True)


# Backwards compatibility alias
SerialThreadClass = SimulatedSerialThread
