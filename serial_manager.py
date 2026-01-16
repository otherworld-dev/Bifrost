"""
Serial Manager Module
Thread-safe serial communication with non-blocking command queue

This class wraps pyserial to provide:
- Thread-safe read/write operations via mutex locks
- Non-blocking command queue for outgoing messages
- Priority queue support for urgent commands (e.g., status requests)
"""

import serial
import threading
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class SerialManager:
    """
    Thread-safe serial port manager with command queueing.

    Provides non-blocking writes via a command queue that is processed
    by the serial thread. All operations are protected by mutex locks
    to ensure thread safety.
    """

    def __init__(self):
        """Initialise serial manager with command queue."""
        self.serial = serial.Serial()
        self.lock = threading.Lock()
        # Non-blocking command queue (thread-safe)
        self.command_queue = []
        self.queue_lock = threading.Lock()

    def write(self, data: bytes, priority: bool = False) -> None:
        """
        Queue data to be written by serial thread (non-blocking).

        Args:
            data: Bytes to write
            priority: If True, add to front of queue for immediate sending
        """
        with self.queue_lock:
            if priority:
                # Insert at beginning for immediate sending (status requests, etc)
                self.command_queue.insert(0, data)
            else:
                # Append to end (normal commands)
                self.command_queue.append(data)

    def _write_internal(self, data: bytes) -> bool:
        """
        Internal method: Actually write data to serial port.

        Should only be called by serial thread.

        Args:
            data: Bytes to write

        Returns:
            True if write succeeded, False otherwise
        """
        with self.lock:
            if self.serial.isOpen():
                try:
                    self.serial.write(data)
                    return True
                except (OSError, serial.SerialException) as e:
                    logger.error(f"Error writing to serial port: {e}")
                    return False
        return False

    def get_next_command(self) -> Optional[bytes]:
        """
        Get next command from queue (thread-safe, non-blocking).

        Returns:
            Command bytes or None if queue is empty
        """
        with self.queue_lock:
            if len(self.command_queue) > 0:
                return self.command_queue.pop(0)
        return None

    def readline(self) -> bytes:
        """
        Read a line from serial port (thread-safe).

        Returns:
            Bytes read, or empty bytes if port closed
        """
        with self.lock:
            if self.serial.isOpen():
                return self.serial.readline()
        return b''

    def isOpen(self) -> bool:
        """Check if serial port is open (thread-safe)."""
        with self.lock:
            return self.serial.isOpen()

    def open(self) -> None:
        """Open serial port (thread-safe)."""
        with self.lock:
            self.serial.open()

    def close(self) -> None:
        """Close serial port (thread-safe)."""
        with self.lock:
            if self.serial.isOpen():
                self.serial.close()

    def inWaiting(self) -> int:
        """Get number of bytes waiting in input buffer (thread-safe)."""
        with self.lock:
            return self.serial.inWaiting()

    def reset_input_buffer(self) -> None:
        """Clear the input buffer to prevent stale data (thread-safe)."""
        with self.lock:
            if self.serial.isOpen():
                self.serial.reset_input_buffer()

    def clear_command_queue(self) -> None:
        """Clear all pending commands from the queue."""
        with self.queue_lock:
            self.command_queue.clear()

    def get_queue_size(self) -> int:
        """Get number of commands waiting in queue."""
        with self.queue_lock:
            return len(self.command_queue)

    @property
    def port(self) -> str:
        """Get current port name."""
        return self.serial.port

    @port.setter
    def port(self, value: str) -> None:
        """Set port name."""
        self.serial.port = value

    @property
    def baudrate(self) -> int:
        """Get current baudrate."""
        return self.serial.baudrate

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        """Set baudrate."""
        self.serial.baudrate = value

    @property
    def timeout(self) -> Optional[float]:
        """Get read timeout."""
        return self.serial.timeout

    @timeout.setter
    def timeout(self, value: Optional[float]) -> None:
        """Set read timeout."""
        self.serial.timeout = value
