import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from gui import Ui_MainWindow
from about import Ui_Dialog as About_Ui_Dialog


import serial_port_finder as spf
import inverse_kinematics as ik
import sequence_recorder as seq_rec
import differential_kinematics as diff_kin
import position_history as pos_hist
import config
import parsing_patterns
from command_builder import CommandBuilder, SerialCommandSender
from robot_controller import RobotController

import serial
import time
import json
import threading
import logging
import numpy as np

# Configure logging for debugging using config
# Only log to file, not to console
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format=config.LOG_FORMAT,
    handlers=[
        logging.FileHandler(config.LOG_FILE)
    ]
)
logger = logging.getLogger(__name__)

# This application is designed for RepRapFirmware (RRF) only
# All parsing patterns are now in parsing_patterns.py module

# Thread-safe serial object with command queue
class SerialManager:
    def __init__(self):
        self.serial = serial.Serial()
        self.lock = threading.Lock()
        # Non-blocking command queue (thread-safe)
        self.command_queue = []
        self.queue_lock = threading.Lock()

    def write(self, data, priority=False):
        """
        Queue data to be written by serial thread (non-blocking)

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

    def _write_internal(self, data):
        """
        Internal method: Actually write data to serial port (called by serial thread only)

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

    def get_next_command(self):
        """
        Get next command from queue (thread-safe, non-blocking)

        Returns:
            Command bytes or None if queue is empty
        """
        with self.queue_lock:
            if len(self.command_queue) > 0:
                return self.command_queue.pop(0)
        return None

    def readline(self):
        with self.lock:
            if self.serial.isOpen():
                return self.serial.readline()
        return b''

    def isOpen(self):
        with self.lock:
            return self.serial.isOpen()

    def open(self):
        with self.lock:
            self.serial.open()

    def close(self):
        with self.lock:
            if self.serial.isOpen():
                self.serial.close()

    def inWaiting(self):
        with self.lock:
            return self.serial.inWaiting()

    def reset_input_buffer(self):
        """Clear the input buffer to prevent stale data"""
        with self.lock:
            if self.serial.isOpen():
                self.serial.reset_input_buffer()

s0 = SerialManager()

class HistoryLineEdit(QtWidgets.QLineEdit):
    """QLineEdit with command history navigation via up/down arrows"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = []
        self.history_position = -1
        self.current_text = ""

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Up:
            # Navigate up through history (older commands)
            if len(self.history) > 0:
                if self.history_position == -1:
                    # Save current text before navigating history
                    self.current_text = self.text()
                    self.history_position = len(self.history) - 1
                elif self.history_position > 0:
                    self.history_position -= 1

                if 0 <= self.history_position < len(self.history):
                    self.setText(self.history[self.history_position])

        elif event.key() == QtCore.Qt.Key_Down:
            # Navigate down through history (newer commands)
            if self.history_position != -1:
                if self.history_position < len(self.history) - 1:
                    self.history_position += 1
                    self.setText(self.history[self.history_position])
                else:
                    # Back to current text
                    self.history_position = -1
                    self.setText(self.current_text)

        else:
            # For any other key, reset history position
            if event.key() not in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down):
                if self.history_position != -1:
                    self.history_position = -1
            super().keyPressEvent(event)

    def addToHistory(self, command):
        """Add a command to history"""
        if command.strip():  # Don't add empty commands
            # Don't add duplicate consecutive commands
            if not self.history or self.history[-1] != command:
                self.history.append(command)
                # Limit history to prevent memory leak in long sessions
                if len(self.history) > 100:
                    self.history.pop(0)
            self.history_position = -1
            self.current_text = ""

class AboutDialog(About_Ui_Dialog):
    def __init__(self, dialog):
        About_Ui_Dialog.__init__(self)
        self.setupUi(dialog)

class ConnectionSignals(QtCore.QObject):
    """Signals for thread-safe connection callbacks"""
    success = pyqtSignal(str, str)  # serialPort, baudrate
    error = pyqtSignal(str)  # error_msg

class BifrostGUI(Ui_MainWindow):
    def __init__(self, dialog):
        Ui_MainWindow.__init__(self)
        self.setupUi(dialog)

        # Create connection signals (will connect after methods are defined)
        self.connection_signals = ConnectionSignals()

        # Replace ConsoleInput with HistoryLineEdit for command history
        old_console_input = self.ConsoleInput
        self.ConsoleInput = HistoryLineEdit(self.centralwidget)
        self.ConsoleInput.setGeometry(old_console_input.geometry())
        self.ConsoleInput.setObjectName("ConsoleInput")
        old_console_input.setParent(None)
        old_console_input.deleteLater()

        # Using RepRapFirmware (RRF) - this is the only supported firmware

        # Track serial thread state
        self.SerialThreadClass = None

        # Track last manual command time to show responses
        self.last_manual_command_time = 0

        # Track homing state
        self.is_homing = False

        # Track jog mode state
        self.jog_mode_enabled = False

        # Initialize command sender (will use ConsoleOutput widget)
        self.command_sender = SerialCommandSender(s0, None)  # Console widget set later after full init

        # Initialize robot controller
        self.robot_controller = RobotController()
        logger.info("RobotController initialized and integrated with GUI")

        self.getSerialPorts()

        self.actionAbout.triggered.connect(self.launchAboutWindow)
        self.actionExit.triggered.connect(self.close_application)

        # Setup embedded position history graph
        self.setupPositionHistoryControls()

        self.HomeButton.pressed.connect(self.sendHomingCycleCommand)
        self.ZeroPositionButton.pressed.connect(self.sendZeroPositionCommand)
        self.KillAlarmLockButton.pressed.connect(self.sendKillAlarmCommand)

        self.G0MoveRadioButton.clicked.connect(self.FeedRateBoxHide)
        self.G1MoveRadioButton.clicked.connect(self.FeedRateBoxHide)
        self.JogModeCheckBox.toggled.connect(self.toggleJogMode)

        # Dynamic FK control connections (80% code reduction via loops)
        # Replace 54 individual signal connections with generic handlers
        for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
            # Get widgets
            go_button = getattr(self, f'FKGoButton{joint}')
            slider = getattr(self, f'FKSlider{joint}')
            spinbox = getattr(self, f'SpinBox{joint}')
            dec10_btn = getattr(self, f'FKDec10Button{joint}')
            dec1_btn = getattr(self, f'FKDec1Button{joint}')
            dec01_btn = getattr(self, f'FKDec0_1Button{joint}')
            inc01_btn = getattr(self, f'FKInc0_1Button{joint}')
            inc1_btn = getattr(self, f'FKInc1Button{joint}')
            inc10_btn = getattr(self, f'FKInc10Button{joint}')

            # Connect signals with lambda closures
            go_button.pressed.connect(lambda j=joint: self.FKMoveJoint(j))
            slider.valueChanged.connect(lambda val, j=joint: self.FKSliderUpdate(j, val))
            spinbox.valueChanged.connect(lambda val, j=joint: self.FKSpinBoxUpdate(j, val))
            dec10_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, -10))
            dec1_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, -1))
            dec01_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, -0.1))
            inc01_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, 0.1))
            inc1_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, 1))
            inc10_btn.pressed.connect(lambda j=joint: self.adjustJointValue(j, 10))

        self.FKGoAllButton.pressed.connect(self.FKMoveAll)

        self.GoButtonGripper.pressed.connect(self.MoveGripper)
        self.SliderGripper.valueChanged.connect(self.SliderUpdateGripper)
        self.SpinBoxGripper.valueChanged.connect(self.SpinBoxUpdateGripper)
        self.Dec10ButtonGripper.pressed.connect(self.Dec10Gripper)
        self.Dec1ButtonGripper.pressed.connect(self.Dec1Gripper)
        self.Inc1ButtonGripper.pressed.connect(self.Inc1Gripper)
        self.Inc10ButtonGripper.pressed.connect(self.Inc10Gripper)

        self.SerialPortRefreshButton.pressed.connect(self.getSerialPorts)
        self.ConnectButton.pressed.connect(self.connectSerial)

        # Connect console input signals after replacing with HistoryLineEdit
        self.ConsoleButtonSend.pressed.connect(self.sendSerialCommand)
        self.ConsoleInput.returnPressed.connect(self.sendSerialCommand)

        # IK Control connections with debounce
        # Use timer to batch rapid spinbox changes for smoother GUI
        self.ik_calc_timer = QtCore.QTimer()
        self.ik_calc_timer.setSingleShot(True)
        self.ik_calc_timer.timeout.connect(self._calculateIKDeferred)

        self.IKInputSpinBoxX.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxY.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxZ.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        # IK increment/decrement buttons using generic method
        self.IkIncButtonX.pressed.connect(lambda: self.adjustIKValue('X', 10))
        self.IkDecButtonX.pressed.connect(lambda: self.adjustIKValue('X', -10))
        self.IkIncButtonY.pressed.connect(lambda: self.adjustIKValue('Y', 10))
        self.IkDecButtonY.pressed.connect(lambda: self.adjustIKValue('Y', -10))
        self.IkIncButtonZ.pressed.connect(lambda: self.adjustIKValue('Z', 10))
        self.IkDecButtonZ.pressed.connect(lambda: self.adjustIKValue('Z', -10))

        # Enable IK controls
        self.InverseKinematicsLabel.setEnabled(True)
        self.IKInputSpinBoxX.setEnabled(True)
        self.IKInputSpinBoxY.setEnabled(True)
        self.IKInputSpinBoxZ.setEnabled(True)
        self.IkOutputValueFrame.setEnabled(True)
        self.IkIncButtonX.setEnabled(True)
        self.IkDecButtonX.setEnabled(True)
        self.IkIncButtonY.setEnabled(True)
        self.IkDecButtonY.setEnabled(True)
        self.IkIncButtonZ.setEnabled(True)
        self.IkDecButtonZ.setEnabled(True)

        # Initialize Sequence Recorder
        self.sequence_recorder = seq_rec.SequenceRecorder()
        self.sequence_player = None  # Will be initialized when needed
        self.is_playing_sequence = False
        self.sequence_timer = QtCore.QTimer()
        self.sequence_timer.timeout.connect(self.updateSequencePlayback)
        self.setupSequenceControls()

        # NOTE: Differential motor tracking, desired positions, and position validation
        # are now handled by RobotController. Local references kept for backward compatibility
        # and will be updated from controller as needed.
        self.current_motor_v = 0.0  # Updated from robot_controller
        self.current_motor_w = 0.0  # Updated from robot_controller
        self.desired_art5 = 0.0  # Updated from robot_controller
        self.desired_art6 = 0.0  # Updated from robot_controller
        self.position_update_count = 0  # Updated from robot_controller
        self.last_gui_update_time = 0  # For throttling GUI updates

        # Position history tracking
        self.position_history = pos_hist.PositionHistory(max_size=config.POSITION_HISTORY_MAX_SIZE)
        logger.info(f"Position history initialized (max_size={config.POSITION_HISTORY_MAX_SIZE}, sample_rate=1/{config.POSITION_HISTORY_SAMPLE_RATE})")

        # Setup endstop status displays
        self.setupEndstopDisplays()

        # Setup generic increment/decrement connections
        self.setupGenericControls()

        # Connect connection signals now that methods are defined
        self.connection_signals.success.connect(self._onConnectionSuccess)
        self.connection_signals.error.connect(self._onConnectionError)

        # Set console widget for command sender now that UI is fully initialized
        self.command_sender.console_output = self.ConsoleOutput

    def setupGenericControls(self):
        """
        Setup generic increment/decrement methods for all joints
        This replaces 36 individual methods with dynamic binding
        """
        # Map joint names to their spinboxes
        self.joint_spinboxes = {
            'Art1': self.SpinBoxArt1,
            'Art2': self.SpinBoxArt2,
            'Art3': self.SpinBoxArt3,
            'Art4': self.SpinBoxArt4,
            'Art5': self.SpinBoxArt5,
            'Art6': self.SpinBoxArt6,
            'Gripper': self.SpinBoxGripper
        }

        # NOTE: Joint configuration is now managed by RobotController
        # This reference is kept for backward compatibility with existing GUI methods
        self.joint_config = self.robot_controller.joint_config

        logger.info("Generic increment/decrement controls initialized (using RobotController config)")

    def adjustJointValue(self, joint_name, delta):
        """
        Generic method to adjust any joint value by a delta

        Args:
            joint_name: Name of joint ('Art1', 'Art2', etc.)
            delta: Amount to add to current value
        """
        if joint_name in self.joint_spinboxes:
            spinbox = self.joint_spinboxes[joint_name]
            new_value = spinbox.value() + delta
            spinbox.setValue(new_value)

            # JOG MODE: If jog mode is enabled, execute movement immediately
            if self.jog_mode_enabled:
                if joint_name == 'Gripper':
                    self.MoveGripper()
                    logger.debug(f"[JOG MODE] Gripper moved to {new_value}%")
                else:
                    self.FKMoveJoint(joint_name)
                    logger.debug(f"[JOG MODE] {joint_name} jogged to {new_value}°")
        else:
            logger.warning(f"Unknown joint name: {joint_name}")

    def FKSliderUpdate(self, joint_name, value):
        """Generic slider update handler - updates spinbox from slider"""
        if joint_name in self.joint_spinboxes:
            val = value / 10.0
            self.joint_spinboxes[joint_name].setValue(val)

    def FKSpinBoxUpdate(self, joint_name, value):
        """Generic spinbox update handler - updates slider from spinbox"""
        slider = getattr(self, f'FKSlider{joint_name}', None)
        if slider:
            val = int(value * 10)
            slider.setValue(val)

    def FKMoveJoint(self, joint_name):
        """
        Generic joint movement handler
        Handles simple joints, coupled motors, and differential kinematics
        """
        if joint_name not in self.joint_config:
            logger.warning(f"Unknown joint: {joint_name}")
            return

        config = self.joint_config[joint_name]
        joint_value = self.joint_spinboxes[joint_name].value()

        # Handle based on joint type
        if config['type'] in ('simple', 'coupled'):
            # Simple and coupled use same logic (just different logging)
            self._FKMoveSimple(joint_name, joint_value, config)
        elif config['type'] == 'differential':
            self._FKMoveDifferential(joint_name, joint_value, config)

    def _FKMoveSimple(self, joint_name, joint_value, config):
        """Move a simple single-axis joint (also handles coupled motors)"""
        # Log with appropriate detail based on joint type
        if config['type'] == 'coupled':
            logger.info(f"{config['log_name']} commanded to: {joint_value}° -> Axis: {config['axis']} (Drives {config['drives']})")
        else:
            logger.info(f"{config['log_name']} commanded to: {joint_value}° -> Axis: {config['axis']}")

        # Build and send command (identical for simple and coupled)
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        command = CommandBuilder.build_single_axis_command(
            movement_type, config['axis'], joint_value, feedrate
        )

        # Send command
        if not self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection):
            logger.warning(f"{joint_name} move attempted but serial not connected")

    def _FKMoveDifferential(self, joint_name, joint_value, config):
        """
        Move a differential joint (Art5/Art6)
        NOW USES: RobotController for differential calculations
        """
        # Check if we have valid position feedback using RobotController
        if not self.robot_controller.check_differential_initialized():
            logger.warning("No position feedback received yet - differential control may be inaccurate!")
            logger.warning("Wait for position update or home the robot first")

        # Calculate new motor positions using RobotController
        motor_v, motor_w, kept_value = self.robot_controller.calculate_differential_move(
            joint_name, joint_value
        )

        # Update tracked positions in controller
        self.robot_controller.update_differential_motors(motor_v, motor_w)

        # Keep local references for backward compatibility
        self.current_motor_v = motor_v
        self.current_motor_w = motor_w
        if joint_name == 'Art5':
            self.desired_art5 = joint_value
        else:
            self.desired_art6 = joint_value

        # Build command using command builder (both V and W motors)
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        command = CommandBuilder.build_axis_command(
            movement_type,
            {"V": motor_v, "W": motor_w},
            feedrate
        )

        # Send command
        if not self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection):
            logger.warning(f"{joint_name} move attempted but serial not connected")

    def close_application(self):
        # Properly cleanup serial connection and thread
        if self.SerialThreadClass and self.SerialThreadClass.isRunning():
            self.SerialThreadClass.stop()
            self.SerialThreadClass.wait(config.SERIAL_THREAD_SHUTDOWN_TIMEOUT)
        s0.close()
        sys.exit()

    def launchAboutWindow(self):
        self.dialogAbout = QtWidgets.QDialog()
        self.ui = AboutDialog(self.dialogAbout)
        self.dialogAbout.exec_()

    def sendHomingCycleCommand(self):
        """Send G28 homing command (RRF: Home all axes)"""
        if self.command_sender.send_if_connected("G28"):
            # Update button state to indicate homing in progress
            self.is_homing = True
            self.HomeButton.setEnabled(False)
            self.HomeButton.setText("Homing...")

    def sendZeroPositionCommand(self):
        """Send command to move all axes to zero position"""
        command = CommandBuilder.build_axis_command(
            "G0",
            {"X": 0, "Y": 0, "Z": 0, "U": 0, "V": 0, "W": 0}
        )
        self.command_sender.send_if_connected(command)

    def sendKillAlarmCommand(self):
        """Send M999 command (RRF: Clear emergency stop / reset)"""
        self.command_sender.send_if_connected("M999")

    def FeedRateBoxHide(self):
        if self.G1MoveRadioButton.isChecked():
            self.FeedRateLabel.setEnabled(True)
            self.FeedRateInput.setEnabled(True)
        else:
            self.FeedRateLabel.setEnabled(False)
            self.FeedRateInput.setEnabled(False)

    def toggleJogMode(self, enabled):
        """
        Enable/disable jog mode with confirmation and visual feedback

        Args:
            enabled: True to enable jog mode, False to disable
        """
        # If enabling, show confirmation warning
        if enabled and config.JOG_MODE_WARNING_ENABLED:
            if not self._showJogModeWarning():
                # User cancelled, uncheck the checkbox
                self.JogModeCheckBox.setChecked(False)
                return

        # Update state
        self.jog_mode_enabled = enabled

        # Update visual feedback
        self._updateJogModeVisuals(enabled)

        # Log state change
        if enabled:
            logger.warning("JOG MODE ENABLED - Inc/Dec buttons will execute movements immediately!")
        else:
            logger.info("Jog mode disabled - Normal operation resumed")

    def _showJogModeWarning(self):
        """
        Show confirmation dialog when enabling jog mode

        Returns:
            True if user confirmed, False if cancelled
        """
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setWindowTitle("Enable Jog Mode")
        msgBox.setText("⚠ Enable Jog Mode?\n\n"
                      "When Jog Mode is active:\n"
                      "• Inc/Dec buttons will IMMEDIATELY execute movements\n"
                      "• No need to click 'Go' buttons\n"
                      "• Works for all FK joints (Art1-6, Gripper) and IK controls\n"
                      "• Uses current G0/G1 and feedrate settings\n\n"
                      "Make sure the robot workspace is clear before proceeding.")
        msgBox.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msgBox.setDefaultButton(QtWidgets.QMessageBox.No)

        result = msgBox.exec_()
        return result == QtWidgets.QMessageBox.Yes

    def _updateJogModeVisuals(self, enabled):
        """
        Update visual feedback for jog mode state

        Args:
            enabled: True if jog mode is enabled, False otherwise
        """
        if enabled:
            # Highlight jog mode checkbox with warning color
            self.JogModeCheckBox.setStyleSheet(
                f"color: rgb(200, 80, 0); background-color: {config.JOG_MODE_VISUAL_HIGHLIGHT}; "
                "padding: 3px; border-radius: 3px; font-weight: bold;"
            )

            # Disable/dim all "Go" buttons (they're not needed in jog mode)
            for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
                go_button = getattr(self, f'FKGoButton{joint}')
                go_button.setEnabled(False)
                go_button.setStyleSheet("background-color: rgb(200, 200, 200);")

            # Disable "Go All" button
            self.FKGoAllButton.setEnabled(False)
            self.FKGoAllButton.setStyleSheet("background-color: rgb(200, 200, 200);")

            # Disable gripper go button
            self.GoButtonGripper.setEnabled(False)
            self.GoButtonGripper.setStyleSheet("background-color: rgb(200, 200, 200);")

        else:
            # Restore normal checkbox appearance
            self.JogModeCheckBox.setStyleSheet("color: rgb(200, 80, 0); font-weight: bold;")

            # Re-enable all "Go" buttons
            for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
                go_button = getattr(self, f'FKGoButton{joint}')
                go_button.setEnabled(True)
                go_button.setStyleSheet("")

            # Re-enable "Go All" button
            self.FKGoAllButton.setEnabled(True)
            self.FKGoAllButton.setStyleSheet("")

            # Re-enable gripper go button
            self.GoButtonGripper.setEnabled(True)
            self.GoButtonGripper.setStyleSheet("")


# OLD FK methods removed - replaced with generic handlers above
    # (FKMoveArt1-6, FKSliderUpdateArt1-6, FKSpinBoxUpdateArt1-6, FKDec/Inc methods)
    # Saved ~290 lines of duplicate code via dynamic signal binding

#FK Every Articulation Functions
    def FKMoveAll(self):
        """Move all joints simultaneously (DIFFERENTIAL MECHANISM: Art5 & Art6 use differential kinematics)"""
        # Get joint values
        art1 = self.SpinBoxArt1.value()
        art2 = self.SpinBoxArt2.value()
        art3 = self.SpinBoxArt3.value()
        art4 = self.SpinBoxArt4.value()
        art5 = self.SpinBoxArt5.value()
        art6 = self.SpinBoxArt6.value()

        # Calculate differential motor positions using helper
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(art5, art6)

        logger.info(f"MoveAll: Art5={art5}° Art6={art6}° -> Differential: V={motor_v:.2f}° W={motor_w:.2f}°")

        # Build command using command builder
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        # Map all axes: X=Art1, Y=Art2(coupled), Z=Art3, U=Art4, V/W=differential(Art5,Art6)
        command = CommandBuilder.build_axis_command(
            movement_type,
            {"X": art1, "Y": art2, "Z": art3, "U": art4, "V": motor_v, "W": motor_w},
            feedrate
        )

        # Send command
        self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection)

# Gripper Functions
    @staticmethod
    def _gripper_percent_to_pwm(percent):
        """Convert gripper percentage (0-100) to PWM value (0-255)"""
        return (config.GRIPPER_PWM_MAX / config.GRIPPER_PERCENT_MAX) * percent

    def MoveGripper(self):
        """Move gripper to specified position"""
        pwm_value = self._gripper_percent_to_pwm(self.SpinBoxGripper.value())
        # Map 0-255 PWM range to 0-180 servo angle for RepRapFirmware M280 command
        servo_angle = int((pwm_value / 255.0) * 180.0)
        command = f"M280 P0 S{servo_angle}"

        # Send command
        self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection)

    def SliderUpdateGripper(self):
        val=self.SliderGripper.value()
        self.SpinBoxGripper.setValue(val)
    def SpinBoxUpdateGripper(self):
        val=int(self.SpinBoxGripper.value())
        self.SliderGripper.setValue(val)
    def Dec10Gripper(self):
        self.adjustJointValue('Gripper', -10)
    def Dec1Gripper(self):
        self.adjustJointValue('Gripper', -1)
    def Inc1Gripper(self):
        self.adjustJointValue('Gripper', 1)
    def Inc10Gripper(self):
        self.adjustJointValue('Gripper', 10)

# Inverse Kinematics Functions
    def _calculateIKDeferred(self):
        """Calculate 6-DOF inverse kinematics for current target position (debounced)"""
        x = self.IKInputSpinBoxX.value()
        y = self.IKInputSpinBoxY.value()
        z = self.IKInputSpinBoxZ.value()

        logger.info(f"IK 6-DOF: Calculating for target X={x}, Y={y}, Z={z}")

        # Solve full 6-DOF IK with default tool-down orientation
        # Future enhancement: add orientation input controls
        solution = ik.solve_ik_full(x, y, z, roll=0, pitch=-np.pi/2, yaw=0)

        # Update output displays
        if solution.valid:
            self.IkOutputValueX.setText(f"{solution.q1:.2f}º")
            self.IkOutputValueY.setText(f"{solution.q2:.2f}º")
            self.IkOutputValueZ.setText(f"{solution.q3:.2f}º")

            # Update FK spinboxes with all 6 calculated joint angles
            self.SpinBoxArt1.setValue(solution.q1)
            self.SpinBoxArt2.setValue(solution.q2)
            self.SpinBoxArt3.setValue(solution.q3)
            self.SpinBoxArt4.setValue(solution.q4)
            self.SpinBoxArt5.setValue(solution.q5)
            self.SpinBoxArt6.setValue(solution.q6)

            # Style valid solution
            self.IkOutputValueFrame.setStyleSheet("background-color:rgb(200, 255, 200)")  # Light green
            logger.info(f"IK 6-DOF: Valid solution - q1={solution.q1:.2f}°, q2={solution.q2:.2f}°, q3={solution.q3:.2f}°, q4={solution.q4:.2f}°, q5={solution.q5:.2f}°, q6={solution.q6:.2f}°")
        else:
            self.IkOutputValueX.setText("--")
            self.IkOutputValueY.setText("--")
            self.IkOutputValueZ.setText("--")

            # Style invalid solution
            self.IkOutputValueFrame.setStyleSheet("background-color:rgb(255, 200, 200)")  # Light red
            logger.warning(f"IK 6-DOF: Invalid solution - {solution.error_msg}")

    def adjustIKValue(self, axis, delta):
        """
        Generic method to adjust IK input value by a delta

        Args:
            axis: Axis letter ('X', 'Y', 'Z')
            delta: Amount to add to current value
        """
        spinbox = getattr(self, f'IKInputSpinBox{axis}')
        new_value = spinbox.value() + delta
        spinbox.setValue(new_value)

        # JOG MODE: If jog mode is enabled, execute movement immediately after IK calculation
        # The IK calculation will be triggered by the spinbox value change (debounced)
        # Once IK updates the FK spinboxes, we execute FKMoveAll to move all joints
        if self.jog_mode_enabled:
            # Wait briefly for IK calculation to complete (debounce timer), then execute
            # Use a single-shot timer to execute after IK calculation
            QtCore.QTimer.singleShot(100, self._executeIKJogMove)
            logger.debug(f"[JOG MODE] IK {axis} jogged to {new_value}mm, executing move after IK calculation")

    def _executeIKJogMove(self):
        """Execute movement after IK calculation in jog mode (called by timer)"""
        if self.jog_mode_enabled:
            # Check if IK solution is valid (green background)
            if "200, 255, 200" in self.IkOutputValueFrame.styleSheet():
                self.FKMoveAll()
                logger.debug("[JOG MODE] IK solution valid, executing FKMoveAll")
            else:
                logger.warning("[JOG MODE] IK solution invalid, movement not executed")

# Sequence Recorder Functions
    def setupSequenceControls(self):
        """Create sequence recorder GUI controls programmatically"""
        # Create group box for sequence controls
        self.sequenceGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.sequenceGroupBox.setGeometry(QtCore.QRect(910, 20, 280, 838))
        self.sequenceGroupBox.setTitle("Sequence Programmer")

        # List widget for sequence points (expanded to fill space, aligned with console send button)
        self.sequencePointsList = QtWidgets.QListWidget(self.sequenceGroupBox)
        self.sequencePointsList.setGeometry(QtCore.QRect(10, 25, 260, 555))

        # Record controls
        self.sequenceRecordButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceRecordButton.setGeometry(QtCore.QRect(10, 590, 120, 30))
        self.sequenceRecordButton.setText("Record Point")
        self.sequenceRecordButton.pressed.connect(self.recordSequencePoint)

        self.sequenceDeleteButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceDeleteButton.setGeometry(QtCore.QRect(150, 590, 120, 30))
        self.sequenceDeleteButton.setText("Delete Point")
        self.sequenceDeleteButton.pressed.connect(self.deleteSequencePoint)

        self.sequenceClearButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceClearButton.setGeometry(QtCore.QRect(10, 625, 260, 30))
        self.sequenceClearButton.setText("Clear All")
        self.sequenceClearButton.pressed.connect(self.clearSequence)

        # Playback controls
        self.sequencePlayButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePlayButton.setGeometry(QtCore.QRect(10, 665, 80, 30))
        self.sequencePlayButton.setText("Play")
        self.sequencePlayButton.pressed.connect(self.playSequence)

        self.sequencePauseButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePauseButton.setGeometry(QtCore.QRect(100, 665, 80, 30))
        self.sequencePauseButton.setText("Pause")
        self.sequencePauseButton.setEnabled(False)
        self.sequencePauseButton.pressed.connect(self.pauseSequence)

        self.sequenceStopButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceStopButton.setGeometry(QtCore.QRect(190, 665, 80, 30))
        self.sequenceStopButton.setText("Stop")
        self.sequenceStopButton.setEnabled(False)
        self.sequenceStopButton.pressed.connect(self.stopSequence)

        # Speed control
        speedLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        speedLabel.setGeometry(QtCore.QRect(10, 705, 50, 20))
        speedLabel.setText("Speed:")

        self.sequenceSpeedSpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceSpeedSpinBox.setGeometry(QtCore.QRect(60, 705, 80, 22))
        self.sequenceSpeedSpinBox.setMinimum(0.1)
        self.sequenceSpeedSpinBox.setMaximum(10.0)
        self.sequenceSpeedSpinBox.setSingleStep(0.1)
        self.sequenceSpeedSpinBox.setValue(1.0)
        self.sequenceSpeedSpinBox.setSuffix("x")

        self.sequenceLoopCheckBox = QtWidgets.QCheckBox(self.sequenceGroupBox)
        self.sequenceLoopCheckBox.setGeometry(QtCore.QRect(150, 705, 60, 20))
        self.sequenceLoopCheckBox.setText("Loop")

        # Delay control
        delayLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        delayLabel.setGeometry(QtCore.QRect(10, 735, 50, 20))
        delayLabel.setText("Delay:")

        self.sequenceDelaySpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceDelaySpinBox.setGeometry(QtCore.QRect(60, 735, 80, 22))
        self.sequenceDelaySpinBox.setMinimum(0.0)
        self.sequenceDelaySpinBox.setMaximum(60.0)
        self.sequenceDelaySpinBox.setSingleStep(0.1)
        self.sequenceDelaySpinBox.setValue(1.0)
        self.sequenceDelaySpinBox.setSuffix("s")

        # File buttons
        self.sequenceSaveButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceSaveButton.setGeometry(QtCore.QRect(10, 770, 260, 30))
        self.sequenceSaveButton.setText("Save Sequence")
        self.sequenceSaveButton.pressed.connect(self.saveSequence)

        self.sequenceLoadButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceLoadButton.setGeometry(QtCore.QRect(10, 805, 260, 30))
        self.sequenceLoadButton.setText("Load Sequence")
        self.sequenceLoadButton.pressed.connect(self.loadSequence)

        logger.info("Sequence recorder controls initialized")

    def setupEndstopDisplays(self):
        """Initialize references to endstop labels (defined in gui.py)"""
        # Endstop labels are inline with articulation controls in gui.py
        # Labels: endstopLabelArt1-6 correspond to axes X, Y, Z, U, V, W
        logger.info("Endstop status displays initialized (inline with articulation controls)")

    def setupPositionHistoryControls(self):
        """Create embedded 3D robot visualization and controls"""
        from robot_3d_visualizer import Robot3DCanvas

        # Create group box for embedded 3D visualization (right side of window)
        self.positionHistoryGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.positionHistoryGroupBox.setGeometry(QtCore.QRect(1210, 10, 600, 900))
        self.positionHistoryGroupBox.setTitle("3D Robot Visualization")

        # Embed 3D matplotlib canvas
        self.position_canvas = Robot3DCanvas(self.positionHistoryGroupBox, width=5.8, height=6.5, dpi=100)
        self.position_canvas.setGeometry(QtCore.QRect(10, 25, 580, 650))

        # Controls panel below 3D visualization
        self.historyControlsFrame = QtWidgets.QFrame(self.positionHistoryGroupBox)
        self.historyControlsFrame.setGeometry(QtCore.QRect(10, 685, 580, 200))
        self.historyControlsFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)

        # Time window control
        self.timeWindowLabel = QtWidgets.QLabel(self.historyControlsFrame)
        self.timeWindowLabel.setGeometry(QtCore.QRect(10, 10, 120, 20))
        self.timeWindowLabel.setText("Time Window (s):")

        self.timeWindowSpinBox = QtWidgets.QSpinBox(self.historyControlsFrame)
        self.timeWindowSpinBox.setGeometry(QtCore.QRect(135, 8, 80, 25))
        self.timeWindowSpinBox.setMinimum(10)
        self.timeWindowSpinBox.setMaximum(600)
        self.timeWindowSpinBox.setValue(60)
        self.timeWindowSpinBox.setSingleStep(10)

        # Display option checkboxes (3 rows of 2)
        checkbox_y = 40
        self.displayCheckboxes = {}
        display_options = [
            ('show_robot', 'Show Robot Arm'),
            ('show_trajectory', 'Show Trajectory'),
            ('show_base_frame', 'Show Base Frame'),
            ('show_workspace', 'Show Workspace'),
            ('show_grid', 'Show Grid Floor'),
            ('auto_rotate', 'Auto-rotate View')
        ]

        for i, (key, label) in enumerate(display_options):
            row = i // 2
            col = i % 2
            x = 10 + col * 290
            y = checkbox_y + row * 30

            checkbox = QtWidgets.QCheckBox(self.historyControlsFrame)
            checkbox.setGeometry(QtCore.QRect(x, y, 280, 25))
            checkbox.setText(label)
            # Default checked states
            checkbox.setChecked(key in ['show_robot', 'show_trajectory', 'show_base_frame', 'show_grid'])
            self.displayCheckboxes[key] = checkbox

        # Action buttons (2 rows)
        button_y = 140
        self.exportHistoryButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.exportHistoryButton.setGeometry(QtCore.QRect(10, button_y, 135, 30))
        self.exportHistoryButton.setText("Export CSV")
        self.exportHistoryButton.pressed.connect(self.exportPositionHistory)

        self.clearHistoryButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.clearHistoryButton.setGeometry(QtCore.QRect(155, button_y, 135, 30))
        self.clearHistoryButton.setText("Clear History")
        self.clearHistoryButton.pressed.connect(self.clearPositionHistory)

        self.resetViewButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.resetViewButton.setGeometry(QtCore.QRect(300, button_y, 135, 30))
        self.resetViewButton.setText("Reset View")
        self.resetViewButton.pressed.connect(self.resetVisualizationView)

        self.refreshGraphButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.refreshGraphButton.setGeometry(QtCore.QRect(445, button_y, 125, 30))
        self.refreshGraphButton.setText("Refresh")
        self.refreshGraphButton.pressed.connect(self.updateEmbeddedGraph)

        # Make the group box visible
        self.positionHistoryGroupBox.show()

        # Start auto-update timer for 3D visualization
        # OPTIMIZED: matplotlib rendering is expensive
        # Dirty flag pattern in visualizer prevents unnecessary redraws
        self.graph_update_timer = QtCore.QTimer()
        self.graph_update_timer.timeout.connect(self.updateEmbeddedGraph)
        self.graph_update_timer.start(config.GRAPH_UPDATE_INTERVAL_MS)

        logger.info(f"Embedded 3D robot visualization initialized ({config.GRAPH_UPDATE_INTERVAL_MS}ms update interval)")

    def updateEmbeddedGraph(self):
        """Update the embedded 3D robot visualization"""
        if not hasattr(self, 'position_canvas'):
            return

        # Get time window from spinbox
        window_size = self.timeWindowSpinBox.value()

        # Get display options from checkboxes
        options = {
            key: checkbox.isChecked()
            for key, checkbox in self.displayCheckboxes.items()
        }

        # Update the 3D visualization
        self.position_canvas.update_visualization(self.position_history, window_size, options)

    def resetVisualizationView(self):
        """Reset 3D visualization view to default isometric angle"""
        if hasattr(self, 'position_canvas'):
            self.position_canvas.reset_view()
            logger.info("3D view reset to isometric")

    def exportPositionHistory(self):
        """Export position history to CSV"""
        if len(self.position_history) == 0:
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Warning)
            msgBox.setText("No position history to export.")
            msgBox.setWindowTitle("No Data")
            msgBox.exec_()
            return

        from datetime import datetime
        default_filename = f"position_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Export Position History",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            if self.position_history.export_to_csv(filename):
                logger.info(f"Position history exported to {filename}")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Position history exported successfully to:\n{filename}\n\n{len(self.position_history)} snapshots saved.")
                msgBox.setWindowTitle("Export Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to export position history.")
                msgBox.setWindowTitle("Export Failed")
                msgBox.exec_()

    def clearPositionHistory(self):
        """Clear position history after confirmation"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Question)
        msgBox.setText(f"Clear all position history?\n\nThis will delete {len(self.position_history)} recorded snapshots.")
        msgBox.setWindowTitle("Clear History")
        msgBox.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msgBox.setDefaultButton(QtWidgets.QMessageBox.No)

        if msgBox.exec_() == QtWidgets.QMessageBox.Yes:
            self.position_history.clear()
            logger.info("Position history cleared by user")

            infoBox = QtWidgets.QMessageBox()
            infoBox.setIcon(QtWidgets.QMessageBox.Information)
            infoBox.setText("Position history cleared.")
            infoBox.setWindowTitle("Cleared")
            infoBox.exec_()

    def recordSequencePoint(self):
        """Record current joint positions to sequence"""
        q1 = self.SpinBoxArt1.value()
        q2 = self.SpinBoxArt2.value()
        q3 = self.SpinBoxArt3.value()
        q4 = self.SpinBoxArt4.value()
        q5 = self.SpinBoxArt5.value()
        q6 = self.SpinBoxArt6.value()
        gripper = self.SpinBoxGripper.value()
        delay = self.sequenceDelaySpinBox.value()

        if not self.sequence_recorder.is_recording:
            self.sequence_recorder.start_recording("Current Sequence")

        self.sequence_recorder.record_point(q1, q2, q3, q4, q5, q6, gripper, delay)

        # Update list display
        point_text = f"Point {len(self.sequence_recorder.current_sequence)}: q1={q1:.1f}° q2={q2:.1f}° q3={q3:.1f}° delay={delay:.1f}s"
        self.sequencePointsList.addItem(point_text)

        logger.info(f"Recorded point: {point_text}")

    def deleteSequencePoint(self):
        """Delete selected point from sequence"""
        current_row = self.sequencePointsList.currentRow()
        if current_row >= 0:
            self.sequence_recorder.current_sequence.remove_point(current_row)
            self.sequencePointsList.takeItem(current_row)
            logger.info(f"Deleted point {current_row + 1}")

    def clearSequence(self):
        """Clear all points from sequence"""
        self.sequence_recorder.current_sequence.clear()
        self.sequencePointsList.clear()
        logger.info("Cleared all sequence points")

    def playSequence(self):
        """Play the recorded sequence"""
        if len(self.sequence_recorder.current_sequence) == 0:
            logger.warning("Cannot play empty sequence")
            return

        if self.is_playing_sequence:
            logger.warning("Sequence already playing")
            return

        # Create player with movement callback
        self.sequence_player = seq_rec.SequencePlayer(self.executeSequenceMove)

        # Get playback parameters
        speed = self.sequenceSpeedSpinBox.value()
        loop = self.sequenceLoopCheckBox.isChecked()

        # Start playback (non-blocking)
        self.sequence_player.start_playback(
            self.sequence_recorder.current_sequence,
            speed=speed,
            loop=loop
        )

        # Update button states
        self.sequencePlayButton.setEnabled(False)
        self.sequencePauseButton.setEnabled(True)
        self.sequenceStopButton.setEnabled(True)
        self.is_playing_sequence = True

        # Start timer to update playback (interval from config)
        self.sequence_timer.start(config.SEQUENCE_TIMER_INTERVAL)

        logger.info(f"Started sequence playback (speed={speed}x, loop={loop})")

    def updateSequencePlayback(self):
        """Called by QTimer to advance sequence playback (thread-safe)"""
        if not self.sequence_player:
            self.sequence_timer.stop()
            return

        should_continue, current, total = self.sequence_player.playNextPoint()

        if not should_continue:
            # Playback finished
            self.stopSequence()
            logger.info("Sequence playback completed")

    def pauseSequence(self):
        """Pause/resume sequence playback"""
        if self.sequence_player:
            if self.sequence_player.is_paused:
                self.sequence_player.resume()
                self.sequencePauseButton.setText("Pause")
            else:
                self.sequence_player.pause()
                self.sequencePauseButton.setText("Resume")

    def stopSequence(self):
        """Stop sequence playback"""
        self.sequence_timer.stop()

        if self.sequence_player:
            self.sequence_player.stop()

        self.sequencePlayButton.setEnabled(True)
        self.sequencePauseButton.setEnabled(False)
        self.sequencePauseButton.setText("Pause")
        self.sequenceStopButton.setEnabled(False)
        self.is_playing_sequence = False

        logger.info("Stopped sequence playback")

    def executeSequenceMove(self, q1, q2, q3, q4, q5, q6, gripper):
        """Execute a single movement during sequence playback"""
        logger.info(f"Executing sequence move: q1={q1:.1f}°, q2={q2:.1f}°, q3={q3:.1f}°, q4={q4:.1f}°, q5={q5:.1f}°, q6={q6:.1f}°, grip={gripper}")

        # DIFFERENTIAL MECHANISM: Calculate motor positions using helper
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(q5, q6)

        # Build and send movement command
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        command = CommandBuilder.build_axis_command(
            movement_type,
            {"X": q1, "Y": q2, "Z": q3, "U": q4, "V": motor_v, "W": motor_w},
            feedrate
        )
        self.command_sender.send(command, show_in_console=False)  # Don't spam console during playback

        # Move gripper if needed
        if gripper > 0:
            pwm_value = self._gripper_percent_to_pwm(gripper)
            # Map 0-255 PWM range to 0-180 servo angle for RepRapFirmware M280 command
            servo_angle = int((pwm_value / 255.0) * 180.0)
            gripper_cmd = f"M280 P0 S{servo_angle}"
            self.command_sender.send(gripper_cmd, show_in_console=False)

    def saveSequence(self):
        """Save sequence to file"""
        if len(self.sequence_recorder.current_sequence) == 0:
            logger.warning("Cannot save empty sequence")
            return

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Save Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            if self.sequence_recorder.save_sequence(filename):
                logger.info(f"Saved sequence to {filename}")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Sequence saved successfully to:\n{filename}")
                msgBox.setWindowTitle("Save Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to save sequence")
                msgBox.setWindowTitle("Save Failed")
                msgBox.exec_()

    def loadSequence(self):
        """Load sequence from file"""
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Load Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            sequence = self.sequence_recorder.load_sequence(filename)
            if sequence:
                self.sequence_recorder.set_current_sequence(sequence)

                # Update list display
                self.sequencePointsList.clear()
                for i, point in enumerate(sequence.points):
                    point_text = f"Point {i+1}: q1={point.q1:.1f}° q2={point.q2:.1f}° q3={point.q3:.1f}° delay={point.delay:.1f}s"
                    self.sequencePointsList.addItem(point_text)

                logger.info(f"Loaded sequence '{sequence.name}' with {len(sequence)} points")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Loaded sequence:\n{sequence.name}\n{len(sequence)} points")
                msgBox.setWindowTitle("Load Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to load sequence")
                msgBox.setWindowTitle("Load Failed")
                msgBox.exec_()

# Serial Connection functions
    def getSerialPorts(self):
        self.SerialPortComboBox.clear()
        available_ports = spf.serial_ports()
        self.SerialPortComboBox.addItems(available_ports)

        # Auto-detect and select the robot's COM port
        robot_port = spf.get_robot_port()
        if robot_port:
            # Find the index of the detected port and select it
            index = self.SerialPortComboBox.findText(robot_port)
            if index >= 0:
                self.SerialPortComboBox.setCurrentIndex(index)
                logger.info(f"Auto-detected robot port: {robot_port}")
            else:
                logger.warning(f"Detected port {robot_port} not found in combo box")
        else:
            logger.info("No robot port auto-detected, please select manually")

    def connectSerial(self):
        # Check if already connected - if so, disconnect
        if s0.isOpen():
            self.disconnectSerial()
            return

        serialPort = self.SerialPortComboBox.currentText()
        baudrate = self.BaudRateComboBox.currentText()
        if serialPort == "":
            self.blankSerialPort()
            return
        if baudrate == "":
            self.blankBaudRate()
            return

        # Disable connect button to prevent multiple clicks
        self.ConnectButton.setEnabled(False)
        self.RobotStateDisplay.setText("Connecting...")
        self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 255, 0)')  # Yellow

        # Run connection in separate thread to prevent GUI freeze
        connection_thread = threading.Thread(
            target=self._connectSerialWorker,
            args=(serialPort, baudrate),
            daemon=True
        )
        connection_thread.start()

    def disconnectSerial(self):
        """Disconnect from serial port"""
        try:
            # Stop serial thread
            if self.SerialThreadClass and self.SerialThreadClass.isRunning():
                self.SerialThreadClass.stop()
                self.SerialThreadClass.wait(2000)

            # Close serial port
            s0.close()

            # Update GUI
            self.serialDisconnected()
            self.ConnectButton.setText("Connect")
            self.ConnectButton.setEnabled(True)

            logger.info("Disconnected from serial port")

        except Exception as e:
            logger.error(f"Error during disconnect: {e}")

    def _connectSerialWorker(self, serialPort, baudrate):
        """Worker thread for serial connection (prevents GUI freeze)"""
        try:
            # Stop existing thread if running
            if self.SerialThreadClass and self.SerialThreadClass.isRunning():
                self.SerialThreadClass.stop()
                self.SerialThreadClass.wait(2000)

            # Close existing connection
            s0.close()

            # Configure and open new connection (this can block on Windows!)
            s0.serial.port = serialPort
            s0.serial.baudrate = int(baudrate)
            s0.serial.timeout = config.SERIAL_TIMEOUT
            s0.open()

            # Clear any stale data in buffers
            s0.reset_input_buffer()
            logger.debug("Cleared serial input buffer")

            # Create and start new serial thread (pass GUI instance for event-driven endstop polling)
            self.SerialThreadClass = SerialThreadClass(gui_instance=self)
            self.SerialThreadClass.start()

            # Emit success signal (thread-safe)
            # Signal connection will happen in GUI thread
            self.connection_signals.success.emit(serialPort, baudrate)

        except Exception as e:
            logger.exception("Serial connection error")
            # Emit error signal (thread-safe)
            self.connection_signals.error.emit(str(e))

    def _onConnectionSuccess(self, serialPort, baudrate):
        """Called when connection succeeds (runs in GUI thread)"""
        # Connect serial signal in GUI thread (critical for Qt signals to work)
        self.SerialThreadClass.serialSignal.connect(self.updateConsole)

        # Update GUI to show connected state
        self.updateCurrentState("Idle")
        self.ConnectButton.setText("Disconnect")
        self.ConnectButton.setEnabled(True)

        # Request initial position after thread is ready
        QtCore.QTimer.singleShot(50, self.requestInitialPosition)

        logger.info(f"Connected to {serialPort} at {baudrate} baud")
        logger.info("Serial thread started (Firmware: RepRapFirmware)")
        logger.info("Requesting position update to initialize differential tracking")

    def _onConnectionError(self, error_msg):
        """Called when connection fails (runs in GUI thread)"""
        self.serialDisconnected()
        self.ConnectButton.setText("Connect")
        self.ConnectButton.setEnabled(True)

        logger.error(f"Error opening serial port: {error_msg}")
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Critical)
        msgBox.setText(f"Failed to connect to serial port:\n{error_msg}")
        msgBox.setWindowTitle("Connection Error")
        msgBox.exec_()

    def requestInitialPosition(self):
        """Request initial position and endstop status after connection (called by QTimer)"""
        if s0.isOpen():
            s0.write("M114\n".encode('UTF-8'), priority=True)
            s0.write("M119\n".encode('UTF-8'), priority=True)
            logger.debug("Requested initial position (M114) and endstop status (M119)")

    def serialDisconnected(self):
        self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 0, 0)')
        self.RobotStateDisplay.setText("Disconnected")
        self.ConnectButton.setText("Connect")

    def updateConsole(self, dataRead):
        verboseShow=self.ConsoleShowVerbosecheckBox.isChecked()
        okShow=self.ConsoleShowOkRespcheckBox.isChecked()

        # Use parsing module to identify response types
        isDataReadVerbose = parsing_patterns.is_m114_response(dataRead)
        isEndstopResponse = parsing_patterns.is_m119_response(dataRead)
        isDataOkResponse = parsing_patterns.is_ok_response(dataRead)

        # Check if homing completed
        if self.is_homing and isDataOkResponse:
            self.is_homing = False
            self.HomeButton.setEnabled(True)
            self.HomeButton.setText("Home")

        if dataRead=="SERIAL-DISCONNECTED":
            s0.close()
            self.serialDisconnected()
            logger.warning("Serial Connection Lost")

        else:
            # Show responses to manual commands for 2 seconds after command
            time_since_manual = time.time() - self.last_manual_command_time
            if time_since_manual < 2.0:
                self.ConsoleOutput.appendPlainText(dataRead)
            # Handle endstop responses
            elif isEndstopResponse:
                self.updateEndstopDisplay(dataRead)
            elif not isDataReadVerbose and not isDataOkResponse:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataOkResponse and okShow:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataReadVerbose:
                self.updateFKPosDisplay(dataRead)
                if verboseShow:
                    self.ConsoleOutput.appendPlainText(dataRead)

    def sendSerialCommand(self):
        """Send manual command from console input"""
        command = self.ConsoleInput.text().strip()

        # Don't send empty commands
        if not command:
            return

        # Send command
        if self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection):
            # Update history and clear input
            self.ConsoleInput.addToHistory(command)
            self.ConsoleInput.clear()
            # Mark time of manual command to show responses for next 2 seconds
            self.last_manual_command_time = time.time()
            logger.debug(f"Manual command sent: {command}")

    def validatePosition(self, axis, value):
        """
        DEPRECATED: Use robot_controller.validate_position() instead
        This method is kept for backward compatibility only

        Validate position value for reasonableness (uses config.py limits)

        Args:
            axis: Axis name (X, Y, Z, U, V, W)
            value: Position value in degrees

        Returns:
            (is_valid, sanitized_value)
        """
        # Delegate to RobotController
        return self.robot_controller.validate_position(axis, value)

    def _parseM114Response(self, dataRead):
        """
        Extract position dictionary from M114 response

        Args:
            dataRead: Raw M114 response string

        Returns:
            dict: Axis positions {axis: value} or None if parse failed
        """
        result = parsing_patterns.parse_m114_response(dataRead)
        return result if result else None

    def _validateAllPositions(self, pos_dict):
        """
        Validate all axis positions and return sanitized values
        NOW USES: RobotController for validation

        Args:
            pos_dict: Dictionary of raw positions from M114

        Returns:
            dict: Validated positions for all axes
        """
        AXES = ['X', 'Y', 'Z', 'U', 'V', 'W']
        positions = {}

        for axis in AXES:
            valid, value = self.robot_controller.validate_position(axis, pos_dict.get(axis, 0.0))
            positions[axis] = value

            if not valid:
                logger.warning(f"Position validation corrected for {axis}")

        return positions

    def _updateInternalState(self, positions):
        """
        Update internal tracking variables from validated positions
        NOW USES: RobotController to update state and calculate Art5/Art6

        Args:
            positions: Dictionary of validated axis positions

        Returns:
            dict: Updated positions with calculated Art5/Art6
        """
        # Use RobotController to update all state
        updated_positions = self.robot_controller.update_positions_from_firmware(positions)

        # Keep local references for backward compatibility with existing code
        self.current_motor_v = self.robot_controller.current_motor_v
        self.current_motor_w = self.robot_controller.current_motor_w
        self.desired_art5 = self.robot_controller.desired_art5
        self.desired_art6 = self.robot_controller.desired_art6
        self.position_update_count = self.robot_controller.position_update_count

        return updated_positions

    def _recordPositionHistory(self, positions):
        """
        Record position to history (sampled based on config)

        Args:
            positions: Dictionary with all position data
        """
        if self.position_update_count % config.POSITION_HISTORY_SAMPLE_RATE == 0:
            self.position_history.add_snapshot(
                art1=positions['X'],
                art2=positions['Y'],
                art3=positions['Z'],
                art4=positions['U'],
                art5=positions['Art5'],
                art6=positions['Art6']
            )

    def _logPositions(self, positions):
        """
        Log position update details (for debugging)

        Args:
            positions: Dictionary with all position data
        """
        logger.debug(
            f"Position feedback - X:{positions['X']:.2f} Y:{positions['Y']:.2f} "
            f"Z:{positions['Z']:.2f} U:{positions['U']:.2f} V:{positions['V']:.2f} "
            f"W:{positions['W']:.2f}"
        )
        logger.info(
            f"  Art1<-X:{positions['X']:.2f}° | Art2<-Y:{positions['Y']:.2f}° | "
            f"Art3<-Z:{positions['Z']:.2f}° | Art4<-U:{positions['U']:.2f}° | "
            f"Art5(calc):{positions['Art5']:.2f}° | Art6(calc):{positions['Art6']:.2f}°"
        )

    def _updateGUIPositions(self, positions):
        """
        Update GUI labels with position data (throttled)

        Args:
            positions: Dictionary with all position data
        """
        # Log positions (throttled)
        if self.position_update_count % config.LOGGING_INTERVAL_POSITIONS == 0:
            self._logPositions(positions)

        # GUI updates (throttled by time)
        current_time = time.time()
        if current_time - self.last_gui_update_time >= config.GUI_UPDATE_INTERVAL:
            self.last_gui_update_time = current_time

            # Update all display labels
            self.FKCurrentPosValueArt1.setText(f"{positions['X']:.2f}º")
            self.FKCurrentPosValueArt2.setText(f"{positions['Y']:.2f}º")
            self.FKCurrentPosValueArt3.setText(f"{positions['Z']:.2f}º")
            self.FKCurrentPosValueArt4.setText(f"{positions['U']:.2f}º")
            self.FKCurrentPosValueArt5.setText(f"{positions['Art5']:.2f}º")
            self.FKCurrentPosValueArt6.setText(f"{positions['Art6']:.2f}º")

            # Set status to Idle since M114 doesn't provide status
            self.updateCurrentState("Idle")

    def updateFKPosDisplay(self, dataRead):
        """
        Parse M114 position response and update all displays

        This method orchestrates position processing through several steps:
        1. Parse raw M114 response
        2. Validate all positions
        3. Update internal state and differential kinematics
        4. Record to position history
        5. Update GUI displays (throttled)

        Args:
            dataRead: Raw M114 response string from firmware
        """
        try:
            # Step 1: Parse M114 response
            pos_dict = self._parseM114Response(dataRead)
            if not pos_dict:
                return

            # Step 2: Validate we have differential axes (V and W required)
            if 'V' not in pos_dict or 'W' not in pos_dict:
                logger.warning("M114 response missing V or W axis - skipping update")
                return

            # Step 3: Validate all positions
            positions = self._validateAllPositions(pos_dict)

            # Step 4: Update internal state and calculate differential kinematics
            positions = self._updateInternalState(positions)

            # Step 5: Record to position history (sampled)
            self._recordPositionHistory(positions)

            # Step 6: Update GUI displays (throttled)
            self._updateGUIPositions(positions)

        except (ValueError, KeyError, IndexError) as e:
            logger.error(f"Error parsing M114 response: {e}")
            logger.debug(f"Problematic data: {dataRead}")
            logger.exception("Position parsing error")

    def updateEndstopDisplay(self, dataRead):
        """Parse M119 endstop response and update GUI displays"""
        # Format: "Endstops - X: at min stop, Y: not stopped, Z: not stopped, U: not stopped, V: at min stop, W: at min stop, Z probe: at min stop"
        try:
            # Use parsing module to extract endstop statuses
            endstops = parsing_patterns.parse_m119_response(dataRead)
            if not endstops:
                return

            # Update GUI labels with color coding (compact format for inline display)
            # Green = not triggered, Red = triggered
            # Mapping: X->Art1, Y->Art2, Z->Art3, U->Art4, V->Art5, W->Art6
            def updateLabel(label, axis_name):
                if axis_name in endstops:
                    status = endstops[axis_name]
                    # Compact status text with axis label
                    if "not stopped" in status:
                        label.setText(f"{axis_name}: OK")
                        label.setStyleSheet("background-color: rgb(200, 255, 200); padding: 2px; border-radius: 3px;")  # Light green
                    elif "min" in status:
                        label.setText(f"{axis_name}: MIN")
                        label.setStyleSheet("background-color: rgb(255, 200, 200); padding: 2px; border-radius: 3px;")  # Light red
                    elif "max" in status:
                        label.setText(f"{axis_name}: MAX")
                        label.setStyleSheet("background-color: rgb(255, 200, 200); padding: 2px; border-radius: 3px;")  # Light red
                    else:
                        label.setText(f"{axis_name}: {status[:6]}")
                        label.setStyleSheet("background-color: rgb(255, 255, 200); padding: 2px; border-radius: 3px;")  # Yellow

            updateLabel(self.endstopLabelArt1, "X")
            updateLabel(self.endstopLabelArt2, "Y")
            updateLabel(self.endstopLabelArt3, "Z")
            updateLabel(self.endstopLabelArt4, "U")
            updateLabel(self.endstopLabelArt5, "V")
            updateLabel(self.endstopLabelArt6, "W")

        except Exception as e:
            logger.error(f"Error parsing M119 endstop response: {e}")
            logger.debug(f"Problematic data: {dataRead}")

    def updateCurrentState(self, state):
        """Update robot state display with appropriate color coding"""
        # State color mapping for visual feedback
        STATE_COLORS = {
            'Idle': 'rgb(0, 255, 0)',      # Green - ready
            'Run': 'rgb(0, 255, 0)',       # Green - running
            'Home': 'rgb(85, 255, 255)',   # Cyan - homing
            'Alarm': 'rgb(255, 255, 0)',   # Yellow - warning
            'Hold': 'rgb(255, 0, 0)',      # Red - stopped
        }

        self.RobotStateDisplay.setText(state)
        color = STATE_COLORS.get(state, 'rgb(255, 255, 255)')  # Default: white
        self.RobotStateDisplay.setStyleSheet(f'background-color: {color}')


    def _showWarningMessage(self, message, title="Warning"):
        """Helper to display warning message boxes"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setWindowTitle(title)
        msgBox.setText(message)
        msgBox.exec_()

    def blankSerialPort(self):
        """Show warning for missing serial port"""
        self._showWarningMessage(
            "There is not Serial Port value indicated to establish the connection.\n"
            "Please check it and try to connect again."
        )

    def blankBaudRate(self):
        """Show warning for missing baud rate"""
        self._showWarningMessage(
            "There is not Baud Rate value indicated to establish the connection.\n"
            "Please check it and try to connect again."
        )

    def noSerialConnection(self):
        """Show warning for no serial connection"""
        self._showWarningMessage(
            "The connection has not been established yet. "
            "Please establish the connection before trying to control."
        )

############### SERIAL READ THREAD CLASS ###############

class SerialThreadClass(QtCore.QThread):
    serialSignal = pyqtSignal(str)

    def __init__(self, gui_instance=None, parent=None):
        super(SerialThreadClass, self).__init__(parent)
        self.gui_instance = gui_instance  # Reference to GUI for movement tracking
        self.running = True
        self.elapsedTime = time.time()
        self.endstopCheckTime = time.time()
        self.status_polling_paused = False  # Flag to pause polling during long commands
        self.blocking_command_start_time = 0.0  # When blocking command was sent

    def stop(self):
        """Gracefully stop the thread"""
        self.running = False

    def run(self):
        """Main thread loop with non-blocking reads and command queue processing"""
        while self.running:
            if not s0.isOpen():
                time.sleep(0.1)
                continue

            try:
                # Check if connection is still alive
                try:
                    bytes_available = s0.inWaiting()
                except (OSError, serial.SerialException):
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    logger.warning("Lost Serial connection!")
                    break

                current_time = time.time()

                # PROCESS COMMAND QUEUE (non-blocking)
                # This is where all writes actually happen, keeping GUI thread free
                command = s0.get_next_command()
                if command:
                    success = s0._write_internal(command)
                    if not success:
                        self.serialSignal.emit("SERIAL-DISCONNECTED")
                        break

                    # Check if this is a long-running blocking command
                    command_str = command.decode('UTF-8', errors='replace').strip().upper()
                    # RRF blocking commands: G28 (home), G29 (bed probe), M999 (reset)
                    blocking_commands = ['G28', 'G29', 'M999']

                    # Pause status polling if we sent a blocking command
                    for block_cmd in blocking_commands:
                        if command_str.startswith(block_cmd):
                            self.status_polling_paused = True
                            self.blocking_command_start_time = current_time
                            logger.info(f"Pausing status polling for blocking command: {command_str}")
                            break

                # Check for timeout on paused polling (safety mechanism)
                if self.status_polling_paused:
                    time_paused = current_time - self.blocking_command_start_time
                    if time_paused >= config.BLOCKING_COMMAND_MAX_PAUSE:
                        self.status_polling_paused = False
                        logger.warning(f"Forcing resume of status polling after {time_paused:.1f}s timeout (max: {config.BLOCKING_COMMAND_MAX_PAUSE}s)")
                        # Request immediate position update
                        s0.write("M114\n".encode('UTF-8'), priority=True)
                        s0.write("M119\n".encode('UTF-8'), priority=True)

                # Send status request (interval from config) - ONLY if not paused
                if not self.status_polling_paused and current_time - self.elapsedTime > config.SERIAL_STATUS_REQUEST_INTERVAL:
                    self.elapsedTime = current_time
                    try:
                        s0.write("M114\n".encode('UTF-8'), priority=True)
                    except Exception as e:
                        logger.error(f"Error queuing status request: {e}")

                # Poll endstop status regularly - ONLY if not paused
                if not self.status_polling_paused and current_time - self.endstopCheckTime > config.SERIAL_ENDSTOP_REQUEST_INTERVAL:
                    self.endstopCheckTime = current_time
                    try:
                        s0.write("M119\n".encode('UTF-8'), priority=True)
                    except Exception as e:
                        logger.error(f"Error queuing endstop request: {e}")

                # NON-BLOCKING BATCH READ: Process multiple lines if available
                # This reduces loop overhead when buffer has multiple responses
                try:
                    if bytes_available > 0:
                        # OPTIMIZATION: Read multiple lines when buffer is full (batch processing)
                        # Estimate lines available (assuming ~30 bytes per line average)
                        estimated_lines = max(1, min(10, bytes_available // 30))

                        for _ in range(estimated_lines):
                            # Check if data still available (non-blocking)
                            if s0.inWaiting() == 0:
                                break

                            dataBytes = s0.readline()
                            if dataBytes:
                                # Decode bytes to string and strip whitespace
                                dataCropped = dataBytes.decode('UTF-8', errors='replace').strip()
                                if dataCropped:
                                    self.serialSignal.emit(dataCropped)

                                    # Resume status polling when blocking command completes
                                    # Must meet BOTH conditions: received "ok" AND minimum time elapsed
                                    if self.status_polling_paused and "ok" in dataCropped.lower():
                                        time_elapsed = current_time - self.blocking_command_start_time
                                        if time_elapsed >= config.BLOCKING_COMMAND_MIN_PAUSE:
                                            self.status_polling_paused = False
                                            logger.info(f"Resuming status polling after blocking command completed ({time_elapsed:.1f}s elapsed)")
                                            # Request immediate position update after resuming
                                            s0.write("M114\n".encode('UTF-8'), priority=True)
                                            s0.write("M119\n".encode('UTF-8'), priority=True)
                                        else:
                                            logger.debug(f"Received 'ok' but only {time_elapsed:.1f}s elapsed (need {config.BLOCKING_COMMAND_MIN_PAUSE}s), waiting...")
                except (OSError, serial.SerialException) as e:
                    logger.error(f"Error reading from serial: {e}")
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    break

                # Small sleep to prevent busy-waiting (from config)
                time.sleep(config.SERIAL_THREAD_SLEEP)

            except Exception as e:
                logger.exception("Unexpected error in serial thread")
                time.sleep(0.1)

        logger.info("Serial thread stopped")


###############  SERIAL READ THREAD CLASS ###############










class MainWindow(QtWidgets.QMainWindow):
    """Custom MainWindow to handle close event"""
    def __init__(self, gui_instance):
        super().__init__()
        self.gui_instance = gui_instance

    def closeEvent(self, event):
        """Handle window close event with proper cleanup"""
        # Stop serial thread if running
        if self.gui_instance.SerialThreadClass and self.gui_instance.SerialThreadClass.isRunning():
            logger.info("Stopping serial thread...")
            self.gui_instance.SerialThreadClass.stop()
            self.gui_instance.SerialThreadClass.wait(2000)

        # Close serial port
        s0.close()
        logger.info("Application closed cleanly")
        event.accept()

if __name__ == '__main__':
    # Enable High DPI scaling for 4K displays
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)

    # Create main window first
    mwindow = MainWindow(None)
    mwindow.setMinimumSize(config.MAIN_WINDOW_MIN_WIDTH, config.MAIN_WINDOW_MIN_HEIGHT)

    # Create GUI instance
    prog = BifrostGUI(mwindow)

    # Link gui instance to window for cleanup
    mwindow.gui_instance = prog

    mwindow.show()
    sys.exit(app.exec_())
