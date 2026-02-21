import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import config

# Import appropriate GUI based on config
if config.USE_MODERN_GUI:
    from gui_modern import Ui_MainWindow, PointEditDialog
else:
    from gui import Ui_MainWindow
    PointEditDialog = None  # Not available in legacy GUI

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
from serial_manager import SerialManager
from serial_thread import SerialThread
from connection_manager import ConnectionManager, ConnectionState
from movement_controller import MovementController, MovementParams, get_movement_params_from_gui
from position_display_controller import PositionDisplayController
from sequence_controller import SequenceController, JointPositions
from ik_controller import IKController
from fk_controller import FKController, JointValues
from gripper_controller import GripperController
from ui_state_manager import UIStateManager, ConnectionState as UIConnectionState, RobotState
from serial_response_router import SerialResponseRouter
from visualization_controller import VisualizationController
from position_history_manager import PositionHistoryManager
from calibration_panel import load_gripper_calibration_on_startup, load_home_position_on_startup, load_park_position_on_startup
from config_g_manager import read_m569_directions, DEFAULT_CONFIG_G_PATH
from coordinate_frames import FrameManager, pose_to_xyz_rpy
import forward_kinematics as fk

import serial
import time
import json
import logging
import numpy as np

# Import simulation hardware (always available for runtime toggle)
from simulated_hardware import SimulatedSerialManager, SimulatedSerialThread

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

# Load calibration from files (if they exist) before GUI starts
load_gripper_calibration_on_startup()
load_home_position_on_startup()
load_park_position_on_startup()

# This application is designed for RepRapFirmware (RRF) only
# All parsing patterns are now in parsing_patterns.py module

# Global serial manager instance (real or simulated based on config)
if config.USE_SIMULATION_MODE:
    logger.info("=== STARTING IN SIMULATION MODE ===")
    logger.info("No hardware required - using simulated robot")
    s0 = SimulatedSerialManager()
else:
    s0 = SerialManager()

class WAlignmentDialog(QtWidgets.QDialog):
    """Dialog for manually aligning J6 (wrist roll) during homing.

    W has no endstop, so the user must jog J6 to the desired zero position
    and confirm before homing finalizes.  Pure J6 rotation requires equal
    V+W movement (differential kinematics).  The cumulative offset is
    tracked so the caller can restore V to its homed position afterwards.
    """

    def __init__(self, command_sender, parent=None):
        super().__init__(parent)
        self.command_sender = command_sender
        self.total_offset = 0
        self.setWindowTitle("Align J6 (Wrist Roll)")
        self.setModal(True)
        self.setMinimumWidth(320)

        layout = QtWidgets.QVBoxLayout(self)

        label = QtWidgets.QLabel(
            "J6 (wrist roll) has no endstop.\n\n"
            "Use the buttons below to rotate J6\n"
            "to the desired zero position, then click OK."
        )
        layout.addWidget(label)

        jog_layout = QtWidgets.QHBoxLayout()
        for step, text in [(-10, "-10\u00b0"), (-1, "-1\u00b0"), (1, "+1\u00b0"), (10, "+10\u00b0")]:
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(lambda checked, s=step: self._jog(s))
            jog_layout.addWidget(btn)
        layout.addLayout(jog_layout)

        ok_btn = QtWidgets.QPushButton("OK - Set as Zero")
        ok_btn.clicked.connect(self.accept)
        layout.addWidget(ok_btn)

    def _jog(self, step):
        # Pure J6 rotation: move V and W equally (ΔArt5=0, ΔArt6=step)
        self.command_sender.send_if_connected("G91")
        self.command_sender.send_if_connected(f"G1 V{step} W{step} F1800")
        self.command_sender.send_if_connected("G90")
        self.total_offset += step


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

        # Create connection manager (handles serial connection lifecycle)
        self.connection_manager = ConnectionManager(s0)
        self.connection_manager.set_serial_thread_class(SerialThreadClass)

        # Keep old connection_signals for backward compatibility
        self.connection_signals = self.connection_manager  # ConnectionManager has same signals

        # Replace ConsoleInput with HistoryLineEdit for command history
        old_console_input = self.ConsoleInput
        parent = old_console_input.parent()  # Keep it on the same panel/container
        layout = parent.layout() if parent else None

        self.ConsoleInput = HistoryLineEdit(parent)
        self.ConsoleInput.setObjectName("ConsoleInput")

        # If the old widget sat in a layout, replace it in-place
        if layout is not None:
            layout.replaceWidget(old_console_input, self.ConsoleInput)
            self.ConsoleInput.setMinimumSize(old_console_input.minimumSize())
            self.ConsoleInput.setMaximumSize(old_console_input.maximumSize())
        else:
            # Fallback for absolute positioning
            self.ConsoleInput.setGeometry(old_console_input.geometry())

        old_console_input.deleteLater()

        # Using RepRapFirmware (RRF) - this is the only supported firmware

        # Track serial thread state
        self.SerialThreadClass = None

        # Track last manual command time to show responses
        self.last_manual_command_time = 0

        # Track homing state
        self.is_homing = False
        self.sync_commands_pending = False

        # Track jog mode state
        self.jog_mode_enabled = True

        # Initialise command sender (will use ConsoleOutput widget)
        self.command_sender = SerialCommandSender(s0, None)  # Console widget set later after full init

        # Initialise robot controller
        self.robot_controller = RobotController()
        logger.info("RobotController initialised and integrated with GUI")

        # Initialise movement controller (will set command_sender after full init)
        self.movement_controller = MovementController(self.robot_controller, command_sender=None)

        self.getSerialPorts()

        # Menu bar is optional (removed in modern GUI)
        if hasattr(self, 'actionAbout'):
            self.actionAbout.triggered.connect(self.launchAboutWindow)
        if hasattr(self, 'actionExit'):
            self.actionExit.triggered.connect(self.close_application)

        # Connect settings button to About dialog (modern GUI)
        if hasattr(self, 'SettingsButton'):
            self.SettingsButton.pressed.connect(self.launchAboutWindow)

        # Setup embedded position history graph
        if not config.USE_MODERN_GUI:
            # Classic GUI: Full controls in separate group box
            self.setupPositionHistoryControls()
        else:
            # Modern GUI: Compact 3D view in robot state panel
            self.setupModern3DVisualization()

        self.HomeButton.pressed.connect(self.sendHomingCycleCommand)
        self.ZeroPositionButton.pressed.connect(self.sendHomePositionCommand)
        self.ParkButton.pressed.connect(self.sendParkCommand)
        self.EmergencyStopButton.pressed.connect(self.sendEmergencyStopCommand)

        # Movement type now controlled by axis column - connect its signal
        if hasattr(self, 'axis_column'):
            self.axis_column.movement_type_changed.connect(self._onMovementTypeChanged)

        self.JogModeCheckBox.toggled.connect(self.toggleJogMode)
        self.JogModeCheckBox.setChecked(True)  # Enable jog mode by default

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

        # Gripper preset buttons (Close/Open) - connect to execute movement
        if hasattr(self, 'gripper_close_btn') and self.gripper_close_btn:
            self.gripper_close_btn.clicked.connect(lambda: self.setGripperAndMove(0))
        if hasattr(self, 'gripper_open_btn') and self.gripper_open_btn:
            self.gripper_open_btn.clicked.connect(lambda: self.setGripperAndMove(100))

        # New axis control column +/- buttons (use step toggle for increment amount)
        if hasattr(self, 'axis_column'):
            # Connect mode change signal
            self.axis_column.mode_changed.connect(self._onAxisControlModeChanged)

            # Connect frame change signal
            self.axis_column.frame_changed.connect(self._onCoordinateFrameChanged)

            # Initial connection for joint mode (default)
            self._connectAxisColumnButtons()

            # Gripper +/- buttons (always visible regardless of mode)
            gripper_row = self.axis_column.rows["Gripper"]
            gripper_row.minus_btn.pressed.connect(
                lambda: self.adjustGripperValue(-self.axis_column.get_step())
            )
            gripper_row.plus_btn.pressed.connect(
                lambda: self.adjustGripperValue(self.axis_column.get_step())
            )
            gripper_row.open_btn.clicked.connect(lambda: self.setGripperAndMove(100))
            gripper_row.close_btn.clicked.connect(lambda: self.setGripperAndMove(0))

        self.SerialPortRefreshButton.pressed.connect(self.getSerialPorts)
        self.ConnectButton.pressed.connect(self.connectSerial)

        # Simulation mode checkbox - enables direct visualization without serial connection
        self.SimulationModeCheckBox.setChecked(config.USE_SIMULATION_MODE)
        self.SimulationModeCheckBox.toggled.connect(self._onSimulationModeToggled)
        self.SimulationModeCheckBox.setToolTip("Move robot in visualization without hardware connection")

        # Connect console input signals after replacing with HistoryLineEdit
        self.ConsoleButtonSend.pressed.connect(self.sendSerialCommand)
        self.ConsoleInput.returnPressed.connect(self.sendSerialCommand)
        self.ConsoleClearButton.pressed.connect(self.clearConsole)

        # Quick command buttons on terminal tab
        self.QuickM114Button.pressed.connect(lambda: self.sendQuickCommand("M114"))
        self.QuickM119Button.pressed.connect(lambda: self.sendQuickCommand("M119"))
        self.QuickG28Button.pressed.connect(lambda: self.sendQuickCommand("G28"))
        self.QuickG92Button.pressed.connect(lambda: self.sendQuickCommand("G92 X0 Y0 Z0 U0 V0 W0"))

        # IK Control connections with debounce (Cartesian spinbox changes → live IK preview)
        self.ik_calc_timer = QtCore.QTimer()
        self.ik_calc_timer.setSingleShot(True)
        self.ik_calc_timer.timeout.connect(self._calculateIKDeferred)

        self.IKInputSpinBoxX.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxY.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxZ.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxA.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxB.valueChanged.connect(lambda: self.ik_calc_timer.start(50))
        self.IKInputSpinBoxC.valueChanged.connect(lambda: self.ik_calc_timer.start(50))

        # Control panel: mode toggle, action buttons
        self._control_mode = "cartesian"  # Default mode
        self.control_panel.controlModeChanged.connect(self._onControlModeChanged)
        self.GoToButton.pressed.connect(self.goToTarget)
        self.GetCurrentButton.pressed.connect(self.getCurrentPosition)

        # Initialise Frame Manager for coordinate frame transformations
        self.frame_manager = FrameManager()
        logger.info(f"Frame manager initialised with frames: {self.frame_manager.list_frames()}")

        # Initialise IK Controller with callbacks and frame manager
        self.ik_controller = IKController(
            output_update_callback=self._onIKOutputUpdate,
            spinbox_update_callback=self._onIKSpinboxUpdate,
            style_update_callback=self._onIKStyleUpdate,
            move_callback=self.FKMoveAll,
            frame_manager=self.frame_manager
        )

        # Initialise FK Controller with callbacks
        self.fk_controller = FKController(
            robot_controller=self.robot_controller,
            command_sender=None,  # Will be set after full init
            spinbox_update_callback=self._onFKSpinboxUpdate,
            slider_update_callback=self._onFKSliderUpdate,
            visualization_update_callback=self._onFKVisualizationUpdate,
            get_movement_params_callback=lambda: CommandBuilder.get_movement_params(self),
            no_connection_callback=self.noSerialConnection
        )

        # Sync jog mode state with controllers (setChecked(True) fires before controllers exist)
        self.ik_controller.set_jog_mode(self.jog_mode_enabled)
        self.fk_controller.set_jog_mode(self.jog_mode_enabled)

        # Initialise Gripper Controller with callbacks
        self.gripper_controller = GripperController(
            command_sender=None,  # Will be set after full init
            spinbox_update_callback=self._onGripperSpinboxUpdate,
            slider_update_callback=self._onGripperSliderUpdate,
            no_connection_callback=self.noSerialConnection
        )

        # Initialise UI State Manager with callbacks
        self.ui_state_manager = UIStateManager(
            set_widget_enabled=self._setWidgetEnabled,
            set_widget_style=self._setWidgetStyle,
            set_widget_visible=self._setWidgetVisible,
            set_widget_text=self._setWidgetText
        )

        # Initialise Sequence Controller with callbacks
        self.sequence_controller = SequenceController(
            command_sender=None,  # Will be set after full init
            list_update_callback=self._onSequencePointAdded,
            list_clear_callback=self._onSequenceCleared,
            list_remove_callback=self._onSequencePointRemoved,
            button_state_callback=self._onSequenceButtonStateChanged,
            pause_text_callback=self._onSequencePauseTextChanged,
            simulation_move_callback=self._onSimulationSequenceMove
        )
        # Backward compatibility references
        self.sequence_recorder = self.sequence_controller.recorder
        self.sequence_player = None
        self.is_playing_sequence = False
        self.sequence_timer = QtCore.QTimer()
        self.sequence_timer.timeout.connect(self.updateSequencePlayback)

        # Simulation interpolation state
        self._sim_interp_timer = QtCore.QTimer()
        self._sim_interp_timer.timeout.connect(self._tickSimulationInterpolation)
        self._sim_interp_start = None   # [q1..q6, gripper] start values
        self._sim_interp_end = None     # [q1..q6, gripper] target values
        self._sim_interp_elapsed = 0.0
        self._sim_interp_duration = 0.0

        # Only setup classic GUI sequence controls if not using modern GUI
        # Modern GUI has these controls built into TEACH mode panel
        if not config.USE_MODERN_GUI:
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
        logger.info(f"Position history initialised (max_size={config.POSITION_HISTORY_MAX_SIZE}, sample_rate=1/{config.POSITION_HISTORY_SAMPLE_RATE})")

        # Initialise position display controller with callbacks
        self.position_display_controller = PositionDisplayController(
            robot_controller=self.robot_controller,
            position_history=self.position_history,
            gui_update_callback=self._onPositionUpdate,
            state_update_callback=self._onStateUpdate,
            endstop_update_callback=self._onEndstopUpdate
        )

        # Initialise Serial Response Router with callbacks
        self.serial_response_router = SerialResponseRouter(
            position_handler=self.updateFKPosDisplay,
            endstop_handler=self.updateEndstopDisplay,
            disconnect_handler=self._handleSerialDisconnect,
            get_is_homing=lambda: self.is_homing,
            set_homing_complete=self._onHomingComplete,
            request_position_update=lambda: self.command_sender.send_if_connected("M114"),
            set_sync_pending=lambda: setattr(self, 'sync_commands_pending', True),
            trigger_sync=self._triggerCommandSync
        )

        # Initialise Visualisation Controller with callbacks
        self.visualization_controller = VisualizationController(
            position_history=self.position_history,
            update_canvas_callback=self._updateVisualizationCanvas,
            reset_view_callback=self._resetVisualizationView,
            get_trajectory_enabled=self._getTrajectoryEnabled,
            get_auto_rotate_enabled=self._getAutoRotateEnabled,
            get_time_window=self._getTimeWindow,
            get_display_checkboxes=self._getDisplayCheckboxes,
            reload_dh_parameters=self._reloadDHParameters
        )

        # Initialise Position History Manager with callbacks
        self.position_history_manager = PositionHistoryManager(
            position_history=self.position_history,
            get_save_filename=self._getSaveFilename,
            show_warning=self._showWarningDialog,
            show_info=self._showInfoDialog,
            show_error=self._showErrorDialog,
            confirm_action=self._confirmAction
        )

        # Setup endstop status displays
        self.setupEndstopDisplays()

        # Setup generic increment/decrement connections
        self.setupGenericControls()

        # Connect connection manager signals now that methods are defined
        self.connection_manager.connected.connect(self._onConnectionSuccess)
        self.connection_manager.error.connect(self._onConnectionError)
        self.connection_manager.disconnected.connect(self.serialDisconnected)

        # Set console widget for command sender now that UI is fully initialized
        self.command_sender.console_output = self.ConsoleOutput

        # Set command sender for movement controller now that it's configured
        self.movement_controller.command_sender = self.command_sender

        # Set command sender for sequence controller
        self.sequence_controller.command_sender = self.command_sender

        # Connect teach panel signals (modern GUI only)
        # In legacy GUI these connections are made in setupSequenceControls()
        if config.USE_MODERN_GUI and hasattr(self, 'teach_panel'):
            self.teach_panel.manualPointRequested.connect(self.openAddManualPointDialog)
            self.teach_panel.pointEditRequested.connect(self.openEditPointDialog)
            self.teach_panel.importCsvRequested.connect(self.importCsvSequence)

            # Connect sequence control buttons
            self.sequenceRecordButton.pressed.connect(self.recordSequencePoint)
            self.sequenceDeleteButton.pressed.connect(self.deleteSequencePoint)
            self.sequenceClearButton.pressed.connect(self.clearSequence)
            self.sequencePlayButton.pressed.connect(self.playSequence)
            self.sequencePauseButton.pressed.connect(self.pauseSequence)
            self.sequenceStopButton.pressed.connect(self.stopSequence)
            self.sequenceSaveButton.pressed.connect(self.saveSequence)
            self.sequenceLoadButton.pressed.connect(self.loadSequence)

            # Control panel sequence buttons (share same handlers)
            self.ctrlRecordButton.pressed.connect(self.recordSequencePoint)
            self.ctrlDeleteButton.pressed.connect(self._deleteSequencePointFromCtrl)
            self.ctrlPlayButton.pressed.connect(self.playSequence)
            self.ctrlPauseButton.pressed.connect(self.pauseSequence)
            self.ctrlStopButton.pressed.connect(self.stopSequence)
            self.ctrlSaveButton.pressed.connect(self.saveSequence)
            self.ctrlLoadButton.pressed.connect(self.loadSequence)

            # Sync sequence names between panels
            self.teach_panel.SequenceNameEdit.textChanged.connect(
                lambda t: self.ctrlSequenceNameEdit.setText(t)
                if self.ctrlSequenceNameEdit.text() != t else None
            )
            self.ctrlSequenceNameEdit.textChanged.connect(
                lambda t: self.teach_panel.SequenceNameEdit.setText(t)
                if self.teach_panel.SequenceNameEdit.text() != t else None
            )

        # Set command sender for FK controller
        self.fk_controller.command_sender = self.command_sender

        # Set command sender for gripper controller
        self.gripper_controller.command_sender = self.command_sender

        # ExecuteMovementButton no longer needed - jog mode controls in sidebar

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

        logger.info("Generic increment/decrement controls initialised (using RobotController config)")

        # Initial TCP sync: set IK spinboxes from home position FK
        self._syncIKFromJointAngles(
            self.SpinBoxArt1.value(), self.SpinBoxArt2.value(),
            self.SpinBoxArt3.value(), self.SpinBoxArt4.value(),
            self.SpinBoxArt5.value(), self.SpinBoxArt6.value()
        )

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
            # Cast to int for QSpinBox (gripper), keep float for QDoubleSpinBox
            if isinstance(spinbox, QtWidgets.QSpinBox):
                spinbox.setValue(int(new_value))
            else:
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
        """Generic slider update handler - updates spinbox from slider via FK controller"""
        self.fk_controller.slider_changed(joint_name, value)
        if self.SimulationModeCheckBox.isChecked():
            self._updateSimulationVisualization()

    def FKSpinBoxUpdate(self, joint_name, value):
        """Generic spinbox update handler - updates slider from spinbox via FK controller"""
        self.fk_controller.spinbox_changed(joint_name, value)
        if self.SimulationModeCheckBox.isChecked():
            self._updateSimulationVisualization()

    def FKMoveJoint(self, joint_name):
        """
        Generic joint movement handler - delegates to FK controller.
        Handles simple joints, coupled motors, and differential kinematics.
        In simulation mode, only updates visualization (no serial command).
        """
        if joint_name not in self.joint_spinboxes:
            logger.warning(f"Unknown joint: {joint_name}")
            return

        if self.SimulationModeCheckBox.isChecked():
            self._updateSimulationVisualization()
            return

        joint_value = self.joint_spinboxes[joint_name].value()

        # For differential joints, pass both joint values so the "hold" joint
        # uses the spinbox value (desired state) instead of motor feedback
        # which can be stale due to M114 polling race conditions.
        other_joint_value = None
        if joint_name == 'Art5':
            other_joint_value = self.joint_spinboxes['Art6'].value()
        elif joint_name == 'Art6':
            other_joint_value = self.joint_spinboxes['Art5'].value()

        self.fk_controller.move_joint(joint_name, joint_value, other_joint_value=other_joint_value)

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
        if self.SimulationModeCheckBox.isChecked():
            # In simulation mode, reset all joints to zero
            for spinbox in [self.SpinBoxArt1, self.SpinBoxArt2, self.SpinBoxArt3,
                            self.SpinBoxArt4, self.SpinBoxArt5, self.SpinBoxArt6]:
                spinbox.setValue(0.0)
            self._updateSimulationVisualization()
            return
        if self.command_sender.send_if_connected("G28"):
            # Update button state to indicate homing in progress
            self.is_homing = True
            self.ui_state_manager.update_homing_state(True)

    def sendHomePositionCommand(self):
        """Send command to move all axes to the calibrated home position"""
        hp = config.HOME_POSITION
        if self.SimulationModeCheckBox.isChecked():
            self.SpinBoxArt1.setValue(hp.get('Art1', 0.0))
            self.SpinBoxArt2.setValue(hp.get('Art2', 0.0))
            self.SpinBoxArt3.setValue(hp.get('Art3', 0.0))
            self.SpinBoxArt4.setValue(hp.get('Art4', 0.0))
            self.SpinBoxArt5.setValue(hp.get('Art5', 0.0))
            self.SpinBoxArt6.setValue(hp.get('Art6', 0.0))
            self._updateSimulationVisualization()
            return
        art5 = hp.get('Art5', 0.0)
        art6 = hp.get('Art6', 0.0)
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(art5, art6)
        command = CommandBuilder.build_axis_command(
            "G0",
            {
                "X": hp.get('Art1', 0.0),
                "Y": hp.get('Art2', 0.0),
                "Z": hp.get('Art3', 0.0),
                "U": hp.get('Art4', 0.0),
                "V": motor_v,
                "W": motor_w,
            }
        )
        self.command_sender.send_if_connected(command)

    def sendParkCommand(self):
        """Park the robot: move to park position, close gripper, disable motors"""
        pp = config.PARK_POSITION
        if self.SimulationModeCheckBox.isChecked():
            self.SpinBoxArt1.setValue(pp.get('Art1', 0.0))
            self.SpinBoxArt2.setValue(pp.get('Art2', 0.0))
            self.SpinBoxArt3.setValue(pp.get('Art3', 0.0))
            self.SpinBoxArt4.setValue(pp.get('Art4', 0.0))
            self.SpinBoxArt5.setValue(pp.get('Art5', 0.0))
            self.SpinBoxArt6.setValue(pp.get('Art6', 0.0))
            self.SpinBoxGripper.setValue(0)
            self._updateSimulationVisualization()
            return
        art5 = pp.get('Art5', 0.0)
        art6 = pp.get('Art6', 0.0)
        motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(art5, art6)
        move_command = CommandBuilder.build_axis_command(
            "G0",
            {
                "X": pp.get('Art1', 0.0),
                "Y": pp.get('Art2', 0.0),
                "Z": pp.get('Art3', 0.0),
                "U": pp.get('Art4', 0.0),
                "V": motor_v,
                "W": motor_w,
            }
        )
        self.command_sender.send_if_connected(move_command)
        self.gripper_controller.move_to_preset('closed')
        self.command_sender.send_if_connected("M400")  # Wait for moves to finish
        self.command_sender.send_if_connected("M84")   # Disable motors

    def sendKillAlarmCommand(self):
        """Send M999 command (RRF: Clear emergency stop / reset)"""
        self.command_sender.send_if_connected("M999")

    def sendEmergencyStopCommand(self):
        """Emergency stop - halt movement immediately while keeping motors engaged.

        Sends M410 (quick stop) to freeze motors in place and aborts any running sequence.
        This prevents the robot from collapsing under gravity unlike M112 which disengages motors.
        """
        # Stop any running sequence first
        if self.is_playing_sequence:
            self.sequence_timer.stop()
            self.sequence_controller.stop_playback()
            self.is_playing_sequence = False
            logger.warning("E-STOP: Sequence playback aborted")

        # Send quick stop to freeze motors in current position
        if self.command_sender.send_if_connected("M410"):
            logger.error("E-STOP activated (M410) - Motors frozen in place. Movement halted.")

        # Show message to user
        QtWidgets.QMessageBox.warning(
            None,
            "E-STOP Activated",
            "Emergency Stop activated!\n\n"
            "Motors are frozen in place.\n"
            "All movement has been halted."
        )

    def FeedRateBoxHide(self):
        """Legacy method - feedrate visibility now handled by axis column"""
        pass  # No longer used, feedrate enabled/disabled in AxisControlColumn

    def _onMovementTypeChanged(self, move_type):
        """Handle movement type change from axis column (G0/G1)"""
        logger.debug(f"Movement type changed to: {move_type}")

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

        # Sync with IK controller (guarded - may not exist during init)
        if hasattr(self, 'ik_controller'):
            self.ik_controller.set_jog_mode(enabled)

        # Sync with FK controller (guarded - may not exist during init)
        if hasattr(self, 'fk_controller'):
            self.fk_controller.set_jog_mode(enabled)

        # Update visual feedback (guarded - may not exist during init)
        if hasattr(self, 'ui_state_manager'):
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
        Update visual feedback for jog mode state - delegates to UIStateManager

        Args:
            enabled: True if jog mode is enabled, False otherwise
        """
        self.ui_state_manager.update_jog_mode(enabled)


# OLD FK methods removed - replaced with generic handlers above
    # (FKMoveArt1-6, FKSliderUpdateArt1-6, FKSpinBoxUpdateArt1-6, FKDec/Inc methods)
    # Saved ~290 lines of duplicate code via dynamic signal binding

#FK Every Articulation Functions
    def syncCommandsToActual(self):
        """Sync command spinbox values with actual robot position (e.g., after homing)"""
        try:
            # Get actual positions from robot controller
            positions = self.robot_controller.get_current_positions()
            if positions:
                self.SpinBoxArt1.setValue(positions.get('Art1', 0.0))
                self.SpinBoxArt2.setValue(positions.get('Art2', 0.0))
                self.SpinBoxArt3.setValue(positions.get('Art3', 0.0))
                self.SpinBoxArt4.setValue(positions.get('Art4', 0.0))
                self.SpinBoxArt5.setValue(positions.get('Art5', 0.0))
                self.SpinBoxArt6.setValue(positions.get('Art6', 0.0))
                logger.info("Synced command spinboxes with actual position")
        except Exception as e:
            logger.warning(f"Could not sync commands to actual: {e}")

    def on_dh_parameters_changed(self):
        """Handle DH parameters changed signal - reload FK parameters and update visualization"""
        try:
            import forward_kinematics as fk
            fk.reload_dh_parameters()
            # Exit preview mode since parameters are now saved
            self.visualization_controller.dh_preview_mode = False
            # Force visualization to redraw with new parameters
            if hasattr(self, 'position_canvas') and self.position_canvas:
                self.position_canvas._is_dirty = True
                self._updateSimulationVisualization()
            logger.info("DH parameters saved and reloaded in FK module")
        except Exception as e:
            logger.error(f"Error reloading DH parameters: {e}")

    def on_dh_preview_changed(self):
        """Handle DH preview signal - update visualisation with current DH panel values"""
        try:
            dh_widget = getattr(self, '_dh_widget', None)
            if dh_widget and hasattr(self, 'position_canvas'):
                # Enable DH preview mode to prevent timer-based updates from overwriting
                self.visualization_controller.enter_dh_preview_mode()

                # Get current parameters from DH panel
                params = dh_widget.get_parameters()
                # Update visualisation with preview
                self.position_canvas.preview_dh_parameters(params)
                logger.debug("DH preview updated")
        except Exception as e:
            logger.error(f"Error updating DH preview: {e}")

    def FKMoveAll(self):
        """Move all joints simultaneously - delegates to FK controller.
        In simulation mode, only updates visualization (no serial command)."""
        if self.SimulationModeCheckBox.isChecked():
            self._updateSimulationVisualization()
            return

        # Get joint values from spinboxes
        joint_values = JointValues(
            art1=self.SpinBoxArt1.value(),
            art2=self.SpinBoxArt2.value(),
            art3=self.SpinBoxArt3.value(),
            art4=self.SpinBoxArt4.value(),
            art5=self.SpinBoxArt5.value(),
            art6=self.SpinBoxArt6.value()
        )

        # Delegate to FK controller
        self.fk_controller.move_all(joint_values)

# Gripper Functions - delegate to GripperController
    def MoveGripper(self):
        """Move gripper to specified position - delegates to controller"""
        self.gripper_controller.move(self.SpinBoxGripper.value())

    def SliderUpdateGripper(self):
        """Slider changed - update spinbox via controller"""
        self.gripper_controller.slider_changed(self.SliderGripper.value())

    def SpinBoxUpdateGripper(self):
        """Spinbox changed - update slider via controller"""
        self.gripper_controller.spinbox_changed(self.SpinBoxGripper.value())

    def Dec10Gripper(self):
        self.adjustJointValue('Gripper', -10)

    def Dec1Gripper(self):
        self.adjustJointValue('Gripper', -1)

    def Inc1Gripper(self):
        self.adjustJointValue('Gripper', 1)

    def Inc10Gripper(self):
        self.adjustJointValue('Gripper', 10)

    def adjustGripperValue(self, delta):
        """Adjust gripper value by delta amount - used by axis column +/- buttons"""
        spinbox = self.joint_spinboxes.get('Gripper')
        if not spinbox:
            return
        new_value = max(0, min(100, int(spinbox.value() + delta)))
        spinbox.setValue(new_value)
        # Update axis row display
        if hasattr(self, 'axis_column') and "Gripper" in self.axis_column.rows:
            self.axis_column.rows["Gripper"].set_value(new_value)
        # Move immediately (gripper is always direct-drive, not deferred like joints)
        self.MoveGripper()

    def setGripperAndMove(self, value):
        """Set gripper to specific value and execute movement - delegates to controller"""
        preset = 'closed' if value == 0 else 'open'
        self.gripper_controller.move_to_preset(preset)
        # Update axis row display
        if hasattr(self, 'axis_column') and "Gripper" in self.axis_column.rows:
            self.axis_column.rows["Gripper"].set_value(value)

# Axis Control Column Functions
    def _connectAxisColumnButtons(self):
        """Connect axis column +/- buttons based on current mode"""
        if not hasattr(self, 'axis_column'):
            return

        mode = self.axis_column.get_mode()

        if mode == "joint":
            # Joint mode: J1-J6 control individual joints
            joint_mapping = [("J1", "Art1"), ("J2", "Art2"), ("J3", "Art3"),
                             ("J4", "Art4"), ("J5", "Art5"), ("J6", "Art6")]
            for axis_name, joint_name in joint_mapping:
                if axis_name in self.axis_column.rows:
                    row = self.axis_column.rows[axis_name]
                    # Connect minus button
                    row.minus_btn.pressed.connect(
                        lambda j=joint_name: self.adjustJointValue(j, -self.axis_column.get_step())
                    )
                    # Connect plus button
                    row.plus_btn.pressed.connect(
                        lambda j=joint_name: self.adjustJointValue(j, self.axis_column.get_step())
                    )
        else:
            # Cartesian mode: X, Y, Z, Roll, Pitch, Yaw control via IK
            cartesian_axes = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            for axis_name in cartesian_axes:
                if axis_name in self.axis_column.rows:
                    row = self.axis_column.rows[axis_name]
                    # Connect minus button
                    row.minus_btn.pressed.connect(
                        lambda a=axis_name: self.adjustCartesianValue(a, -self.axis_column.get_step())
                    )
                    # Connect plus button
                    row.plus_btn.pressed.connect(
                        lambda a=axis_name: self.adjustCartesianValue(a, self.axis_column.get_step())
                    )

    def _onAxisControlModeChanged(self, mode):
        """Handle axis control column mode change (joint vs cartesian)"""
        logger.info(f"Axis control mode changed to: {mode}")

        # Reconnect buttons for new mode
        # Note: The old buttons are deleted when rows are recreated,
        # so we just connect the new ones
        self._connectAxisColumnButtons()

        # Update display values for the new mode
        self._updateAxisColumnValues()

    def _updateAxisColumnValues(self):
        """Update axis column value displays based on current mode"""
        if not hasattr(self, 'axis_column'):
            return

        mode = self.axis_column.get_mode()

        if mode == "joint":
            # Update with current joint positions
            # This will be called by the position update callback
            pass
        else:
            # Cartesian mode: Show current TCP position
            # Get current position from IK spinboxes or FK calculation
            if hasattr(self, 'IKInputSpinBoxX'):
                x = self.IKInputSpinBoxX.value()
                y = self.IKInputSpinBoxY.value()
                z = self.IKInputSpinBoxZ.value()
                # Update display labels if they exist
                if "X" in self.axis_column.rows:
                    self.axis_column.rows["X"].set_value(x)
                if "Y" in self.axis_column.rows:
                    self.axis_column.rows["Y"].set_value(y)
                if "Z" in self.axis_column.rows:
                    self.axis_column.rows["Z"].set_value(z)
                # Roll, Pitch, Yaw from A/B/C spinboxes
                orientation_map = {"Roll": "A", "Pitch": "B", "Yaw": "C"}
                for axis_name, spinbox_letter in orientation_map.items():
                    if axis_name in self.axis_column.rows:
                        spinbox = getattr(self, f'IKInputSpinBox{spinbox_letter}', None)
                        val = spinbox.value() if spinbox else 0.0
                        self.axis_column.rows[axis_name].set_value(val)

    def _onCoordinateFrameChanged(self, frame_name):
        """Handle coordinate frame selection change"""
        logger.info(f"Coordinate frame changed to: {frame_name}")

        # Update frame manager if available
        if hasattr(self, 'ik_controller') and self.ik_controller.frame_manager:
            try:
                self.ik_controller.frame_manager.set_active_frame(frame_name)
                logger.info(f"IK controller frame set to: {frame_name}")
            except KeyError:
                logger.warning(f"Frame not found in frame manager: {frame_name}")

        # Recalculate current position in new frame
        self._updateAxisColumnValues()

    def adjustCartesianValue(self, axis, delta):
        """
        Adjust Cartesian position/orientation by delta.
        In jog mode, uses Jacobian-based IK for reliable incremental moves.

        Args:
            axis: Axis name ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
            delta: Amount to add to current value (mm for position, degrees for orientation)
        """
        # Map axis to Cartesian delta index: [dx, dy, dz, droll, dpitch, dyaw]
        axis_map = {'X': 0, 'Y': 1, 'Z': 2, 'Roll': 3, 'Pitch': 4, 'Yaw': 5}
        axis_idx = axis_map.get(axis)

        if axis_idx is None:
            logger.warning(f"Unknown Cartesian axis: {axis}")
            return

        if self.jog_mode_enabled:
            # Jacobian-based jog: compute joint delta directly
            cartesian_delta = [0.0] * 6
            if axis_idx < 3:
                cartesian_delta[axis_idx] = delta  # mm
            else:
                cartesian_delta[axis_idx] = np.radians(delta)  # degrees → radians
            self._executeJacobianJog(cartesian_delta)
        else:
            # Non-jog: update spinbox for display (IK recalculates via debounce timer)
            if axis in ['X', 'Y', 'Z']:
                spinbox = getattr(self, f'IKInputSpinBox{axis}', None)
                if spinbox:
                    spinbox.setValue(spinbox.value() + delta)
            else:
                axis_to_spinbox = {'Roll': 'A', 'Pitch': 'B', 'Yaw': 'C'}
                spinbox_letter = axis_to_spinbox.get(axis)
                if spinbox_letter:
                    spinbox = getattr(self, f'IKInputSpinBox{spinbox_letter}', None)
                    if spinbox:
                        spinbox.setValue(spinbox.value() + delta)

# Inverse Kinematics Functions
    def _calculateIKDeferred(self):
        """
        Calculate 6-DOF inverse kinematics for current target position (debounced).
        Delegates to IKController with position and orientation.
        """
        x = self.IKInputSpinBoxX.value()
        y = self.IKInputSpinBoxY.value()
        z = self.IKInputSpinBoxZ.value()
        a = self.IKInputSpinBoxA.value()  # Roll (degrees)
        b = self.IKInputSpinBoxB.value()  # Pitch (degrees)
        c = self.IKInputSpinBoxC.value()  # Yaw (degrees)

        # Delegate to controller - it will update GUI via callbacks
        self.ik_controller.calculate_and_update(x, y, z, roll_deg=a, pitch_deg=b, yaw_deg=c)

    def adjustIKValue(self, axis, delta):
        """
        Adjust IK input value by a delta.
        In jog mode, uses Jacobian-based IK for reliable incremental moves.

        Args:
            axis: Axis letter ('X', 'Y', 'Z')
            delta: Amount to add to current value (mm)
        """
        if self.jog_mode_enabled:
            # Jacobian-based jog: compute joint delta directly
            axis_map = {'X': 0, 'Y': 1, 'Z': 2}
            axis_idx = axis_map.get(axis, 0)
            cartesian_delta = [0.0] * 6
            cartesian_delta[axis_idx] = delta  # mm
            self._executeJacobianJog(cartesian_delta)
        else:
            # Non-jog: update spinbox for display (IK recalculates via debounce timer)
            spinbox = getattr(self, f'IKInputSpinBox{axis}')
            spinbox.setValue(spinbox.value() + delta)

    def _executeJacobianJog(self, cartesian_delta):
        """
        Execute a Jacobian-based jog move from current joint position.

        Computes joint increment via Jacobian pseudoinverse, updates FK spinboxes,
        syncs IK display, and executes the move.

        Args:
            cartesian_delta: [dx, dy, dz, droll_rad, dpitch_rad, dyaw_rad]
        """
        # Get current joint angles from FK spinboxes
        current_joints = [
            self.SpinBoxArt1.value(), self.SpinBoxArt2.value(),
            self.SpinBoxArt3.value(), self.SpinBoxArt4.value(),
            self.SpinBoxArt5.value(), self.SpinBoxArt6.value()
        ]

        # Compute new joints via Jacobian
        new_joints = self.ik_controller.calculate_jacobian_move(current_joints, cartesian_delta)
        if new_joints is None:
            logger.warning("[JOG MODE] Jacobian IK failed, move not executed")
            return

        # Update FK spinboxes (block signals to avoid cascading updates)
        for spinbox, key in [
            (self.SpinBoxArt1, 'Art1'), (self.SpinBoxArt2, 'Art2'),
            (self.SpinBoxArt3, 'Art3'), (self.SpinBoxArt4, 'Art4'),
            (self.SpinBoxArt5, 'Art5'), (self.SpinBoxArt6, 'Art6'),
        ]:
            spinbox.blockSignals(True)
            spinbox.setValue(new_joints[key])
            spinbox.blockSignals(False)

        # Sync IK display from new joints
        self._syncIKFromJointAngles(
            new_joints['Art1'], new_joints['Art2'], new_joints['Art3'],
            new_joints['Art4'], new_joints['Art5'], new_joints['Art6']
        )

        # Update axis column Cartesian display immediately (don't wait for M114 feedback)
        self._updateAxisColumnValues()

        # Execute the move
        self.FKMoveAll()
        logger.debug(f"[JOG MODE] Jacobian jog executed, delta={cartesian_delta}")

    def _syncIKFromJointAngles(self, q1, q2, q3, q4, q5, q6):
        """
        Compute forward kinematics and update IK spinboxes with current TCP pose.
        Signals are blocked to prevent feedback loops (FK->IK->FK).

        Args:
            q1-q6: Joint angles in degrees
        """
        try:
            # Compute full TCP transform (position + orientation)
            tcp_transform = fk.compute_tcp_transform(q1, q2, q3, q4, q5, q6)
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(tcp_transform)

            # Convert orientation from radians to degrees
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)

            # Update IK spinboxes with signals blocked (no IK recalculation)
            for spinbox, value in [
                (self.IKInputSpinBoxX, x),
                (self.IKInputSpinBoxY, y),
                (self.IKInputSpinBoxZ, z),
                (self.IKInputSpinBoxA, roll_deg),
                (self.IKInputSpinBoxB, pitch_deg),
                (self.IKInputSpinBoxC, yaw_deg),
            ]:
                spinbox.blockSignals(True)
                spinbox.setValue(value)
                spinbox.blockSignals(False)

        except Exception as e:
            logger.error(f"FK->TCP sync failed: {e}")

# Control Panel Functions (Go To / Get Current / Mode Toggle)
    def _onControlModeChanged(self, mode):
        """Handle Cartesian/Joint toggle on the control panel."""
        self._control_mode = mode  # "cartesian" or "joint"
        logger.info(f"Control panel mode changed to: {mode}")

    def goToTarget(self):
        """Move robot to the position specified in the control panel target fields.

        Cartesian mode: solve IK, validate, update FK spinboxes, move.
        Joint mode: read joint targets, update FK spinboxes, move.
        """
        if getattr(self, '_control_mode', 'cartesian') == 'cartesian':
            # Solve IK from Cartesian target inputs
            x = self.IKInputSpinBoxX.value()
            y = self.IKInputSpinBoxY.value()
            z = self.IKInputSpinBoxZ.value()
            a = self.IKInputSpinBoxA.value()
            b = self.IKInputSpinBoxB.value()
            c = self.IKInputSpinBoxC.value()

            result = self.ik_controller.calculate_and_update(
                x, y, z, roll_deg=a, pitch_deg=b, yaw_deg=c
            )
            if not result or not result.valid:
                logger.warning("Go To: IK solution invalid, move not executed")
                return

            # Update FK spinboxes from IK solution (block signals to avoid cascading)
            joint_dict = result.to_dict()
            for spinbox, key in [
                (self.SpinBoxArt1, 'Art1'), (self.SpinBoxArt2, 'Art2'),
                (self.SpinBoxArt3, 'Art3'), (self.SpinBoxArt4, 'Art4'),
                (self.SpinBoxArt5, 'Art5'), (self.SpinBoxArt6, 'Art6'),
            ]:
                spinbox.blockSignals(True)
                spinbox.setValue(joint_dict[key])
                spinbox.blockSignals(False)
        else:
            # Joint mode: read target spinboxes directly
            targets = [
                (self.SpinBoxArt1, self.JointTargetSpinBox1),
                (self.SpinBoxArt2, self.JointTargetSpinBox2),
                (self.SpinBoxArt3, self.JointTargetSpinBox3),
                (self.SpinBoxArt4, self.JointTargetSpinBox4),
                (self.SpinBoxArt5, self.JointTargetSpinBox5),
                (self.SpinBoxArt6, self.JointTargetSpinBox6),
            ]
            for fk_spinbox, target_spinbox in targets:
                fk_spinbox.blockSignals(True)
                fk_spinbox.setValue(target_spinbox.value())
                fk_spinbox.blockSignals(False)

        # Execute the move
        self.FKMoveAll()

    def getCurrentPosition(self):
        """Read current robot position into the control panel target fields."""
        if getattr(self, '_control_mode', 'cartesian') == 'cartesian':
            # Compute FK to get TCP pose, populate Cartesian spinboxes
            self._syncIKFromJointAngles(
                self.SpinBoxArt1.value(), self.SpinBoxArt2.value(),
                self.SpinBoxArt3.value(), self.SpinBoxArt4.value(),
                self.SpinBoxArt5.value(), self.SpinBoxArt6.value()
            )
        else:
            # Copy current FK spinbox values to joint target spinboxes
            targets = [
                (self.JointTargetSpinBox1, self.SpinBoxArt1),
                (self.JointTargetSpinBox2, self.SpinBoxArt2),
                (self.JointTargetSpinBox3, self.SpinBoxArt3),
                (self.JointTargetSpinBox4, self.SpinBoxArt4),
                (self.JointTargetSpinBox5, self.SpinBoxArt5),
                (self.JointTargetSpinBox6, self.SpinBoxArt6),
            ]
            for target_spinbox, fk_spinbox in targets:
                target_spinbox.blockSignals(True)
                target_spinbox.setValue(fk_spinbox.value())
                target_spinbox.blockSignals(False)

        # Also sync gripper value on the control panel
        self.ctrlGripperSpinBox.setValue(self.SpinBoxGripper.value())

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

        logger.info("Sequence recorder controls initialised")

    def setupEndstopDisplays(self):
        """Initialise references to endstop labels (defined in gui.py)"""
        # Endstop labels are inline with articulation controls in gui.py
        # Labels: endstopLabelArt1-6 correspond to axes X, Y, Z, U, V, W
        logger.info("Endstop status displays initialised (inline with articulation controls)")

    def setupPositionHistoryControls(self):
        """Create embedded 3D robot visualisation and controls"""
        from robot_3d_visualizer import Robot3DCanvas

        # Create group box for embedded 3D visualisation (right side of window)
        self.positionHistoryGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.positionHistoryGroupBox.setGeometry(QtCore.QRect(1210, 10, 600, 900))
        self.positionHistoryGroupBox.setTitle("3D Robot Visualisation")

        # Embed 3D matplotlib canvas
        self.position_canvas = Robot3DCanvas(self.positionHistoryGroupBox, width=5.8, height=6.5, dpi=100)
        self.position_canvas.setGeometry(QtCore.QRect(10, 25, 580, 650))

        # Controls panel below 3D visualisation
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

        # Start auto-update timer for 3D visualisation
        # OPTIMISED: matplotlib rendering is expensive
        # Dirty flag pattern in visualiser prevents unnecessary redraws
        self.graph_update_timer = QtCore.QTimer()
        self.graph_update_timer.timeout.connect(self.updateEmbeddedGraph)
        self.graph_update_timer.start(config.GRAPH_UPDATE_INTERVAL_MS)

        logger.info(f"Embedded 3D robot visualisation initialised ({config.GRAPH_UPDATE_INTERVAL_MS}ms update interval)")

    def setupModern3DVisualization(self):
        """Setup 3D visualisation for modern GUI (compact controls in robot state panel)"""
        # Canvas is already created in gui_modern.py and mapped via self.position_canvas
        # Just need to start the update timer

        # Start auto-update timer for 3D visualisation
        # OPTIMIZED: matplotlib rendering is expensive
        # Dirty flag pattern in visualizer prevents unnecessary redraws
        self.graph_update_timer = QtCore.QTimer()
        self.graph_update_timer.timeout.connect(self.updateModern3DVisualization)
        self.graph_update_timer.start(config.GRAPH_UPDATE_INTERVAL_MS)

        # Link calibration panel to GUI instance (if it exists)
        if hasattr(self, 'calibration_panel') and self.calibration_panel:
            self.calibration_panel.gui_instance = self
            logger.info("Calibration panel linked to GUI instance")

        # Connect joint frames checkbox to canvas flag
        if hasattr(self, 'show_joint_frames_check'):
            self.show_joint_frames_check.toggled.connect(self._onJointFramesToggled)

        # Connect DH panel signals (deferred to ensure all widgets are ready)
        # Use QTimer.singleShot to connect after event loop processes pending events
        QtCore.QTimer.singleShot(100, self._connectDHPanelSignals)

        logger.info(f"Modern 3D visualisation initialised ({config.GRAPH_UPDATE_INTERVAL_MS}ms update interval)")

    def _connectDHPanelSignals(self):
        """Connect DH panel signals - deferred to ensure widgets are ready"""
        # DH widget is inside calibration_panel, not a top-level attribute
        dh_widget = None
        if hasattr(self, 'calibration_panel') and hasattr(self.calibration_panel, 'dh_parameters'):
            dh_widget = self.calibration_panel.dh_parameters
        elif hasattr(self, 'dh_panel') and self.dh_panel:
            dh_widget = self.dh_panel

        if dh_widget:
            self._dh_widget = dh_widget
            dh_widget.parameters_changed.connect(self.on_dh_parameters_changed)
            if hasattr(dh_widget, 'preview_changed'):
                dh_widget.preview_changed.connect(self.on_dh_preview_changed)
            logger.info("DH panel signals connected for live preview")
        else:
            logger.warning("DH panel not found, live preview disabled")

        # Connect mode switching to exit DH preview mode when leaving DH panel
        if hasattr(self, 'mode_selector') and self.mode_selector:
            self.mode_selector.mode_changed.connect(self._onModeChanged)

    def _onModeChanged(self, mode_index):
        """Handle mode switching - exit DH preview mode when leaving DH panel"""
        # Delegate to visualisation controller
        self.visualization_controller.on_mode_changed(mode_index, dh_mode_index=5)

    def updateModern3DVisualization(self):
        """Update the modern GUI 3D visualisation - delegates to VisualizationController"""
        if not hasattr(self, 'position_canvas'):
            logger.warning("updateModern3DVisualization: position_canvas not found")
            return

        self.visualization_controller.update_modern_visualization()

    def updateEmbeddedGraph(self):
        """Update the embedded 3D robot visualisation - delegates to VisualizationController"""
        if not hasattr(self, 'position_canvas'):
            return

        self.visualization_controller.update_embedded_visualization()

    def resetVisualizationView(self):
        """Reset 3D visualisation view to default - delegates to VisualizationController"""
        self.visualization_controller.reset_view()

    def exportPositionHistory(self):
        """Export position history to CSV - delegates to PositionHistoryManager"""
        self.position_history_manager.export_to_csv()

    def clearPositionHistory(self):
        """Clear position history after confirmation - delegates to PositionHistoryManager"""
        self.position_history_manager.clear_history()

    def recordSequencePoint(self):
        """Record current joint positions to sequence. Delegates to SequenceController."""
        positions = JointPositions(
            q1=self.SpinBoxArt1.value(),
            q2=self.SpinBoxArt2.value(),
            q3=self.SpinBoxArt3.value(),
            q4=self.SpinBoxArt4.value(),
            q5=self.SpinBoxArt5.value(),
            q6=self.SpinBoxArt6.value(),
            gripper=self.SpinBoxGripper.value()
        )

        # Read delay from the active panel's delay spinbox
        if hasattr(self, 'mode_stack') and self.mode_stack.currentIndex() == 0:
            delay = self.ctrlDelaySpinBox.value()
        else:
            delay = self.sequenceDelaySpinBox.value()

        # Update movement params from GUI before recording
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        self.sequence_controller.set_movement_params(movement_type, feedrate)

        self.sequence_controller.record_point(positions, delay)

    def deleteSequencePoint(self):
        """Delete selected point from teach panel sequence list."""
        current_row = self.sequencePointsList.currentRow()
        self.sequence_controller.delete_point(current_row)

    def _deleteSequencePointFromCtrl(self):
        """Delete selected point from control panel sequence list."""
        current_row = self.ctrlSequencePointsList.currentRow()
        self.sequence_controller.delete_point(current_row)

    def clearSequence(self):
        """Clear all points from sequence. Delegates to SequenceController."""
        self.sequence_controller.clear_sequence()

    def playSequence(self):
        """Play the recorded sequence. Delegates to SequenceController."""
        # Update movement params from GUI before playback
        movement_type, feedrate = CommandBuilder.get_movement_params(self)
        self.sequence_controller.set_movement_params(movement_type, feedrate)

        # Get playback parameters from the active panel
        if hasattr(self, 'mode_stack') and self.mode_stack.currentIndex() == 0:
            speed = self.ctrlSpeedSpinBox.value()
            loop = self.ctrlLoopCheckBox.isChecked()
        else:
            speed = self.sequenceSpeedSpinBox.value()
            loop = self.sequenceLoopCheckBox.isChecked()

        if self.sequence_controller.start_playback(speed=speed, loop=loop):
            self.is_playing_sequence = True
            self.sequence_timer.start(config.SEQUENCE_TIMER_INTERVAL)

    def updateSequencePlayback(self):
        """Called by QTimer to advance sequence playback."""
        should_continue, current, total = self.sequence_controller.update_playback()

        if not should_continue:
            self.sequence_timer.stop()
            self.is_playing_sequence = False

    def pauseSequence(self):
        """Pause/resume sequence playback. Delegates to SequenceController."""
        self.sequence_controller.pause_playback()

    def stopSequence(self):
        """Stop sequence playback. Delegates to SequenceController."""
        self.sequence_timer.stop()
        self._sim_interp_timer.stop()
        self.sequence_controller.stop_playback()
        self.is_playing_sequence = False

    def saveSequence(self):
        """Save sequence to file."""
        if self.sequence_controller.sequence_length == 0:
            logger.warning("Cannot save empty sequence")
            return

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Save Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            if self.sequence_controller.save_sequence(filename):
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
        """Load sequence from file."""
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Load Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            sequence = self.sequence_controller.load_sequence(filename)
            if sequence:
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

    def openAddManualPointDialog(self):
        """Open dialog to manually add a sequence point."""
        if PointEditDialog is None:
            logger.warning("Manual point dialog not available in legacy GUI")
            return

        dialog = PointEditDialog(None)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            point_data = dialog.get_point_data()
            self.sequence_controller.add_manual_point(point_data)
            logger.info(f"Added manual point: {point_data}")

    def openEditPointDialog(self, index):
        """Open dialog to edit an existing sequence point."""
        if PointEditDialog is None:
            logger.warning("Edit point dialog not available in legacy GUI")
            return

        point_data = self.sequence_controller.get_point_data(index)
        if point_data is None:
            logger.warning(f"No point found at index {index}")
            return

        dialog = PointEditDialog(None, point_data=point_data, point_index=index)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            new_data = dialog.get_point_data()
            self.sequence_controller.update_point(index, new_data)
            logger.info(f"Updated point {index + 1}: {new_data}")

    def importCsvSequence(self):
        """Import sequence points from a CSV file."""
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Import CSV Sequence",
            "",
            "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            success, message = self.sequence_controller.import_csv(filename)
            msgBox = QtWidgets.QMessageBox()
            if success:
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setWindowTitle("Import Successful")
            else:
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setWindowTitle("Import Failed")
            msgBox.setText(message)
            msgBox.exec_()

# Serial Connection functions
    def getSerialPorts(self):
        """Refresh available serial ports in combo box"""
        self.SerialPortComboBox.clear()

        # In simulation mode, show "SIMULATION" as the only option
        if self.SimulationModeCheckBox.isChecked():
            self.SerialPortComboBox.addItem("SIMULATION")
            self.SerialPortComboBox.setEnabled(False)  # Disable selection in simulation mode
            logger.info("Simulation mode: port selection disabled")
        else:
            available_ports = self.connection_manager.get_available_ports()
            self.SerialPortComboBox.addItems(available_ports)

            # Auto-detect and select the robot's COM port
            robot_port = self.connection_manager.get_recommended_port()
            if robot_port:
                index = self.SerialPortComboBox.findText(robot_port)
                if index >= 0:
                    self.SerialPortComboBox.setCurrentIndex(index)
                    logger.info(f"Auto-detected robot port: {robot_port}")
                else:
                    logger.warning(f"Detected port {robot_port} not found in combo box")
            else:
                logger.info("No robot port auto-detected, please select manually")

    def _onSimulationModeToggled(self, enabled):
        """Handle simulation mode checkbox toggle"""
        self.sequence_controller.simulation_mode = enabled
        if enabled:
            logger.info("Simulation mode enabled - direct visualization control")
            self.SerialPortComboBox.setEnabled(False)
            self.SerialPortRefreshButton.setEnabled(False)
            # Show current spinbox positions in visualization immediately
            self._updateSimulationVisualization()
        else:
            logger.info("Simulation mode disabled - serial connection required")
            self.SerialPortComboBox.setEnabled(True)
            self.SerialPortRefreshButton.setEnabled(True)
        self.getSerialPorts()

    def _updateSimulationVisualization(self):
        """Update 3D visualization and axis column displays from current spinbox values (no connection needed)"""
        joint_angles = [
            self.SpinBoxArt1.value(),
            self.SpinBoxArt2.value(),
            self.SpinBoxArt3.value(),
            self.SpinBoxArt4.value(),
            self.SpinBoxArt5.value(),
            self.SpinBoxArt6.value()
        ]

        # Update 3D visualization
        if hasattr(self, 'position_canvas') and self.position_canvas:
            self.position_canvas.update_robot(joint_angles)

        # Update axis column value labels so the sidebar reflects current positions
        if hasattr(self, 'axis_column'):
            joint_mapping = [("J1", 0), ("J2", 1), ("J3", 2),
                             ("J4", 3), ("J5", 4), ("J6", 5)]
            for axis_name, idx in joint_mapping:
                if axis_name in self.axis_column.rows:
                    self.axis_column.rows[axis_name].set_value(joint_angles[idx])

            # Update gripper display too
            if "Gripper" in self.axis_column.rows:
                self.axis_column.rows["Gripper"].set_value(self.SpinBoxGripper.value())

        # Continuous TCP tracking: compute FK and update IK spinboxes
        self._syncIKFromJointAngles(*joint_angles)

    def _onSimulationSequenceMove(self, q1, q2, q3, q4, q5, q6, gripper, duration):
        """Start interpolated movement to target position in simulation mode."""
        SIM_INTERP_FPS = 30
        self._sim_interp_start = [
            self.SpinBoxArt1.value(), self.SpinBoxArt2.value(), self.SpinBoxArt3.value(),
            self.SpinBoxArt4.value(), self.SpinBoxArt5.value(), self.SpinBoxArt6.value(),
            self.SpinBoxGripper.value()
        ]
        self._sim_interp_end = [q1, q2, q3, q4, q5, q6, gripper]
        self._sim_interp_elapsed = 0.0
        # Use 80% of the delay so animation finishes before the next point fires
        self._sim_interp_duration = max(duration * 0.8, 0.1)
        interval_ms = int(1000 / SIM_INTERP_FPS)
        self._sim_interp_timer.start(interval_ms)

    def _tickSimulationInterpolation(self):
        """Advance one frame of simulation interpolation."""
        SIM_INTERP_FPS = 30
        dt = 1.0 / SIM_INTERP_FPS
        self._sim_interp_elapsed += dt
        t = min(self._sim_interp_elapsed / self._sim_interp_duration, 1.0)
        # Smooth-step for nicer easing
        t = t * t * (3.0 - 2.0 * t)

        spinboxes = [
            self.SpinBoxArt1, self.SpinBoxArt2, self.SpinBoxArt3,
            self.SpinBoxArt4, self.SpinBoxArt5, self.SpinBoxArt6,
            self.SpinBoxGripper
        ]
        for i, spinbox in enumerate(spinboxes):
            val = self._sim_interp_start[i] + t * (self._sim_interp_end[i] - self._sim_interp_start[i])
            spinbox.setValue(val)

        self._updateSimulationVisualization()

        if t >= 1.0:
            self._sim_interp_timer.stop()

    def connectSerial(self):
        """Connect to or disconnect from serial port"""
        # Check if already connected - if so, disconnect
        if self.connection_manager.is_connected:
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

        # Use ConnectionManager to connect (runs in background thread)
        self.connection_manager.connect(serialPort, baudrate, gui_instance=self)

    def disconnectSerial(self):
        """Disconnect from serial port"""
        try:
            # Use ConnectionManager to disconnect (handles thread cleanup)
            self.connection_manager.disconnect()

            # Update GUI
            self.ConnectButton.setText("Connect")
            self.ConnectButton.setEnabled(True)

        except Exception as e:
            logger.error(f"Error during disconnect: {e}")

    def _onConnectionSuccess(self, serialPort, baudrate):
        """Called when connection succeeds (runs in GUI thread)"""
        # Get serial thread from ConnectionManager and store reference
        self.SerialThreadClass = self.connection_manager.get_serial_thread()

        # Connect serial signal in GUI thread (critical for Qt signals to work)
        if self.SerialThreadClass:
            self.SerialThreadClass.serialSignal.connect(self.updateConsole)

        # Update GUI to show connected state
        self.updateCurrentState("Idle")
        self.ConnectButton.setText("Disconnect")
        self.ConnectButton.setEnabled(True)

        # Request initial position after thread is ready
        QtCore.QTimer.singleShot(50, self.requestInitialPosition)

        # Sync motor directions from config.g to firmware
        QtCore.QTimer.singleShot(100, self._syncMotorDirections)

        # Display appropriate connection message
        if self.SimulationModeCheckBox.isChecked():
            logger.info("=== CONNECTED TO SIMULATED ROBOT ===")
            logger.info("Simulation mode active - no hardware required")
            # Update console to show simulation mode
            self.ConsoleOutput.appendPlainText("=== SIMULATION MODE ACTIVE ===")
            self.ConsoleOutput.appendPlainText("No hardware connected - using simulated robot")
        else:
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
        self.connection_manager.request_position_update()

    def _syncMotorDirections(self):
        """Send M569 commands to firmware to match saved config.g motor directions."""
        try:
            directions = read_m569_directions(DEFAULT_CONFIG_G_PATH)
            if not directions:
                logger.warning("No M569 directions found in config.g")
                return

            for drive, s_value in sorted(directions.items()):
                cmd = f"M569 P{drive} S{s_value}"
                self.command_sender.send_if_connected(cmd, show_in_console=False)

            logger.info(f"Synced {len(directions)} motor directions from config.g")
        except Exception as e:
            logger.error(f"Failed to sync motor directions: {e}")

    def serialDisconnected(self):
        """Handle serial disconnection - delegates to UIStateManager"""
        self.ui_state_manager.update_connection_state(UIConnectionState.DISCONNECTED)

    # Callback methods for SerialResponseRouter
    def _handleSerialDisconnect(self):
        """Callback for serial disconnection from router"""
        s0.close()
        self.serialDisconnected()

    def _onHomingComplete(self):
        """Callback when homing cycle completes"""
        self.is_homing = False
        # Defer W alignment dialog so serial response processing finishes first
        QTimer.singleShot(0, self._showWAlignmentDialog)

    def _showWAlignmentDialog(self):
        """Show dialog for manual J6 alignment, then finalize homing.

        The dialog jogs V+W equally for pure J6 rotation.  After jogging
        by d, motors are at V=d, W=d but physically J5=0, J6=d.
        Re-zeroing both with G92 V0 W0 maps J5=0,J6=0 to the current
        physical state.  V's endstop reference shifts by d (small,
        corrected on next homing).
        """
        dialog = WAlignmentDialog(self.command_sender)
        dialog.exec_()
        # Re-zero both motors at current position (no physical movement)
        self.command_sender.send_if_connected("G92 V0 W0")
        # Finalize homing
        self.ui_state_manager.update_homing_state(False)
        self.robot_controller.reset_position_tracking()
        # Request fresh position to sync displays
        self.command_sender.send_if_connected("M114")
        self.sync_commands_pending = True

    def _triggerCommandSync(self):
        """Callback to trigger command sync to actual positions"""
        if getattr(self, 'sync_commands_pending', False):
            self.sync_commands_pending = False
            self.syncCommandsToActual()

    def updateConsole(self, dataRead):
        """
        Route serial response to appropriate handler.

        Delegates to SerialResponseRouter for response identification and routing.
        Handles console display based on routing result.
        """
        verbose_show = self.ConsoleShowVerbosecheckBox.isChecked()
        ok_show = self.ConsoleShowOkRespcheckBox.isChecked()
        sync_pending = getattr(self, 'sync_commands_pending', False)

        # Route response through the router
        result = self.serial_response_router.route_response(
            dataRead,
            verbose_show=verbose_show,
            ok_show=ok_show,
            sync_pending=sync_pending
        )

        # Display in console if router says so
        if result.show_in_console:
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
            # Notify router about manual command (for console display timing)
            self.serial_response_router.mark_manual_command_sent()
            logger.debug(f"Manual command sent: {command}")

    def clearConsole(self):
        """Clear the console output"""
        self.ConsoleOutput.clear()

    def sendQuickCommand(self, command):
        """Send a quick command from terminal tab buttons"""
        if self.command_sender.send_if_connected(command, error_callback=self.noSerialConnection):
            self.serial_response_router.mark_manual_command_sent()
            logger.debug(f"Quick command sent: {command}")

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

    # Callback methods for PositionDisplayController
    def _onPositionUpdate(self, positions):
        """Callback to update GUI position labels and sync IK spinboxes to current TCP"""
        joint_angles = [
            positions['X'], positions['Y'], positions['Z'],
            positions['U'], positions['Art5'], positions['Art6']
        ]

        # Update axis column rows (dynamically looked up to survive mode switches)
        if hasattr(self, 'axis_column'):
            joint_mapping = [("J1", 0), ("J2", 1), ("J3", 2),
                             ("J4", 3), ("J5", 4), ("J6", 5)]
            for axis_name, idx in joint_mapping:
                if axis_name in self.axis_column.rows:
                    self.axis_column.rows[axis_name].set_value(joint_angles[idx])

        # Continuous TCP tracking: compute FK and update IK spinboxes
        self._syncIKFromJointAngles(*joint_angles)

        # Update axis column if in Cartesian mode (rows are X/Y/Z/Roll/Pitch/Yaw, not J1-J6)
        if hasattr(self, 'axis_column') and self.axis_column.get_mode() != "joint":
            self._updateAxisColumnValues()

    def _onStateUpdate(self, state, color):
        """Callback to update robot state display"""
        self.RobotStateDisplay.setText(state)
        self.RobotStateDisplay.setStyleSheet(f'background-color: {color}')

    def _onEndstopUpdate(self, axis, text, style):
        """Callback to update endstop label/indicator"""
        if not hasattr(self, 'axis_column'):
            return

        # Map firmware axis letter to joint row name
        axis_to_row = {
            'X': 'J1', 'Y': 'J2', 'Z': 'J3',
            'U': 'J4', 'V': 'J5', 'W': 'J6'
        }
        row_name = axis_to_row.get(axis)
        if not row_name or row_name not in self.axis_column.rows:
            return

        indicator = self.axis_column.rows[row_name].endstop_indicator
        is_triggered = "255, 200, 200" in style or "triggered" in text.lower()
        if is_triggered:
            indicator.setStyleSheet("font-size: 7pt; color: #f44336;")  # Red
        else:
            indicator.setStyleSheet("font-size: 7pt; color: #4CAF50;")  # Green

    # Callback methods for SequenceController (dual-panel: teach + control)
    def _onSequencePointAdded(self, point_text):
        """Callback when a point is added to the sequence - updates both panels"""
        self.sequencePointsList.addItem(point_text)
        self.ctrlSequencePointsList.addItem(point_text)

    def _onSequenceCleared(self):
        """Callback when sequence is cleared - updates both panels"""
        self.sequencePointsList.clear()
        self.ctrlSequencePointsList.clear()

    def _onSequencePointRemoved(self, index):
        """Callback when a point is removed from sequence - updates both panels"""
        self.sequencePointsList.takeItem(index)
        self.ctrlSequencePointsList.takeItem(index)

    def _onSequenceButtonStateChanged(self, button_name, enabled):
        """Callback to update sequence button states on both panels"""
        teach_map = {
            'play': self.sequencePlayButton,
            'pause': self.sequencePauseButton,
            'stop': self.sequenceStopButton
        }
        ctrl_map = {
            'play': self.ctrlPlayButton,
            'pause': self.ctrlPauseButton,
            'stop': self.ctrlStopButton
        }
        if button_name in teach_map:
            teach_map[button_name].setEnabled(enabled)
        if button_name in ctrl_map:
            ctrl_map[button_name].setEnabled(enabled)

    def _onSequencePauseTextChanged(self, text):
        """Callback to update pause button text on both panels"""
        self.sequencePauseButton.setText(text)
        self.ctrlPauseButton.setText(text)

    # Callback methods for IKController
    def _onIKOutputUpdate(self, x_text, y_text, z_text):
        """Callback to update IK output labels"""
        self.IkOutputValueX.setText(x_text)
        self.IkOutputValueY.setText(y_text)
        self.IkOutputValueZ.setText(z_text)

    def _onIKSpinboxUpdate(self, joint_values):
        """Callback to update FK spinboxes from IK solution.

        Only updates FK spinboxes in jog mode (where the move will be executed
        immediately). In non-jog mode, the IK result is display-only via the
        output labels — FK spinboxes must reflect the actual robot position to
        prevent jumps when switching to Jacobian jog.
        """
        if not self.jog_mode_enabled:
            # Non-jog: don't touch FK spinboxes, IK result shown in output labels only
            return

        self.SpinBoxArt1.setValue(joint_values['Art1'])
        self.SpinBoxArt2.setValue(joint_values['Art2'])
        self.SpinBoxArt3.setValue(joint_values['Art3'])
        self.SpinBoxArt4.setValue(joint_values['Art4'])
        self.SpinBoxArt5.setValue(joint_values['Art5'])
        self.SpinBoxArt6.setValue(joint_values['Art6'])

    def _onIKStyleUpdate(self, style):
        """Callback to update IK output frame style"""
        self.IkOutputValueFrame.setStyleSheet(style)

    # Callback methods for FKController
    def _onFKSpinboxUpdate(self, joint_name, value):
        """Callback to update a spinbox from FK controller"""
        if joint_name in self.joint_spinboxes:
            self.joint_spinboxes[joint_name].setValue(value)

    def _onFKSliderUpdate(self, joint_name, slider_value):
        """Callback to update a slider from FK controller"""
        slider = getattr(self, f'FKSlider{joint_name}', None)
        if slider:
            slider.setValue(slider_value)

    def _onFKVisualizationUpdate(self, joint_angles):
        """Callback to update 3D visualisation from FK controller"""
        if hasattr(self, 'position_canvas') and self.position_canvas:
            self.position_canvas.update_robot(joint_angles)

    # Callback methods for GripperController
    def _onGripperSpinboxUpdate(self, value):
        """Callback to update gripper spinbox from controller"""
        self.SpinBoxGripper.setValue(int(value))

    def _onGripperSliderUpdate(self, value):
        """Callback to update gripper slider from controller"""
        self.SliderGripper.setValue(value)

    # Callback methods for UIStateManager
    def _setWidgetEnabled(self, widget_name, enabled):
        """Callback to set widget enabled state"""
        widget = getattr(self, widget_name, None)
        if widget:
            widget.setEnabled(enabled)

    def _setWidgetStyle(self, widget_name, style):
        """Callback to set widget stylesheet"""
        widget = getattr(self, widget_name, None)
        if widget:
            widget.setStyleSheet(style)

    def _setWidgetVisible(self, widget_name, visible):
        """Callback to set widget visibility"""
        widget = getattr(self, widget_name, None)
        if widget and hasattr(widget, 'setVisible'):
            widget.setVisible(visible)

    def _setWidgetText(self, widget_name, text):
        """Callback to set widget text"""
        widget = getattr(self, widget_name, None)
        if widget and hasattr(widget, 'setText'):
            widget.setText(text)

    # Callback methods for VisualizationController
    def _updateVisualizationCanvas(self, history, window_size, options):
        """Callback to update 3D visualisation canvas"""
        if hasattr(self, 'position_canvas') and self.position_canvas:
            try:
                self.position_canvas.update_visualization(history, window_size, options)
            except RuntimeError as e:
                logger.debug(f"OpenGL context error (safe to ignore): {e}")
            except Exception as e:
                logger.error(f"Error updating 3D canvas: {e}")

    def _resetVisualizationView(self):
        """Callback to reset 3D view to default"""
        if hasattr(self, 'position_canvas') and self.position_canvas:
            self.position_canvas.reset_view()

    def _getTrajectoryEnabled(self):
        """Callback to get trajectory checkbox state"""
        if hasattr(self, 'show_trajectory_check'):
            return self.show_trajectory_check.isChecked()
        return False

    def _onJointFramesToggled(self, enabled):
        """Handle joint frames checkbox toggle"""
        if hasattr(self, 'position_canvas') and self.position_canvas:
            self.position_canvas.show_joint_frames = enabled
            # Re-render immediately with current joint angles
            joint_angles = [
                self.SpinBoxArt1.value(), self.SpinBoxArt2.value(), self.SpinBoxArt3.value(),
                self.SpinBoxArt4.value(), self.SpinBoxArt5.value(), self.SpinBoxArt6.value()
            ]
            self.position_canvas.update_robot(joint_angles)

    def _getAutoRotateEnabled(self):
        """Callback to get auto-rotate checkbox state"""
        if hasattr(self, 'auto_rotate_check'):
            return self.auto_rotate_check.isChecked()
        return False

    def _getTimeWindow(self):
        """Callback to get time window value"""
        if hasattr(self, 'timeWindowSpinBox'):
            return self.timeWindowSpinBox.value()
        return 60

    def _getDisplayCheckboxes(self):
        """Callback to get all display checkbox states"""
        if hasattr(self, 'displayCheckboxes'):
            return {key: cb.isChecked() for key, cb in self.displayCheckboxes.items()}
        return {}

    def _reloadDHParameters(self):
        """Callback to reload DH parameters from file"""
        import forward_kinematics as fk
        fk.reload_dh_parameters()

    # Callback methods for PositionHistoryManager
    def _getSaveFilename(self, default_name):
        """Callback to get save filename via file dialog"""
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Export Position History",
            default_name,
            "CSV Files (*.csv);;All Files (*)"
        )
        return filename if filename else None

    def _showWarningDialog(self, message, title):
        """Callback to show warning dialog"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setWindowTitle(title)
        msgBox.setText(message)
        msgBox.exec_()

    def _showInfoDialog(self, message, title):
        """Callback to show info dialog"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Information)
        msgBox.setWindowTitle(title)
        msgBox.setText(message)
        msgBox.exec_()

    def _showErrorDialog(self, message, title):
        """Callback to show error dialog"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Critical)
        msgBox.setWindowTitle(title)
        msgBox.setText(message)
        msgBox.exec_()

    def _confirmAction(self, message, title):
        """Callback to confirm action with Yes/No dialog"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Question)
        msgBox.setWindowTitle(title)
        msgBox.setText(message)
        msgBox.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msgBox.setDefaultButton(QtWidgets.QMessageBox.No)
        return msgBox.exec_() == QtWidgets.QMessageBox.Yes

    def updateFKPosDisplay(self, dataRead):
        """
        Parse M114 position response and update all displays.

        Delegates to PositionDisplayController for processing.
        Maintains backward compatibility references for existing code.

        Args:
            dataRead: Raw M114 response string from firmware
        """
        # Delegate to controller
        positions = self.position_display_controller.process_m114_response(dataRead)

        # Update backward compatibility references from controller
        if positions:
            self.current_motor_v = self.position_display_controller.current_motor_v
            self.current_motor_w = self.position_display_controller.current_motor_w
            self.desired_art5 = self.position_display_controller.desired_art5
            self.desired_art6 = self.position_display_controller.desired_art6
            self.position_update_count = self.position_display_controller.position_update_count

    def updateEndstopDisplay(self, dataRead):
        """
        Parse M119 endstop response and update GUI displays.

        Delegates to PositionDisplayController for processing.

        Args:
            dataRead: Raw M119 response string from firmware
        """
        self.position_display_controller.process_m119_response(dataRead)

    def updateCurrentState(self, state):
        """
        Update robot state display with appropriate color coding.

        Delegates to PositionDisplayController.

        Args:
            state: State string (Idle, Run, Home, Alarm, Hold)
        """
        self.position_display_controller.update_state(state)



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
# SerialThreadClass uses the global s0 SerialManager instance
# The actual implementation is in serial_thread.py (SerialThread class)

class SerialThreadClass(SimulatedSerialThread if config.USE_SIMULATION_MODE else SerialThread):
    """
    Backward-compatible wrapper around SerialThread or SimulatedSerialThread.
    Uses the global s0 SerialManager instance.
    """

    def __init__(self, gui_instance=None, parent=None):
        """
        Initialize serial thread with global serial manager.

        Args:
            gui_instance: Reference to GUI (kept for backward compatibility)
            parent: Optional Qt parent object
        """
        super().__init__(serial_manager=s0, parent=parent)
        self.gui_instance = gui_instance  # Kept for backward compatibility


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
