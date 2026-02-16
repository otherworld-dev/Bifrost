"""
Robot Calibration Panel
Consolidates all calibration settings:
- Motor direction verification
- Gripper PWM calibration
- DH parameters editing
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from pathlib import Path
import json
import logging
from forward_kinematics import get_dh_params, reload_dh_parameters
from config_g_manager import JOINT_TO_DRIVES, get_joint_directions, set_joint_direction
import config

logger = logging.getLogger(__name__)

# DH parameters file path
DH_PARAMS_FILE = Path(__file__).parent / 'dh_parameters.json'
GRIPPER_CALIBRATION_FILE = Path(__file__).parent / 'gripper_calibration.json'


def load_gripper_calibration_on_startup():
    """
    Load gripper calibration from file and apply to config module.
    Call this at application startup to ensure settings are loaded
    before the calibration panel is opened.
    """
    try:
        if GRIPPER_CALIBRATION_FILE.exists():
            with open(GRIPPER_CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
            config.GRIPPER_PWM_OPEN = data.get('pwm_open', 255)
            config.GRIPPER_PWM_CLOSED = data.get('pwm_closed', 0)
            logger.info(f"Loaded gripper calibration: open={config.GRIPPER_PWM_OPEN}, closed={config.GRIPPER_PWM_CLOSED}")
        else:
            logger.debug("No gripper calibration file found, using defaults")
    except Exception as e:
        logger.error(f"Error loading gripper calibration on startup: {e}")


class JointCalibrationWidget(QtWidgets.QWidget):
    """Widget for verifying direction of a single joint"""

    test_movement = QtCore.pyqtSignal(str, float)  # joint_name, delta_angle
    direction_changed = QtCore.pyqtSignal(str, int)  # joint_name, direction (+1 or -1)

    def __init__(self, joint_name, joint_description, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.joint_description = joint_description
        self.current_direction = 1

        self.setup_ui()

    def setup_ui(self):
        """Create UI elements for this joint"""
        # Main layout
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # Frame for controls
        frame = QtWidgets.QFrame()
        frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        frame_layout = QtWidgets.QGridLayout(frame)
        frame_layout.setContentsMargins(10, 8, 10, 8)

        # Row 0: Joint name and description + Direction control
        header_label = QtWidgets.QLabel(f"<b>{self.joint_name}</b>")
        header_label.setStyleSheet("font-size: 13px;")
        frame_layout.addWidget(header_label, 0, 0)

        desc_label = QtWidgets.QLabel(self.joint_description)
        desc_label.setStyleSheet("color: gray; font-size: 10px;")
        frame_layout.addWidget(desc_label, 0, 1)

        # Direction control on same row
        direction_layout = QtWidgets.QHBoxLayout()
        self.direction_button_group = QtWidgets.QButtonGroup(self)

        self.forward_radio = QtWidgets.QRadioButton("Forward")
        self.forward_radio.setChecked(True)
        self.direction_button_group.addButton(self.forward_radio, 1)
        direction_layout.addWidget(self.forward_radio)

        self.reverse_radio = QtWidgets.QRadioButton("Reverse")
        self.direction_button_group.addButton(self.reverse_radio, -1)
        direction_layout.addWidget(self.reverse_radio)

        frame_layout.addLayout(direction_layout, 0, 2)

        # Row 1: Test movement buttons
        frame_layout.addWidget(QtWidgets.QLabel("Test:"), 1, 0)

        test_btn_layout = QtWidgets.QHBoxLayout()
        self.test_minus_10 = QtWidgets.QPushButton("-10°")
        self.test_minus_10.setStyleSheet("background-color: #ffcccc;")
        self.test_minus_1 = QtWidgets.QPushButton("-1°")
        self.test_minus_1.setStyleSheet("background-color: #ffcccc;")
        self.test_plus_1 = QtWidgets.QPushButton("+1°")
        self.test_plus_1.setStyleSheet("background-color: #ccffcc;")
        self.test_plus_10 = QtWidgets.QPushButton("+10°")
        self.test_plus_10.setStyleSheet("background-color: #ccffcc;")

        test_btn_layout.addWidget(self.test_minus_10)
        test_btn_layout.addWidget(self.test_minus_1)
        test_btn_layout.addWidget(self.test_plus_1)
        test_btn_layout.addWidget(self.test_plus_10)
        frame_layout.addLayout(test_btn_layout, 1, 1, 1, 2)

        main_layout.addWidget(frame)

        # Connect signals - use clicked (only fires on user interaction, not setChecked)
        self.forward_radio.clicked.connect(lambda: self._on_user_direction_click(1))
        self.reverse_radio.clicked.connect(lambda: self._on_user_direction_click(-1))

        self.test_minus_10.clicked.connect(lambda: self.test_movement.emit(self.joint_name, -10))
        self.test_minus_1.clicked.connect(lambda: self.test_movement.emit(self.joint_name, -1))
        self.test_plus_1.clicked.connect(lambda: self.test_movement.emit(self.joint_name, 1))
        self.test_plus_10.clicked.connect(lambda: self.test_movement.emit(self.joint_name, 10))

    def _on_user_direction_click(self, direction):
        """Called when user clicks a direction radio button"""
        self.current_direction = direction
        self.direction_changed.emit(self.joint_name, direction)

    def set_direction(self, direction):
        """Set direction programmatically"""
        self.current_direction = direction
        if direction == 1:
            self.forward_radio.setChecked(True)
        else:
            self.reverse_radio.setChecked(True)

    def get_direction(self):
        """Get current direction value"""
        return self.current_direction


class GripperCalibrationWidget(QtWidgets.QWidget):
    """Widget for calibrating gripper PWM range"""

    # Signal emitted when gripper test command should be sent
    test_gripper = QtCore.pyqtSignal(int)  # PWM value to test

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        self.load_calibration()

    def setup_ui(self):
        """Create UI elements for gripper calibration"""
        main_layout = QtWidgets.QVBoxLayout(self)

        # Header
        header_layout = QtWidgets.QHBoxLayout()
        header_label = QtWidgets.QLabel("<b>Gripper PWM Calibration</b>")
        header_label.setStyleSheet("font-size: 14px;")
        header_layout.addWidget(header_label)
        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # Description
        desc_label = QtWidgets.QLabel(
            "Adjust PWM range to prevent servo stalling. "
            "Reduce 'Closed PWM' if gripper locks up when gripping objects."
        )
        desc_label.setWordWrap(True)
        desc_label.setStyleSheet("color: gray; font-size: 10px; margin-bottom: 5px;")
        main_layout.addWidget(desc_label)

        # Frame for controls
        frame = QtWidgets.QFrame()
        frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        frame_layout = QtWidgets.QGridLayout(frame)

        # Row 0: Open PWM (100% position)
        frame_layout.addWidget(QtWidgets.QLabel("Open PWM (100%):"), 0, 0)
        self.open_pwm_spinbox = QtWidgets.QSpinBox()
        self.open_pwm_spinbox.setRange(0, 255)
        self.open_pwm_spinbox.setValue(255)
        self.open_pwm_spinbox.setToolTip("PWM value when gripper is fully open")
        frame_layout.addWidget(self.open_pwm_spinbox, 0, 1)

        self.test_open_btn = QtWidgets.QPushButton("Test Open")
        self.test_open_btn.setStyleSheet("background-color: #ccffcc;")
        self.test_open_btn.clicked.connect(lambda: self.test_gripper.emit(self.open_pwm_spinbox.value()))
        frame_layout.addWidget(self.test_open_btn, 0, 2)

        # Row 1: Closed PWM (0% position)
        frame_layout.addWidget(QtWidgets.QLabel("Closed PWM (0%):"), 1, 0)
        self.closed_pwm_spinbox = QtWidgets.QSpinBox()
        self.closed_pwm_spinbox.setRange(0, 255)
        self.closed_pwm_spinbox.setValue(0)
        self.closed_pwm_spinbox.setToolTip("PWM value when gripper is fully closed - reduce to prevent stalling")
        frame_layout.addWidget(self.closed_pwm_spinbox, 1, 1)

        self.test_closed_btn = QtWidgets.QPushButton("Test Closed")
        self.test_closed_btn.setStyleSheet("background-color: #ffcccc;")
        self.test_closed_btn.clicked.connect(lambda: self.test_gripper.emit(self.closed_pwm_spinbox.value()))
        frame_layout.addWidget(self.test_closed_btn, 1, 2)

        # Row 2: Direct PWM test slider
        frame_layout.addWidget(QtWidgets.QLabel("Test PWM:"), 2, 0)
        self.test_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.test_slider.setRange(0, 255)
        self.test_slider.setValue(128)
        frame_layout.addWidget(self.test_slider, 2, 1)

        self.test_value_label = QtWidgets.QLabel("128")
        self.test_value_label.setMinimumWidth(30)
        frame_layout.addWidget(self.test_value_label, 2, 2)

        # Connect slider to label update
        self.test_slider.valueChanged.connect(lambda v: self.test_value_label.setText(str(v)))

        # Row 3: Send test button
        self.send_test_btn = QtWidgets.QPushButton("Send Test PWM")
        self.send_test_btn.setStyleSheet("background-color: #ffffcc;")
        self.send_test_btn.clicked.connect(lambda: self.test_gripper.emit(self.test_slider.value()))
        frame_layout.addWidget(self.send_test_btn, 3, 1, 1, 2)

        # Row 4: Current effective range display
        frame_layout.addWidget(QtWidgets.QLabel("Effective Range:"), 4, 0)
        self.range_label = QtWidgets.QLabel("0 - 255 PWM")
        self.range_label.setStyleSheet("font-weight: bold; color: #0066cc;")
        frame_layout.addWidget(self.range_label, 4, 1, 1, 2)

        main_layout.addWidget(frame)

        # Update range label when values change
        self.open_pwm_spinbox.valueChanged.connect(self.update_range_label)
        self.closed_pwm_spinbox.valueChanged.connect(self.update_range_label)


    def update_range_label(self):
        """Update the effective range display"""
        closed = self.closed_pwm_spinbox.value()
        open_val = self.open_pwm_spinbox.value()
        self.range_label.setText(f"{closed} - {open_val} PWM")

    def load_calibration(self):
        """Load gripper calibration from file or config defaults"""
        try:
            if GRIPPER_CALIBRATION_FILE.exists():
                with open(GRIPPER_CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)
                self.open_pwm_spinbox.setValue(data.get('pwm_open', 255))
                self.closed_pwm_spinbox.setValue(data.get('pwm_closed', 0))
                logger.info("Loaded gripper calibration from file")
            else:
                # Use config defaults
                self.open_pwm_spinbox.setValue(config.GRIPPER_PWM_OPEN)
                self.closed_pwm_spinbox.setValue(config.GRIPPER_PWM_CLOSED)
                logger.info("Using default gripper calibration from config")

            self.update_range_label()
            self.apply_to_config()

        except Exception as e:
            logger.error(f"Error loading gripper calibration: {e}")

    def apply_to_config(self):
        """Apply current values to the config module at runtime"""
        config.GRIPPER_PWM_OPEN = self.open_pwm_spinbox.value()
        config.GRIPPER_PWM_CLOSED = self.closed_pwm_spinbox.value()
        logger.debug(f"Applied gripper PWM: open={config.GRIPPER_PWM_OPEN}, closed={config.GRIPPER_PWM_CLOSED}")


class DHParametersWidget(QtWidgets.QWidget):
    """Widget for editing DH parameters table"""

    parameters_changed = QtCore.pyqtSignal()  # Emitted when parameters are saved
    preview_changed = QtCore.pyqtSignal()  # Emitted when any value changes (for live preview)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.spinboxes = {}
        self._loading = False
        self.setup_ui()
        self.load_parameters()

    def setup_ui(self):
        """Create UI elements for DH parameters"""
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # Header
        header_label = QtWidgets.QLabel("<b>DH Parameters</b>")
        header_label.setStyleSheet("font-size: 14px;")
        main_layout.addWidget(header_label)

        # Create table
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Link", "θ offset (°)", "d (mm)", "a (mm)", "α (°)"])
        self.table.setRowCount(6)

        # Set column widths
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.table.setColumnWidth(0, 40)

        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(220)
        self.table.verticalHeader().setDefaultSectionSize(30)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                alternate-background-color: #f9f9f9;
                gridline-color: #ddd;
                border: 1px solid #ccc;
            }
            QHeaderView::section {
                background-color: #e8e8e8;
                padding: 4px;
                border: 1px solid #ccc;
                font-weight: bold;
            }
        """)

        # Create widgets for each cell
        for row in range(6):
            # Link number (read-only)
            link_item = QtWidgets.QTableWidgetItem(str(row + 1))
            link_item.setFlags(QtCore.Qt.ItemIsEnabled)
            link_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.table.setItem(row, 0, link_item)

            # theta_offset spinbox
            spinbox = QtWidgets.QDoubleSpinBox()
            spinbox.setRange(-360, 360)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setAlignment(QtCore.Qt.AlignCenter)
            self.table.setCellWidget(row, 1, spinbox)
            self.spinboxes[(row, 'theta_offset')] = spinbox

            # d, a, alpha spinboxes
            for col, param in enumerate(['d', 'a', 'alpha'], start=2):
                spinbox = QtWidgets.QDoubleSpinBox()
                spinbox.setRange(-1000 if param in ['d', 'a'] else -360, 1000 if param in ['d', 'a'] else 360)
                spinbox.setDecimals(2)
                spinbox.setSingleStep(1.0)
                spinbox.setAlignment(QtCore.Qt.AlignCenter)
                self.table.setCellWidget(row, col, spinbox)
                self.spinboxes[(row, param)] = spinbox

        main_layout.addWidget(self.table)

        # Connect all spinboxes to emit preview_changed
        for spinbox in self.spinboxes.values():
            spinbox.valueChanged.connect(self._on_value_changed)

        # Buttons
        button_layout = QtWidgets.QHBoxLayout()

        self.load_button = QtWidgets.QPushButton("Load")
        self.load_button.clicked.connect(self.load_parameters)
        button_layout.addWidget(self.load_button)

        self.reset_button = QtWidgets.QPushButton("Reset to Default")
        self.reset_button.clicked.connect(self.reset_to_default)
        button_layout.addWidget(self.reset_button)

        button_layout.addStretch()
        main_layout.addLayout(button_layout)

    def load_parameters(self):
        """Load DH parameters from file"""
        self._loading = True
        try:
            if DH_PARAMS_FILE.exists():
                with open(DH_PARAMS_FILE, 'r') as f:
                    dh_params = json.load(f)

                for link_data in dh_params['links']:
                    row = link_data['link'] - 1
                    self.spinboxes[(row, 'theta_offset')].setValue(link_data['theta_offset'])
                    self.spinboxes[(row, 'd')].setValue(link_data['d'])
                    self.spinboxes[(row, 'a')].setValue(link_data['a'])
                    self.spinboxes[(row, 'alpha')].setValue(link_data['alpha'])

                logger.info("Loaded DH parameters from file")
            else:
                self.reset_to_default()
        except Exception as e:
            logger.error(f"Error loading DH parameters: {e}")
        finally:
            self._loading = False

    def _on_value_changed(self):
        """Handle spinbox/combo value change - emit preview signal"""
        if not self._loading:
            self.preview_changed.emit()

    def get_parameters(self):
        """Get current DH parameters from the table as a list of dicts"""
        params = []
        descriptions = ["Base rotation", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Wrist yaw / TCP"]
        for row in range(6):
            params.append({
                "link": row + 1,
                "theta_offset": self.spinboxes[(row, 'theta_offset')].value(),
                "d": self.spinboxes[(row, 'd')].value(),
                "a": self.spinboxes[(row, 'a')].value(),
                "alpha": self.spinboxes[(row, 'alpha')].value(),
                "description": descriptions[row],
            })
        return params

    def save_parameters(self):
        """Save DH parameters to file"""
        try:
            descriptions = ["Base rotation", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Wrist yaw / TCP"]

            dh_data = {
                "version": "1.1",
                "description": "Thor Robot DH Parameters",
                "date_modified": QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd"),
                "links": []
            }

            for row in range(6):
                link_data = {
                    "link": row + 1,
                    "theta_offset": self.spinboxes[(row, 'theta_offset')].value(),
                    "d": self.spinboxes[(row, 'd')].value(),
                    "a": self.spinboxes[(row, 'a')].value(),
                    "alpha": self.spinboxes[(row, 'alpha')].value(),
                    "description": descriptions[row]
                }
                dh_data['links'].append(link_data)

            with open(DH_PARAMS_FILE, 'w') as f:
                json.dump(dh_data, f, indent=4)

            reload_dh_parameters()
            self.parameters_changed.emit()

            logger.info("Saved DH parameters to file")
            QtWidgets.QMessageBox.information(
                self,
                "Saved",
                "DH parameters saved.\nVisualization updated."
            )

        except Exception as e:
            logger.error(f"Error saving DH parameters: {e}")
            QtWidgets.QMessageBox.critical(self, "Save Error", f"Failed to save:\n{e}")

    def reset_to_default(self):
        """Reset to default Thor DH parameters"""
        self._loading = True
        try:
            default_params = [
                {"theta_offset": 0, "d": 202, "a": 0, "alpha": 90},
                {"theta_offset": 90, "d": 0, "a": 160, "alpha": 0},
                {"theta_offset": 90, "d": 0, "a": 0, "alpha": 90},
                {"theta_offset": 0, "d": 195, "a": 0, "alpha": -90},
                {"theta_offset": 0, "d": 0, "a": 0, "alpha": 90},
                {"theta_offset": 0, "d": 67.15, "a": 0, "alpha": 0},
            ]

            for row, params in enumerate(default_params):
                self.spinboxes[(row, 'theta_offset')].setValue(params['theta_offset'])
                self.spinboxes[(row, 'd')].setValue(params['d'])
                self.spinboxes[(row, 'a')].setValue(params['a'])
                self.spinboxes[(row, 'alpha')].setValue(params['alpha'])

            logger.info("Reset DH parameters to defaults")
        finally:
            self._loading = False


class CalibrationPanel(QtWidgets.QWidget):
    """Main calibration panel with direction verification, gripper calibration, and DH parameters"""

    def __init__(self, gui_instance, parent=None):
        super().__init__(parent)
        self.gui_instance = gui_instance
        self.joint_widgets = {}

        self.setup_ui()
        self.load_current_calibration()

        logger.info("Calibration panel initialised")

    def setup_ui(self):
        """Create the calibration panel UI"""
        main_layout = QtWidgets.QVBoxLayout(self)

        # Header
        header = QtWidgets.QLabel("<h2>Robot Calibration</h2>")
        main_layout.addWidget(header)

        # Instructions
        instructions = QtWidgets.QLabel(
            "<b>Verify motor directions match visualization:</b><br>"
            "1. Click +10° for each joint<br>"
            "2. If physical robot moves opposite to visualization, toggle 'Reverse'<br>"
            "3. Save when all joints are correct"
        )
        instructions.setWordWrap(True)
        instructions.setStyleSheet("background-color: #ffffcc; padding: 8px; border: 1px solid #cccc00;")
        main_layout.addWidget(instructions)

        # Scroll area for joint widgets
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        scroll_content = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        # Create joint calibration widgets
        joint_info = [
            ('Art1', 'Base rotation (X axis)'),
            ('Art2', 'Shoulder pitch (Y axis)'),
            ('Art3', 'Elbow pitch (Z axis)'),
            ('Art4', 'Wrist roll (U axis)'),
            ('Art5', 'Wrist pitch (V+W differential)'),
            ('Art6', 'Wrist yaw (V-W differential)')
        ]

        for joint_name, description in joint_info:
            widget = JointCalibrationWidget(joint_name, description)
            widget.test_movement.connect(self.on_test_movement)
            widget.direction_changed.connect(self.on_joint_direction_changed)
            self.joint_widgets[joint_name] = widget
            scroll_layout.addWidget(widget)

        # Add separator
        separator = QtWidgets.QFrame()
        separator.setFrameShape(QtWidgets.QFrame.HLine)
        separator.setStyleSheet("background-color: #ccc; margin: 10px 0;")
        scroll_layout.addWidget(separator)

        # Add gripper calibration widget
        self.gripper_calibration = GripperCalibrationWidget()
        self.gripper_calibration.test_gripper.connect(self.on_test_gripper)
        self.gripper_calibration.open_pwm_spinbox.valueChanged.connect(self._auto_save_gripper)
        self.gripper_calibration.closed_pwm_spinbox.valueChanged.connect(self._auto_save_gripper)
        scroll_layout.addWidget(self.gripper_calibration)

        # Add separator before DH parameters
        separator2 = QtWidgets.QFrame()
        separator2.setFrameShape(QtWidgets.QFrame.HLine)
        separator2.setStyleSheet("background-color: #ccc; margin: 10px 0;")
        scroll_layout.addWidget(separator2)

        # Add DH parameters widget
        self.dh_parameters = DHParametersWidget()
        scroll_layout.addWidget(self.dh_parameters)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # Status bar
        self.status_label = QtWidgets.QLabel("Status: Ready to calibrate")
        self.status_label.setStyleSheet("background-color: #e0e0e0; padding: 5px;")
        main_layout.addWidget(self.status_label)

    def on_test_movement(self, joint_name, delta_angle):
        """Handle test movement button clicks"""
        logger.info(f"Test movement: {joint_name} {delta_angle:+.1f}°")

        # Get the corresponding spinbox from main GUI
        joint_spinbox_map = {
            'Art1': 'SpinBoxArt1',
            'Art2': 'SpinBoxArt2',
            'Art3': 'SpinBoxArt3',
            'Art4': 'SpinBoxArt4',
            'Art5': 'SpinBoxArt5',
            'Art6': 'SpinBoxArt6'
        }

        spinbox_name = joint_spinbox_map.get(joint_name)
        if spinbox_name and hasattr(self.gui_instance, spinbox_name):
            spinbox = getattr(self.gui_instance, spinbox_name)
            new_value = spinbox.value() + delta_angle
            spinbox.setValue(new_value)

            # Execute movement
            self.gui_instance.FKMoveJoint(joint_name)

            self.status_label.setText(f"Status: Moved {joint_name} {delta_angle:+.1f}° → {new_value:.1f}°")
            self.status_label.setStyleSheet("background-color: #ccffcc; padding: 5px;")

    def on_joint_direction_changed(self, joint_name, direction):
        """Send M569 command to firmware and update config.g for persistence."""
        drives = JOINT_TO_DRIVES.get(joint_name)
        if drives is None:
            return

        try:
            s_value = 0 if direction == 1 else 1
            dir_label = 'Forward' if direction == 1 else 'Reverse'

            # Send M569 commands to firmware for each drive
            commands_sent = 0
            if self.gui_instance and hasattr(self.gui_instance, 'command_sender'):
                for drive in drives:
                    command = f"M569 P{drive} S{s_value}"
                    if self.gui_instance.command_sender.send_if_connected(command):
                        commands_sent += 1
                    else:
                        logger.warning(f"Not connected - M569 P{drive} not sent")

            # Always persist to config.g (even when not connected)
            config_g_path = Path(__file__).parent / 'sys' / 'config.g'
            set_joint_direction(config_g_path, joint_name, direction)

            if commands_sent > 0:
                logger.info(f"{joint_name} direction set to {dir_label} (sent {commands_sent} M569 command(s))")
                self.status_label.setText(f"Status: {joint_name} = {dir_label} (M569 sent)")
            else:
                logger.info(f"{joint_name} direction set to {dir_label} (saved to config.g, not connected)")
                self.status_label.setText(f"Status: {joint_name} = {dir_label} (saved, not connected)")
            self.status_label.setStyleSheet("background-color: #ccffcc; padding: 5px;")

        except Exception as e:
            logger.error(f"Error updating {joint_name} direction: {e}")
            self.status_label.setText(f"Status: Error - {e}")
            self.status_label.setStyleSheet("background-color: #ffcccc; padding: 5px;")

    def on_test_gripper(self, pwm_value):
        """Handle gripper test button clicks - send direct PWM command"""
        logger.info(f"Test gripper PWM: {pwm_value}")

        if not self.gui_instance:
            self.status_label.setText("Status: No GUI instance - cannot send command")
            self.status_label.setStyleSheet("background-color: #ffcccc; padding: 5px;")
            return

        # Convert PWM to servo angle (0-255 PWM -> 0-180 servo angle)
        servo_angle = int((pwm_value / 255.0) * 180.0)
        command = f"M280 P0 S{servo_angle}"

        # Send via serial if connected
        if hasattr(self.gui_instance, 'serial_manager') and self.gui_instance.serial_manager:
            if self.gui_instance.serial_manager.is_connected():
                self.gui_instance.serial_manager.send_command(command)
                self.status_label.setText(f"Status: Sent gripper test PWM={pwm_value} (angle={servo_angle}°)")
                self.status_label.setStyleSheet("background-color: #ccffcc; padding: 5px;")
            else:
                self.status_label.setText("Status: Not connected - cannot send gripper command")
                self.status_label.setStyleSheet("background-color: #ffcccc; padding: 5px;")
        else:
            self.status_label.setText("Status: Serial manager not available")
            self.status_label.setStyleSheet("background-color: #ffcccc; padding: 5px;")

    def load_current_calibration(self):
        """Load direction settings from config.g M569 commands"""
        try:
            config_g_path = Path(__file__).parent / 'sys' / 'config.g'
            directions = get_joint_directions(config_g_path)

            for joint_name, direction in directions.items():
                if joint_name in self.joint_widgets:
                    self.joint_widgets[joint_name].set_direction(direction)

            self.status_label.setText("Status: Loaded motor directions from config.g")
            self.status_label.setStyleSheet("background-color: #ccffcc; padding: 5px;")
            logger.info("Loaded direction settings from config.g")

        except Exception as e:
            logger.error(f"Error loading calibration: {e}")
            logger.exception("Full traceback:")
            self.status_label.setText(f"Status: Error loading - {e}")
            self.status_label.setStyleSheet("background-color: #ffcccc; padding: 5px;")

    def _auto_save_gripper(self):
        """Auto-save gripper calibration when spinbox values change."""
        try:
            gripper_data = {
                'pwm_open': self.gripper_calibration.open_pwm_spinbox.value(),
                'pwm_closed': self.gripper_calibration.closed_pwm_spinbox.value()
            }
            with open(GRIPPER_CALIBRATION_FILE, 'w') as f:
                json.dump(gripper_data, f, indent=4)
            self.gripper_calibration.apply_to_config()
            logger.debug(f"Auto-saved gripper calibration: {gripper_data}")
        except Exception as e:
            logger.error(f"Error auto-saving gripper calibration: {e}")

