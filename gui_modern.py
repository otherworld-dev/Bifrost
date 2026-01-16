# -*- coding: utf-8 -*-
"""
Bifrost Modern UI - Mode-Based Interface
Industry-standard robot control interface with mode switching

Modes:
- JOG: Manual joint control
- INVERSE: Inverse kinematics (Cartesian control)
- TEACH: Sequence programming
- TERMINAL: Console and debugging
- 3D VIEW: Full-screen visualization
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
                            QGridLayout, QLabel, QPushButton, QRadioButton,
                            QDoubleSpinBox, QCheckBox, QComboBox, QSpinBox,
                            QFrame, QButtonGroup, QStackedWidget, QTableWidget,
                            QTableWidgetItem, QHeaderView, QPlainTextEdit,
                            QLineEdit, QListWidget, QGroupBox, QSizePolicy)
from robot_3d_visualizer import Robot3DCanvas


class TableItemLabelWrapper:
    """
    Wrapper class to make QTableWidgetItem behave like QLabel for compatibility
    with existing bifrost.py code that calls .setText() on labels
    """
    def __init__(self, table_item):
        self.table_item = table_item

    def setText(self, text):
        """Delegate setText to QTableWidgetItem"""
        self.table_item.setText(text)

    def text(self):
        """Delegate text() to QTableWidgetItem"""
        return self.table_item.text()

    def setStyleSheet(self, stylesheet):
        """Handle stylesheet by setting background colour for table items"""
        # Extract background-color from stylesheet
        if "background-color" in stylesheet:
            # Simple parsing for common patterns
            if "rgb(200, 255, 200)" in stylesheet:
                self.table_item.setBackground(QtGui.QColor(200, 255, 200))
            elif "rgb(255, 200, 200)" in stylesheet:
                self.table_item.setBackground(QtGui.QColor(255, 200, 200))
            elif "rgb(255, 255, 200)" in stylesheet:
                self.table_item.setBackground(QtGui.QColor(255, 255, 200))
            elif "rgb(200, 200, 200)" in stylesheet:
                self.table_item.setBackground(QtGui.QColor(200, 200, 200))
        # Ignore other stylesheet properties for table items


class ModernMainWindow(QMainWindow):
    """Modern mode-based GUI for Bifrost robot control"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bifrost - Thor Robot Control")
        self.setMinimumSize(1200, 750)

        # Create central widget
        self.centralwidget = QWidget()
        self.setCentralWidget(self.centralwidget)

        # Main layout
        main_layout = QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        # Top bar (connection status)
        self.top_bar = ConnectionBar()
        main_layout.addWidget(self.top_bar)

        # Mode selector bar
        self.mode_selector = ModeSelectorBar()
        main_layout.addWidget(self.mode_selector)

        # Main content area (split left/right)
        content_splitter = QtWidgets.QSplitter(Qt.Horizontal)

        # Left panel: Mode-specific controls (40%)
        self.mode_stack = QStackedWidget()
        content_splitter.addWidget(self.mode_stack)

        # Right panel: Unified robot state (60%)
        self.robot_state_panel = RobotStatePanel()
        content_splitter.addWidget(self.robot_state_panel)

        # Set splitter proportions and initial sizes
        content_splitter.setStretchFactor(0, 40)  # Left: 40%
        content_splitter.setStretchFactor(1, 60)  # Right: 60%

        # Force initial sizes (40% / 60% of 1200px window = 480px / 720px)
        content_splitter.setSizes([480, 720])

        # Set minimum width for left panel so it can't be collapsed
        self.mode_stack.setMinimumWidth(400)

        # Prevent splitter panels from collapsing
        content_splitter.setCollapsible(0, False)  # Left panel
        content_splitter.setCollapsible(1, False)  # Right panel

        main_layout.addWidget(content_splitter)

        # Create mode panels
        self.setup_mode_panels()

        # Connect mode switching
        self.mode_selector.mode_changed.connect(self.switch_mode)

        # Default to JOG mode
        self.switch_mode(0)

    def setup_mode_panels(self):
        """Create all mode-specific panels (JOG removed - controls in sidebar)"""
        # Mode 0: INVERSE
        self.inverse_panel = InverseModePanel()
        self.mode_stack.addWidget(self.inverse_panel)

        # Mode 1: TEACH
        self.teach_panel = TeachModePanel()
        self.mode_stack.addWidget(self.teach_panel)

        # Mode 2: TERMINAL
        self.terminal_panel = TerminalModePanel()
        self.mode_stack.addWidget(self.terminal_panel)

        # Mode 3: CALIBRATE (includes DH parameters)
        from calibration_panel import CalibrationPanel
        # gui_instance will be set later by BifrostGUI
        self.calibration_panel = CalibrationPanel(gui_instance=None)
        self.mode_stack.addWidget(self.calibration_panel)

        # Mode 4: FRAMES
        from frame_panel import FrameManagementPanel
        self.frames_panel = FrameManagementPanel()
        self.mode_stack.addWidget(self.frames_panel)

    def switch_mode(self, mode_index):
        """Switch to specified mode"""
        self.mode_stack.setCurrentIndex(mode_index)


class ConnectionBar(QFrame):
    """Top bar showing connection status and settings"""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumHeight(50)
        self.setMaximumHeight(50)
        self.setStyleSheet("ConnectionBar { background-color: #e8e8e8; border-bottom: 2px solid #ccc; }")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(10)

        # App title
        title = QLabel("Bifrost")
        title_font = QtGui.QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        layout.addWidget(QLabel("|"))

        # Serial port
        self.SerialPortLabel = QLabel("COM Port:")
        layout.addWidget(self.SerialPortLabel)

        self.SerialPortComboBox = QComboBox()
        self.SerialPortComboBox.setMinimumWidth(80)
        layout.addWidget(self.SerialPortComboBox)

        self.SerialPortRefreshButton = QPushButton("⟳")
        self.SerialPortRefreshButton.setMaximumWidth(30)
        self.SerialPortRefreshButton.setToolTip("Refresh serial ports")
        layout.addWidget(self.SerialPortRefreshButton)

        # Baud rate
        layout.addWidget(QLabel("@"))
        self.BaudRateComboBox = QComboBox()
        self.BaudRateComboBox.setMinimumWidth(80)
        self.BaudRateComboBox.addItems([
            "9600", "14400", "19200", "28800", "38400",
            "57600", "115200", "230400", "250000", "500000", "1000000", "2000000"
        ])
        self.BaudRateComboBox.setCurrentText("115200")
        layout.addWidget(self.BaudRateComboBox)

        layout.addWidget(QLabel("|"))

        # Connection status
        layout.addWidget(QLabel("Status:"))
        self.RobotStateDisplay = QLabel("Disconnected")
        self.RobotStateDisplay.setFrameShape(QFrame.Box)
        self.RobotStateDisplay.setMinimumWidth(100)
        self.RobotStateDisplay.setAlignment(Qt.AlignCenter)
        self.RobotStateDisplay.setStyleSheet("background-color: rgb(255, 0, 0); font-weight: bold; padding: 3px;")
        layout.addWidget(self.RobotStateDisplay)

        layout.addStretch()

        # Connect/Disconnect button
        self.ConnectButton = QPushButton("Connect")
        self.ConnectButton.setMinimumWidth(100)
        self.ConnectButton.setMinimumHeight(35)
        connect_font = QtGui.QFont()
        connect_font.setBold(True)
        self.ConnectButton.setFont(connect_font)
        layout.addWidget(self.ConnectButton)

        layout.addWidget(QLabel("|"))

        # Emergency Stop button - freezes motors and stops sequences
        self.EmergencyStopButton = QPushButton("🛑 E-STOP")
        self.EmergencyStopButton.setMinimumWidth(100)
        self.EmergencyStopButton.setMinimumHeight(35)
        estop_font = QtGui.QFont()
        estop_font.setBold(True)
        estop_font.setPointSize(10)
        self.EmergencyStopButton.setFont(estop_font)
        self.EmergencyStopButton.setStyleSheet("""
            QPushButton {
                background-color: #D32F2F;
                color: white;
                border: 3px solid #B71C1C;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #C62828;
            }
            QPushButton:pressed {
                background-color: #B71C1C;
            }
        """)
        self.EmergencyStopButton.setToolTip("Emergency Stop - Freeze motors and abort sequences")
        layout.addWidget(self.EmergencyStopButton)

        # About button
        self.SettingsButton = QPushButton("ℹ")
        self.SettingsButton.setMaximumWidth(40)
        self.SettingsButton.setMinimumHeight(35)
        self.SettingsButton.setToolTip("About")
        layout.addWidget(self.SettingsButton)


class ModeSelectorBar(QFrame):
    """Mode selection buttons"""

    mode_changed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumHeight(50)
        self.setMaximumHeight(50)
        self.setStyleSheet("ModeSelectorBar { background-color: #d8d8d8; border-bottom: 2px solid #bbb; }")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(10)

        # Mode buttons (JOG removed - axis controls now in sidebar)
        self.mode_group = QButtonGroup()

        self.btn_inverse = QPushButton("🎯 INVERSE")
        self.btn_inverse.setCheckable(True)
        self.btn_inverse.setMinimumHeight(35)
        self.btn_inverse.setMinimumWidth(130)
        self.mode_group.addButton(self.btn_inverse, 0)
        layout.addWidget(self.btn_inverse)

        self.btn_teach = QPushButton("💾 TEACH")
        self.btn_teach.setCheckable(True)
        self.btn_teach.setMinimumHeight(35)
        self.btn_teach.setMinimumWidth(120)
        self.mode_group.addButton(self.btn_teach, 1)
        layout.addWidget(self.btn_teach)

        # Stretch to push remaining buttons to the right
        layout.addStretch()

        self.btn_terminal = QPushButton("🖥 TERMINAL")
        self.btn_terminal.setCheckable(True)
        self.btn_terminal.setMinimumHeight(35)
        self.btn_terminal.setMinimumWidth(140)
        self.mode_group.addButton(self.btn_terminal, 2)
        layout.addWidget(self.btn_terminal)

        self.btn_calibrate = QPushButton("🔧 CALIBRATE")
        self.btn_calibrate.setCheckable(True)
        self.btn_calibrate.setMinimumHeight(35)
        self.btn_calibrate.setMinimumWidth(150)
        self.mode_group.addButton(self.btn_calibrate, 3)
        layout.addWidget(self.btn_calibrate)

        self.btn_frames = QPushButton("📐 FRAMES")
        self.btn_frames.setCheckable(True)
        self.btn_frames.setMinimumHeight(35)
        self.btn_frames.setMinimumWidth(120)
        self.mode_group.addButton(self.btn_frames, 4)
        layout.addWidget(self.btn_frames)

        # Set default to INVERSE (was JOG, now removed)
        self.btn_inverse.setChecked(True)

        # Connect signals
        self.mode_group.buttonClicked.connect(self.on_mode_clicked)

        # Style selected button
        self.update_button_styles()

    def on_mode_clicked(self, button):
        """Handle mode button click"""
        mode_id = self.mode_group.id(button)
        self.mode_changed.emit(mode_id)
        self.update_button_styles()

    def update_button_styles(self):
        """Update visual style of mode buttons"""
        for button in self.mode_group.buttons():
            if button.isChecked():
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #4CAF50;
                        color: white;
                        font-weight: bold;
                        border: 2px solid #45a049;
                        border-radius: 3px;
                    }
                """)
            else:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #f0f0f0;
                        color: #333;
                        border: 1px solid #ccc;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #e0e0e0;
                    }
                """)


class AxisRow(QFrame):
    """Single axis control row with +/- buttons and value display"""

    def __init__(self, joint_name, axis_label, is_gripper=False, is_cartesian=False):
        super().__init__()
        self.joint_name = joint_name
        self.is_gripper = is_gripper
        self.is_cartesian = is_cartesian
        self.axis_label = axis_label

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(1)

        # Top row: Joint/axis label + indicators
        top_row = QHBoxLayout()
        top_row.setSpacing(2)

        # Label formatting depends on mode
        if is_gripper:
            label_text = "Grip"
        elif is_cartesian:
            # Cartesian mode: "X (mm)" or "Roll (°)"
            label_text = f"{joint_name} ({axis_label})"
        else:
            # Joint mode: "J1 (X)"
            label_text = f"{joint_name} ({axis_label})"
        self.label = QLabel(label_text)
        self.label.setStyleSheet("font-size: 8pt; font-weight: bold;")
        top_row.addWidget(self.label)

        top_row.addStretch()

        # Endstop indicator (small colored dot)
        self.endstop_indicator = QLabel("●")
        self.endstop_indicator.setStyleSheet("font-size: 7pt; color: #4CAF50;")  # Green = OK
        self.endstop_indicator.setToolTip("Endstop status")
        top_row.addWidget(self.endstop_indicator)

        # Position match indicator
        self.position_indicator = QLabel("○")
        self.position_indicator.setStyleSheet("font-size: 7pt; color: #888;")
        self.position_indicator.setToolTip("Position match")
        top_row.addWidget(self.position_indicator)

        layout.addLayout(top_row)

        # Bottom row: [-] value [+]
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(2)

        # Minus button
        self.minus_btn = QPushButton("-")
        self.minus_btn.setFixedSize(24, 24)
        self.minus_btn.setStyleSheet("font-size: 10pt; font-weight: bold;")
        self.minus_btn.setProperty("joint", joint_name)
        self.minus_btn.setProperty("direction", -1)
        bottom_row.addWidget(self.minus_btn)

        # Value label (read-only)
        self.value_label = QLabel("0.0" if not is_gripper else "0%")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 8pt; background: #f0f0f0; border-radius: 2px; padding: 2px;")
        self.value_label.setMinimumWidth(40)
        bottom_row.addWidget(self.value_label, 1)

        # Plus button
        self.plus_btn = QPushButton("+")
        self.plus_btn.setFixedSize(24, 24)
        self.plus_btn.setStyleSheet("font-size: 10pt; font-weight: bold;")
        self.plus_btn.setProperty("joint", joint_name)
        self.plus_btn.setProperty("direction", 1)
        bottom_row.addWidget(self.plus_btn)

        layout.addLayout(bottom_row)

        # Separator line at bottom
        self.setStyleSheet("AxisRow { border-bottom: 1px solid #ddd; }")

    def set_value(self, value):
        """Update the displayed value"""
        if self.is_gripper:
            self.value_label.setText(f"{int(value)}%")
        elif self.is_cartesian:
            # Cartesian mode: mm for position, ° for orientation
            if self.joint_name in ['X', 'Y', 'Z']:
                self.value_label.setText(f"{value:.1f}")
            else:
                # Roll, Pitch, Yaw - degrees
                self.value_label.setText(f"{value:.1f}")
        else:
            # Joint mode - degrees
            self.value_label.setText(f"{value:.1f}")

    def set_endstop_status(self, triggered):
        """Update endstop indicator colour"""
        if triggered:
            self.endstop_indicator.setStyleSheet("font-size: 7pt; color: #f44336;")  # Red
        else:
            self.endstop_indicator.setStyleSheet("font-size: 7pt; color: #4CAF50;")  # Green

    def set_position_match(self, matched):
        """Update position match indicator"""
        if matched:
            self.position_indicator.setStyleSheet("font-size: 7pt; color: #4CAF50;")  # Green
        else:
            self.position_indicator.setStyleSheet("font-size: 7pt; color: #888;")  # Grey


class AxisControlColumn(QFrame):
    """Narrow vertical column with axis controls and step toggle"""

    # Signal emitted when step value changes
    step_changed = pyqtSignal(float)
    # Signal emitted when control mode changes (joint vs cartesian)
    mode_changed = pyqtSignal(str)  # "joint", "cartesian"
    # Signal emitted when coordinate frame changes
    frame_changed = pyqtSignal(str)  # frame name: "base", "tool", "world"
    # Signal emitted when movement type changes (g0=rapid, g1=feed)
    movement_type_changed = pyqtSignal(str)  # "G0", "G1"
    # Signal emitted when feedrate changes
    feedrate_changed = pyqtSignal(float)

    # Control mode definitions
    JOINT_MODE = "joint"
    CARTESIAN_MODE = "cartesian"

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setFixedWidth(105)

        self.current_step = 1.0  # Default step size
        self.current_mode = self.JOINT_MODE  # Default to joint control
        self.current_frame = "base"  # Default coordinate frame

        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(0)

        # Mode selector (Joint / XYZ toggle)
        mode_frame = QFrame()
        mode_frame.setStyleSheet("background: #d0d0d0; border-radius: 3px;")
        mode_layout = QHBoxLayout(mode_frame)
        mode_layout.setContentsMargins(2, 2, 2, 2)
        mode_layout.setSpacing(2)

        self.joint_mode_btn = QPushButton("Joint")
        self.joint_mode_btn.setCheckable(True)
        self.joint_mode_btn.setChecked(True)
        self.joint_mode_btn.setFixedHeight(22)
        self.joint_mode_btn.setStyleSheet("""
            QPushButton {
                font-size: 8pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:checked {
                background: #2196F3;
                color: white;
                font-weight: bold;
            }
        """)
        self.joint_mode_btn.clicked.connect(lambda: self._set_mode(self.JOINT_MODE))
        mode_layout.addWidget(self.joint_mode_btn)

        self.cartesian_mode_btn = QPushButton("XYZ")
        self.cartesian_mode_btn.setCheckable(True)
        self.cartesian_mode_btn.setFixedHeight(22)
        self.cartesian_mode_btn.setStyleSheet("""
            QPushButton {
                font-size: 8pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:checked {
                background: #2196F3;
                color: white;
                font-weight: bold;
            }
        """)
        self.cartesian_mode_btn.clicked.connect(lambda: self._set_mode(self.CARTESIAN_MODE))
        mode_layout.addWidget(self.cartesian_mode_btn)

        layout.addWidget(mode_frame)

        # Coordinate frame selector (only visible in Cartesian mode)
        self.frame_selector_container = QFrame()
        self.frame_selector_container.setStyleSheet("background: #e0e0e0; border-radius: 2px;")
        frame_sel_layout = QHBoxLayout(self.frame_selector_container)
        frame_sel_layout.setContentsMargins(2, 2, 2, 2)
        frame_sel_layout.setSpacing(2)

        frame_label = QLabel("Frame:")
        frame_label.setStyleSheet("font-size: 7pt;")
        frame_sel_layout.addWidget(frame_label)

        self.frame_selector = QComboBox()
        self.frame_selector.setFixedHeight(20)
        self.frame_selector.setStyleSheet("font-size: 7pt;")
        self.frame_selector.addItems(["Base", "Tool", "World"])
        self.frame_selector.currentTextChanged.connect(self._on_frame_changed)
        frame_sel_layout.addWidget(self.frame_selector, 1)

        layout.addWidget(self.frame_selector_container)
        self.frame_selector_container.setVisible(False)  # Hidden in joint mode

        # Small spacer
        layout.addSpacing(2)

        # Axis rows container (will be repopulated on mode change)
        self.rows_container = QVBoxLayout()
        self.rows_container.setSpacing(0)
        layout.addLayout(self.rows_container)

        # Create rows for both modes
        self.rows = {}
        self._create_joint_rows()

        # Gripper row (always visible)
        gripper_row = AxisRow("Gripper", "", is_gripper=True)
        self.rows["Gripper"] = gripper_row
        layout.addWidget(gripper_row)

        # Movement type section
        move_frame = QFrame()
        move_frame.setStyleSheet("background: #e0e8f0; border-radius: 3px;")
        move_layout = QVBoxLayout(move_frame)
        move_layout.setContentsMargins(3, 3, 3, 3)
        move_layout.setSpacing(2)

        # Movement Type heading
        move_heading = QLabel("Move Type")
        move_heading.setStyleSheet("font-size: 7pt; font-weight: bold; color: #555;")
        move_heading.setAlignment(Qt.AlignCenter)
        move_layout.addWidget(move_heading)

        # G0/G1 toggle row
        move_btn_row = QHBoxLayout()
        move_btn_row.setSpacing(2)

        self.g0_btn = QPushButton("G0")
        self.g0_btn.setCheckable(True)
        self.g0_btn.setChecked(True)
        self.g0_btn.setFixedHeight(20)
        self.g0_btn.setToolTip("Rapid movement")
        self.g0_btn.setStyleSheet("""
            QPushButton {
                font-size: 7pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:checked {
                background: #FF9800;
                color: white;
                font-weight: bold;
            }
        """)
        self.g0_btn.clicked.connect(lambda: self._set_movement_type("G0"))
        move_btn_row.addWidget(self.g0_btn)

        self.g1_btn = QPushButton("G1")
        self.g1_btn.setCheckable(True)
        self.g1_btn.setFixedHeight(20)
        self.g1_btn.setToolTip("Feed movement")
        self.g1_btn.setStyleSheet("""
            QPushButton {
                font-size: 7pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:checked {
                background: #FF9800;
                color: white;
                font-weight: bold;
            }
        """)
        self.g1_btn.clicked.connect(lambda: self._set_movement_type("G1"))
        move_btn_row.addWidget(self.g1_btn)

        move_layout.addLayout(move_btn_row)

        # Feedrate row (only enabled when G1)
        feed_row = QHBoxLayout()
        feed_row.setSpacing(2)

        feed_label = QLabel("F:")
        feed_label.setStyleSheet("font-size: 7pt;")
        feed_label.setFixedWidth(12)
        feed_row.addWidget(feed_label)

        self.feedrate_spin = QSpinBox()
        self.feedrate_spin.setRange(1, 10000)
        self.feedrate_spin.setValue(1000)
        self.feedrate_spin.setSuffix("")
        self.feedrate_spin.setFixedHeight(20)
        self.feedrate_spin.setStyleSheet("font-size: 7pt;")
        self.feedrate_spin.setEnabled(False)  # Disabled by default (G0 mode)
        self.feedrate_spin.valueChanged.connect(lambda v: self.feedrate_changed.emit(float(v)))
        feed_row.addWidget(self.feedrate_spin, 1)

        move_layout.addLayout(feed_row)
        layout.addWidget(move_frame)

        self.current_movement_type = "G0"

        layout.addStretch()

        # Step toggle section
        step_frame = QFrame()
        step_frame.setStyleSheet("background: #e8e8e8; border-radius: 3px;")
        step_layout = QVBoxLayout(step_frame)
        step_layout.setContentsMargins(3, 3, 3, 3)
        step_layout.setSpacing(2)

        # Step buttons row
        btn_row = QHBoxLayout()
        btn_row.setSpacing(2)

        self.step_buttons = {}
        for step_val in [0.1, 1.0, 10.0]:
            btn = QPushButton(f"{step_val:g}")
            btn.setCheckable(True)
            btn.setFixedHeight(22)
            btn.setStyleSheet("""
                QPushButton {
                    font-size: 8pt;
                    border: 1px solid #999;
                    border-radius: 2px;
                    background: #fff;
                }
                QPushButton:checked {
                    background: #4CAF50;
                    color: white;
                    font-weight: bold;
                }
            """)
            btn.clicked.connect(lambda checked, s=step_val: self._on_step_clicked(s))
            self.step_buttons[step_val] = btn
            btn_row.addWidget(btn)

        # Set default step
        self.step_buttons[1.0].setChecked(True)

        step_layout.addLayout(btn_row)
        layout.addWidget(step_frame)

        # Quick commands section
        quick_frame = QFrame()
        quick_frame.setStyleSheet("background: #f0e8e8; border-radius: 3px;")
        quick_layout = QVBoxLayout(quick_frame)
        quick_layout.setContentsMargins(3, 3, 3, 3)
        quick_layout.setSpacing(2)

        # Quick Commands heading
        quick_heading = QLabel("Quick")
        quick_heading.setStyleSheet("font-size: 7pt; font-weight: bold; color: #555;")
        quick_heading.setAlignment(Qt.AlignCenter)
        quick_layout.addWidget(quick_heading)

        # Home button
        self.HomeButton = QPushButton("🏠 Home")
        self.HomeButton.setFixedHeight(24)
        self.HomeButton.setStyleSheet("""
            QPushButton {
                font-size: 7pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:hover {
                background: #e8f4e8;
            }
        """)
        self.HomeButton.setToolTip("Home all axes (G28)")
        quick_layout.addWidget(self.HomeButton)

        # Zero button
        self.ZeroPositionButton = QPushButton("Zero")
        self.ZeroPositionButton.setFixedHeight(24)
        self.ZeroPositionButton.setStyleSheet("""
            QPushButton {
                font-size: 7pt;
                border: 1px solid #888;
                border-radius: 2px;
                background: #fff;
            }
            QPushButton:hover {
                background: #e8e8f4;
            }
        """)
        self.ZeroPositionButton.setToolTip("Zero all positions (G92)")
        quick_layout.addWidget(self.ZeroPositionButton)

        # Jog mode checkbox
        self.jog_mode_checkbox = QCheckBox("Jog Mode")
        self.jog_mode_checkbox.setStyleSheet("font-size: 7pt;")
        self.jog_mode_checkbox.setToolTip("Enable immediate movement on +/- buttons")
        self.jog_mode_checkbox.setChecked(True)  # Default enabled
        quick_layout.addWidget(self.jog_mode_checkbox)

        layout.addWidget(quick_frame)

    def _create_joint_rows(self):
        """Create axis rows for joint control mode (J1-J6)"""
        self._clear_rows()
        joint_data = [
            ("J1", "X"), ("J2", "Y"), ("J3", "Z"),
            ("J4", "U"), ("J5", "V"), ("J6", "W")
        ]
        for joint_name, axis_label in joint_data:
            row = AxisRow(joint_name, axis_label, is_gripper=False)
            self.rows[joint_name] = row
            self.rows_container.addWidget(row)

    def _create_cartesian_rows(self):
        """Create axis rows for Cartesian control mode (X, Y, Z, Roll, Pitch, Yaw)"""
        self._clear_rows()
        cartesian_data = [
            ("X", "mm"), ("Y", "mm"), ("Z", "mm"),
            ("Roll", "°"), ("Pitch", "°"), ("Yaw", "°")
        ]
        for axis_name, unit in cartesian_data:
            row = AxisRow(axis_name, unit, is_gripper=False, is_cartesian=True)
            self.rows[axis_name] = row
            self.rows_container.addWidget(row)

    def _clear_rows(self):
        """Remove all axis rows (except gripper)"""
        # Keep gripper row reference
        gripper = self.rows.get("Gripper")
        # Clear container
        while self.rows_container.count():
            item = self.rows_container.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        # Reset rows dict but preserve gripper
        self.rows = {}
        if gripper:
            self.rows["Gripper"] = gripper

    def _set_mode(self, mode):
        """Switch between joint and cartesian control modes"""
        if mode == self.current_mode:
            return

        self.current_mode = mode

        # Update button states
        self.joint_mode_btn.setChecked(mode == self.JOINT_MODE)
        self.cartesian_mode_btn.setChecked(mode == self.CARTESIAN_MODE)

        # Show/hide frame selector based on mode
        self.frame_selector_container.setVisible(mode == self.CARTESIAN_MODE)

        # Recreate rows for new mode
        if mode == self.JOINT_MODE:
            self._create_joint_rows()
        else:
            self._create_cartesian_rows()

        # Emit signal for external handling
        self.mode_changed.emit(mode)

    def get_mode(self):
        """Get current control mode"""
        return self.current_mode

    def _on_frame_changed(self, frame_text):
        """Handle coordinate frame selection change"""
        # Convert display name to internal name
        frame_map = {"Base": "base", "Tool": "tool", "World": "world"}
        frame_name = frame_map.get(frame_text, "base")

        if frame_name != self.current_frame:
            self.current_frame = frame_name
            self.frame_changed.emit(frame_name)

    def get_frame(self):
        """Get current coordinate frame"""
        return self.current_frame

    def set_available_frames(self, frames):
        """Update the frame selector with available frames"""
        self.frame_selector.blockSignals(True)
        self.frame_selector.clear()
        for frame in frames:
            # Capitalize for display
            display_name = frame.capitalize() if frame else "Base"
            self.frame_selector.addItem(display_name)
        self.frame_selector.blockSignals(False)

    def _on_step_clicked(self, step_value):
        """Handle step button click"""
        self.current_step = step_value
        # Uncheck all buttons except clicked one
        for val, btn in self.step_buttons.items():
            btn.setChecked(val == step_value)
        self.step_changed.emit(step_value)

    def get_step(self):
        """Get current step value"""
        return self.current_step

    def _set_movement_type(self, move_type):
        """Switch between G0 (rapid) and G1 (feed) movement types"""
        if move_type == self.current_movement_type:
            return

        self.current_movement_type = move_type

        # Update button states
        self.g0_btn.setChecked(move_type == "G0")
        self.g1_btn.setChecked(move_type == "G1")

        # Enable/disable feedrate based on mode
        self.feedrate_spin.setEnabled(move_type == "G1")

        # Emit signal
        self.movement_type_changed.emit(move_type)

    def get_movement_type(self):
        """Get current movement type (G0 or G1)"""
        return self.current_movement_type

    def get_feedrate(self):
        """Get current feedrate value"""
        return float(self.feedrate_spin.value())

    def set_feedrate(self, value):
        """Set feedrate value"""
        self.feedrate_spin.setValue(int(value))


class RobotStatePanel(QFrame):
    """Right panel - always visible robot state with 3D visualisation and axis controls"""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Main content: 3D visualisation (left, expanding) + Axis controls (right, fixed)
        main_row = QHBoxLayout()
        main_row.setSpacing(5)

        # 3D visualisation - takes most space
        viz_frame = QFrame()
        viz_layout = QVBoxLayout(viz_frame)
        viz_layout.setContentsMargins(0, 0, 0, 0)
        viz_layout.setSpacing(5)

        self.robot_3d_canvas = Robot3DCanvas(self, width=7.5, height=6.5, dpi=95)
        self.robot_3d_canvas.setMinimumHeight(450)
        viz_layout.addWidget(self.robot_3d_canvas, 1)

        # Visualisation controls at bottom of 3D view
        viz_controls = QHBoxLayout()
        viz_controls.setSpacing(8)
        viz_controls.setContentsMargins(0, 0, 0, 0)
        self.show_trajectory_check = QCheckBox("Trajectory")
        self.show_trajectory_check.setChecked(True)
        self.auto_rotate_check = QCheckBox("Auto-rotate")
        self.auto_rotate_check.setChecked(False)
        viz_controls.addWidget(self.show_trajectory_check)
        viz_controls.addWidget(self.auto_rotate_check)
        viz_controls.addStretch()
        viz_layout.addLayout(viz_controls)

        main_row.addWidget(viz_frame, 1)  # stretch=1 to expand

        # Axis control column (right side, fixed width)
        self.axis_column = AxisControlColumn()
        main_row.addWidget(self.axis_column)

        layout.addLayout(main_row)

        # Keep backward-compatible references for bifrost.py
        # These will need to be connected differently but provide access points
        self.joint_table = None  # Removed - use axis_column.rows instead
        self.gripper_spinbox = None  # Removed - use axis_column.rows['Gripper'] instead
        self.gripper_slider = None  # Removed
        self.gripper_open_btn = None  # Removed
        self.gripper_close_btn = None  # Removed


class JointStatusTable(QTableWidget):
    """Unified joint status table with controls"""

    def __init__(self):
        super().__init__()

        # Setup table
        self.setRowCount(6)  # J1-J6 only (gripper separated)
        self.setColumnCount(8)

        headers = ["Joint", "Cmd", "Actual", "ES", "[-10]", "[-1]", "[+1]", "[+10]"]
        self.setHorizontalHeaderLabels(headers)

        # Configure table appearance
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # Fill available space
        self.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)  # Joint name fixed width
        self.horizontalHeader().resizeSection(0, 90)  # Set Joint column width to 90px
        self.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)  # ES fixed
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.setMinimumHeight(205)
        self.setMaximumHeight(225)

        # Populate rows (J1-J6 only, no gripper)
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        axis_labels = ["X", "Y", "Z", "U", "V", "W"]  # Corresponding axes

        for row, joint_name in enumerate(joint_names):
            # Joint name with axis label
            display_name = f"{joint_name} ({axis_labels[row]})"
            name_item = QTableWidgetItem(display_name)
            name_item.setTextAlignment(Qt.AlignCenter)
            name_font = QtGui.QFont()
            name_font.setBold(True)
            name_item.setFont(name_font)
            self.setItem(row, 0, name_item)

            # Command value (editable spinbox)
            cmd_spin = QDoubleSpinBox()
            cmd_spin.setAlignment(Qt.AlignCenter)
            # Set proper limits based on joint type (Art2, Art3, Art5 have 90° range)
            if joint_name in ['Art2', 'Art3', 'Art5']:
                cmd_spin.setMinimum(-90.0)
                cmd_spin.setMaximum(90.0)
            elif row < 6:  # Other joints (Art1, Art4, Art6)
                cmd_spin.setMinimum(-180.0)
                cmd_spin.setMaximum(180.0)
            else:  # Gripper
                cmd_spin.setMinimum(0.0)
                cmd_spin.setMaximum(100.0)
            cmd_spin.setDecimals(2 if row < 6 else 0)
            cmd_spin.setObjectName(f"SpinBox{joint_name}")
            self.setCellWidget(row, 1, cmd_spin)

            # Actual value (read-only)
            actual_item = QTableWidgetItem("0.0")
            actual_item.setTextAlignment(Qt.AlignCenter)
            actual_item.setFlags(Qt.ItemIsEnabled)
            actual_item.setBackground(QtGui.QColor(240, 240, 240))
            self.setItem(row, 2, actual_item)

            # Endstop status
            es_item = QTableWidgetItem("✓")
            es_item.setTextAlignment(Qt.AlignCenter)
            es_item.setBackground(QtGui.QColor(200, 255, 200))
            self.setItem(row, 3, es_item)

            # Inc/Dec buttons
            for col, delta in enumerate([-10, -1, 1, 10], start=4):
                btn = QPushButton(f"{delta:+d}")
                btn.setProperty("delta", delta)
                btn.setProperty("joint", joint_name)
                btn.setMinimumHeight(28)  # Make buttons taller
                # Remove max width - let buttons stretch to fill space
                self.setCellWidget(row, col, btn)

        # Store references for easy access
        self.spinboxes = {}
        self.actual_items = {}
        self.endstop_items = {}

        for row, joint_name in enumerate(joint_names):
            self.spinboxes[joint_name] = self.cellWidget(row, 1)
            self.actual_items[joint_name] = self.item(row, 2)
            self.endstop_items[joint_name] = self.item(row, 3)


class JogModePanel(QFrame):
    """MODE: JOG - Manual joint control"""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(380, 600)  # Increased from 400 to 600 for more height
        self.setStyleSheet("JogModePanel { background-color: #f5f5f5; }")  # Light grey background

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)  # Add spacing between items

        # Movement type
        move_group = QGroupBox("Movement Type")
        move_layout = QVBoxLayout(move_group)

        self.G0MoveRadioButton = QRadioButton("Rapid (G0)")
        self.G0MoveRadioButton.setChecked(True)
        move_layout.addWidget(self.G0MoveRadioButton)

        self.G1MoveRadioButton = QRadioButton("Feed (G1)")
        move_layout.addWidget(self.G1MoveRadioButton)

        # Feed rate
        feed_layout = QHBoxLayout()
        self.FeedRateLabel = QLabel("Feed Rate:")
        self.FeedRateLabel.setEnabled(False)
        feed_layout.addWidget(self.FeedRateLabel)

        self.FeedRateInput = QSpinBox()
        self.FeedRateInput.setRange(1, 10000)
        self.FeedRateInput.setValue(1000)
        self.FeedRateInput.setSuffix(" mm/min")
        self.FeedRateInput.setEnabled(False)
        feed_layout.addWidget(self.FeedRateInput)

        move_layout.addLayout(feed_layout)
        layout.addWidget(move_group)

        # Connect radio buttons to enable/disable feedrate
        self.G1MoveRadioButton.toggled.connect(self.FeedRateLabel.setEnabled)
        self.G1MoveRadioButton.toggled.connect(self.FeedRateInput.setEnabled)

        # Jog mode
        self.JogModeCheckBox = QCheckBox("☑ Jog Mode (Live Movement)")
        jog_font = QtGui.QFont()
        jog_font.setBold(True)
        self.JogModeCheckBox.setFont(jog_font)
        self.JogModeCheckBox.setStyleSheet("color: rgb(200, 80, 0);")
        layout.addWidget(self.JogModeCheckBox)

        # Execute Movement button (visible when jog mode is OFF)
        self.ExecuteMovementButton = QPushButton("▶ Execute Movement")
        self.ExecuteMovementButton.setMinimumHeight(50)
        self.ExecuteMovementButton.setMinimumWidth(200)
        exec_font = QtGui.QFont()
        exec_font.setPointSize(11)
        exec_font.setBold(True)
        self.ExecuteMovementButton.setFont(exec_font)
        self.ExecuteMovementButton.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: 2px solid #1976D2;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
            QPushButton:disabled {
                background-color: #BDBDBD;
                color: #757575;
                border: 2px solid #9E9E9E;
            }
        """)
        self.ExecuteMovementButton.setToolTip("Execute movement to all currently set joint positions (G0/G1)")
        layout.addWidget(self.ExecuteMovementButton)

        layout.addStretch()

        # Quick commands
        quick_group = QGroupBox("Quick Commands")
        quick_layout = QVBoxLayout(quick_group)

        self.HomeButton = QPushButton("🏠 Home All Axes")
        self.HomeButton.setMinimumHeight(40)
        self.HomeButton.setMinimumWidth(200)
        quick_layout.addWidget(self.HomeButton)

        self.ZeroPositionButton = QPushButton("Zero All Positions")
        self.ZeroPositionButton.setMinimumHeight(40)
        self.ZeroPositionButton.setMinimumWidth(200)
        quick_layout.addWidget(self.ZeroPositionButton)

        layout.addWidget(quick_group)


class InverseModePanel(QFrame):
    """MODE: INVERSE - Inverse kinematics control"""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title = QLabel("Cartesian Target Position")
        title_font = QtGui.QFont()
        title_font.setPointSize(10)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Visual directional pad
        pad_frame = QFrame()
        pad_frame.setFrameShape(QFrame.Box)
        pad_frame.setMinimumHeight(200)
        pad_layout = QGridLayout(pad_frame)

        # Y+ button
        self.IkIncButtonY = QPushButton("Y+")
        self.IkIncButtonY.setMinimumSize(60, 60)
        pad_layout.addWidget(self.IkIncButtonY, 0, 1)

        # X- button
        self.IkDecButtonX = QPushButton("X-")
        self.IkDecButtonX.setMinimumSize(60, 60)
        pad_layout.addWidget(self.IkDecButtonX, 1, 0)

        # Centre label
        center_label = QLabel("[XYZ]")
        center_label.setAlignment(Qt.AlignCenter)
        center_label.setStyleSheet("border: 2px solid #999; border-radius: 5px; font-weight: bold;")
        center_label.setMinimumSize(60, 60)
        pad_layout.addWidget(center_label, 1, 1)

        # X+ button
        self.IkIncButtonX = QPushButton("X+")
        self.IkIncButtonX.setMinimumSize(60, 60)
        pad_layout.addWidget(self.IkIncButtonX, 1, 2)

        # Y- button
        self.IkDecButtonY = QPushButton("Y-")
        self.IkDecButtonY.setMinimumSize(60, 60)
        pad_layout.addWidget(self.IkDecButtonY, 2, 1)

        layout.addWidget(pad_frame)

        # Z axis controls
        z_group = QGroupBox("Z Axis")
        z_layout = QVBoxLayout(z_group)

        z_btn_layout = QHBoxLayout()
        self.IkIncButtonZ = QPushButton("▲ Z+")
        self.IkIncButtonZ.setMinimumHeight(40)
        z_btn_layout.addWidget(self.IkIncButtonZ)

        self.IkDecButtonZ = QPushButton("▼ Z-")
        self.IkDecButtonZ.setMinimumHeight(40)
        z_btn_layout.addWidget(self.IkDecButtonZ)

        z_layout.addLayout(z_btn_layout)
        layout.addWidget(z_group)

        # Position inputs
        pos_group = QGroupBox("Target Position")
        pos_layout = QGridLayout(pos_group)

        # X
        pos_layout.addWidget(QLabel("X:"), 0, 0)
        self.IKInputSpinBoxX = QDoubleSpinBox()
        self.IKInputSpinBoxX.setRange(-999, 999)
        self.IKInputSpinBoxX.setDecimals(2)
        self.IKInputSpinBoxX.setSuffix(" mm")
        pos_layout.addWidget(self.IKInputSpinBoxX, 0, 1)

        # Y
        pos_layout.addWidget(QLabel("Y:"), 1, 0)
        self.IKInputSpinBoxY = QDoubleSpinBox()
        self.IKInputSpinBoxY.setRange(-999, 999)
        self.IKInputSpinBoxY.setDecimals(2)
        self.IKInputSpinBoxY.setSuffix(" mm")
        pos_layout.addWidget(self.IKInputSpinBoxY, 1, 1)

        # Z
        pos_layout.addWidget(QLabel("Z:"), 2, 0)
        self.IKInputSpinBoxZ = QDoubleSpinBox()
        self.IKInputSpinBoxZ.setRange(-999, 999)
        self.IKInputSpinBoxZ.setDecimals(2)
        self.IKInputSpinBoxZ.setSuffix(" mm")
        pos_layout.addWidget(self.IKInputSpinBoxZ, 2, 1)

        # A (orientation)
        pos_layout.addWidget(QLabel("A:"), 3, 0)
        self.IKInputSpinBoxA = QDoubleSpinBox()
        self.IKInputSpinBoxA.setRange(-180, 180)
        self.IKInputSpinBoxA.setDecimals(2)
        self.IKInputSpinBoxA.setSuffix(" °")
        pos_layout.addWidget(self.IKInputSpinBoxA, 3, 1)

        # B (orientation)
        pos_layout.addWidget(QLabel("B:"), 4, 0)
        self.IKInputSpinBoxB = QDoubleSpinBox()
        self.IKInputSpinBoxB.setRange(-180, 180)
        self.IKInputSpinBoxB.setDecimals(2)
        self.IKInputSpinBoxB.setSuffix(" °")
        pos_layout.addWidget(self.IKInputSpinBoxB, 4, 1)

        # C (orientation)
        pos_layout.addWidget(QLabel("C:"), 5, 0)
        self.IKInputSpinBoxC = QDoubleSpinBox()
        self.IKInputSpinBoxC.setRange(-180, 180)
        self.IKInputSpinBoxC.setDecimals(2)
        self.IKInputSpinBoxC.setSuffix(" °")
        pos_layout.addWidget(self.IKInputSpinBoxC, 5, 1)

        layout.addWidget(pos_group)

        # Calculate button
        self.CalculateIKButton = QPushButton("Calculate IK Solution")
        self.CalculateIKButton.setMinimumHeight(35)
        layout.addWidget(self.CalculateIKButton)

        # IK solution display
        self.IkOutputValueFrame = QFrame()
        self.IkOutputValueFrame.setFrameShape(QFrame.Box)
        self.IkOutputValueFrame.setStyleSheet("background-color: rgb(255, 255, 255);")
        sol_layout = QVBoxLayout(self.IkOutputValueFrame)

        # Individual joint output labels (required by bifrost.py)
        output_grid = QGridLayout()
        output_grid.addWidget(QLabel("X:"), 0, 0)
        self.IkOutputValueX = QLabel("--")
        self.IkOutputValueX.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueX, 0, 1)

        output_grid.addWidget(QLabel("Y:"), 0, 2)
        self.IkOutputValueY = QLabel("--")
        self.IkOutputValueY.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueY, 0, 3)

        output_grid.addWidget(QLabel("Z:"), 0, 4)
        self.IkOutputValueZ = QLabel("--")
        self.IkOutputValueZ.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueZ, 0, 5)

        # Orientation output labels (A, B, C)
        output_grid.addWidget(QLabel("A:"), 1, 0)
        self.IkOutputValueA = QLabel("--")
        self.IkOutputValueA.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueA, 1, 1)

        output_grid.addWidget(QLabel("B:"), 1, 2)
        self.IkOutputValueB = QLabel("--")
        self.IkOutputValueB.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueB, 1, 3)

        output_grid.addWidget(QLabel("C:"), 1, 4)
        self.IkOutputValueC = QLabel("--")
        self.IkOutputValueC.setAlignment(Qt.AlignCenter)
        output_grid.addWidget(self.IkOutputValueC, 1, 5)

        sol_layout.addLayout(output_grid)

        layout.addWidget(self.IkOutputValueFrame)

        layout.addStretch()


class PointEditDialog(QtWidgets.QDialog):
    """Dialog for adding/editing sequence points manually"""

    def __init__(self, parent=None, point_data=None, point_index=None):
        """
        Args:
            parent: Parent widget
            point_data: Dict with q1-q6, gripper, delay for editing (None for new point)
            point_index: Index of point being edited (None for new point)
        """
        super().__init__(parent)
        self.point_index = point_index
        self.setWindowTitle("Edit Point" if point_data else "Add Manual Point")
        self.setModal(True)
        self.setMinimumWidth(300)

        layout = QVBoxLayout(self)

        # Joint inputs
        joints_group = QGroupBox("Joint Positions (degrees)")
        joints_layout = QGridLayout(joints_group)

        self.joint_spinboxes = {}
        joint_names = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        for i, name in enumerate(joint_names):
            row, col = i // 2, (i % 2) * 2
            joints_layout.addWidget(QLabel(f"{name.upper()}:"), row, col)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-360.0, 360.0)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setSuffix("°")
            if point_data:
                spinbox.setValue(point_data.get(name, 0.0))
            self.joint_spinboxes[name] = spinbox
            joints_layout.addWidget(spinbox, row, col + 1)

        layout.addWidget(joints_group)

        # Gripper and delay
        extras_group = QGroupBox("Gripper & Timing")
        extras_layout = QGridLayout(extras_group)

        extras_layout.addWidget(QLabel("Gripper:"), 0, 0)
        self.gripper_spinbox = QDoubleSpinBox()
        self.gripper_spinbox.setRange(0.0, 100.0)
        self.gripper_spinbox.setDecimals(0)
        self.gripper_spinbox.setSingleStep(5.0)
        self.gripper_spinbox.setSuffix("%")
        if point_data:
            self.gripper_spinbox.setValue(point_data.get('gripper', 0.0))
        extras_layout.addWidget(self.gripper_spinbox, 0, 1)

        extras_layout.addWidget(QLabel("Delay:"), 1, 0)
        self.delay_spinbox = QDoubleSpinBox()
        self.delay_spinbox.setRange(0.0, 60.0)
        self.delay_spinbox.setDecimals(1)
        self.delay_spinbox.setSingleStep(0.5)
        self.delay_spinbox.setSuffix("s")
        self.delay_spinbox.setValue(point_data.get('delay', 1.0) if point_data else 1.0)
        extras_layout.addWidget(self.delay_spinbox, 1, 1)

        layout.addWidget(extras_group)

        # Buttons
        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect(self.accept)
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)
        layout.addLayout(button_layout)

    def get_point_data(self):
        """Return dict with all point values"""
        return {
            'q1': self.joint_spinboxes['q1'].value(),
            'q2': self.joint_spinboxes['q2'].value(),
            'q3': self.joint_spinboxes['q3'].value(),
            'q4': self.joint_spinboxes['q4'].value(),
            'q5': self.joint_spinboxes['q5'].value(),
            'q6': self.joint_spinboxes['q6'].value(),
            'gripper': self.gripper_spinbox.value(),
            'delay': self.delay_spinbox.value()
        }


class TeachModePanel(QFrame):
    """MODE: TEACH - Sequence programming"""

    # Signals for manual point operations
    manualPointRequested = pyqtSignal()  # Request to open add dialog
    pointEditRequested = pyqtSignal(int)  # Request to edit point at index
    importCsvRequested = pyqtSignal()  # Request to import CSV file

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        # Sequence name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Sequence:"))
        self.SequenceNameEdit = QLineEdit("Untitled_Sequence")
        name_layout.addWidget(self.SequenceNameEdit)
        layout.addLayout(name_layout)

        # Point list
        list_label = QLabel("Sequence Points:")
        layout.addWidget(list_label)

        self.sequencePointsList = QListWidget()
        self.sequencePointsList.setMinimumHeight(250)
        layout.addWidget(self.sequencePointsList, 1)  # Stretch factor 1 to fill remaining space

        # Recording controls
        rec_group = QGroupBox("Recording")
        rec_layout = QVBoxLayout(rec_group)

        self.sequenceRecordButton = QPushButton("⏺ Record Current Position")
        self.sequenceRecordButton.setMinimumHeight(35)
        rec_layout.addWidget(self.sequenceRecordButton)

        self.sequenceAddManualButton = QPushButton("✏ Add Manual Point")
        self.sequenceAddManualButton.setMinimumHeight(30)
        self.sequenceAddManualButton.clicked.connect(lambda: self.manualPointRequested.emit())
        rec_layout.addWidget(self.sequenceAddManualButton)

        self.sequenceDeleteButton = QPushButton("🗑 Delete Selected Point")
        self.sequenceDeleteButton.setMinimumHeight(30)
        rec_layout.addWidget(self.sequenceDeleteButton)

        self.sequenceClearButton = QPushButton("🆕 Clear All Points")
        self.sequenceClearButton.setMinimumHeight(30)
        rec_layout.addWidget(self.sequenceClearButton)

        layout.addWidget(rec_group)

        # Playback controls
        playback_group = QGroupBox("Playback")
        playback_layout = QVBoxLayout(playback_group)

        # Play/Pause/Stop buttons
        play_btn_layout = QHBoxLayout()
        self.sequencePlayButton = QPushButton("▶ Play")
        self.sequencePlayButton.setMinimumHeight(30)
        play_btn_layout.addWidget(self.sequencePlayButton)

        self.sequencePauseButton = QPushButton("⏸ Pause")
        self.sequencePauseButton.setMinimumHeight(30)
        self.sequencePauseButton.setEnabled(False)
        play_btn_layout.addWidget(self.sequencePauseButton)

        self.sequenceStopButton = QPushButton("⏹ Stop")
        self.sequenceStopButton.setMinimumHeight(30)
        self.sequenceStopButton.setEnabled(False)
        play_btn_layout.addWidget(self.sequenceStopButton)

        playback_layout.addLayout(play_btn_layout)

        # Speed and loop controls
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed:"))
        self.sequenceSpeedSpinBox = QDoubleSpinBox()
        self.sequenceSpeedSpinBox.setRange(0.1, 10.0)
        self.sequenceSpeedSpinBox.setSingleStep(0.1)
        self.sequenceSpeedSpinBox.setValue(1.0)
        self.sequenceSpeedSpinBox.setSuffix("x")
        speed_layout.addWidget(self.sequenceSpeedSpinBox)

        self.sequenceLoopCheckBox = QCheckBox("Loop")
        speed_layout.addWidget(self.sequenceLoopCheckBox)

        playback_layout.addLayout(speed_layout)

        # Delay control
        delay_layout = QHBoxLayout()
        delay_layout.addWidget(QLabel("Delay:"))
        self.sequenceDelaySpinBox = QDoubleSpinBox()
        self.sequenceDelaySpinBox.setRange(0.0, 60.0)
        self.sequenceDelaySpinBox.setSingleStep(0.1)
        self.sequenceDelaySpinBox.setValue(1.0)
        self.sequenceDelaySpinBox.setSuffix("s")
        delay_layout.addWidget(self.sequenceDelaySpinBox)

        playback_layout.addLayout(delay_layout)

        layout.addWidget(playback_group)

        # File operations
        file_layout = QHBoxLayout()
        self.sequenceSaveButton = QPushButton("💾 Save")
        file_layout.addWidget(self.sequenceSaveButton)

        self.sequenceLoadButton = QPushButton("📂 Load")
        file_layout.addWidget(self.sequenceLoadButton)

        self.sequenceImportCsvButton = QPushButton("📥 Import CSV")
        self.sequenceImportCsvButton.clicked.connect(lambda: self.importCsvRequested.emit())
        file_layout.addWidget(self.sequenceImportCsvButton)

        layout.addLayout(file_layout)

        # Double-click to edit point
        self.sequencePointsList.itemDoubleClicked.connect(self._on_point_double_clicked)

    def _on_point_double_clicked(self, item):
        """Handle double-click on a point to request editing"""
        index = self.sequencePointsList.row(item)
        self.pointEditRequested.emit(index)


class TerminalModePanel(QFrame):
    """MODE: TERMINAL - Console and debugging"""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        # Command input
        input_group = QGroupBox("G-Code / Command Input")
        input_layout = QVBoxLayout(input_group)

        cmd_layout = QHBoxLayout()
        self.ConsoleInput = QLineEdit()
        self.ConsoleInput.setPlaceholderText("Enter command (e.g., M114, G28, etc.)")
        cmd_layout.addWidget(self.ConsoleInput)

        self.ConsoleButtonSend = QPushButton("Send")
        self.ConsoleButtonSend.setMinimumWidth(80)
        cmd_layout.addWidget(self.ConsoleButtonSend)

        input_layout.addLayout(cmd_layout)

        # Quick commands
        quick_layout = QHBoxLayout()
        self.QuickM114Button = QPushButton("M114 (Position)")
        quick_layout.addWidget(self.QuickM114Button)

        self.QuickM119Button = QPushButton("M119 (Endstops)")
        quick_layout.addWidget(self.QuickM119Button)

        self.QuickG28Button = QPushButton("G28 (Home)")
        quick_layout.addWidget(self.QuickG28Button)

        input_layout.addLayout(quick_layout)

        layout.addWidget(input_group)

        # Console options
        options_layout = QHBoxLayout()
        self.ConsoleShowVerbosecheckBox = QCheckBox("Show M114 Verbose")
        options_layout.addWidget(self.ConsoleShowVerbosecheckBox)

        self.ConsoleShowOkRespcheckBox = QCheckBox("Show 'ok' Responses")
        options_layout.addWidget(self.ConsoleShowOkRespcheckBox)

        options_layout.addStretch()
        layout.addLayout(options_layout)

        # Console output
        output_label = QLabel("Console Output:")
        layout.addWidget(output_label)

        self.ConsoleOutput = QPlainTextEdit()
        self.ConsoleOutput.setReadOnly(True)
        self.ConsoleOutput.setMinimumHeight(300)
        layout.addWidget(self.ConsoleOutput, 1)  # Stretch factor 1 to fill remaining space

        # Clear button
        self.ConsoleClearButton = QPushButton("Clear Console")
        layout.addWidget(self.ConsoleClearButton)


class Ui_MainWindow:
    """Main UI class compatible with existing bifrost.py interface"""

    def setup_mode_panels(self):
        """Create all mode-specific panels (JOG removed - controls in sidebar)"""
        # Mode 0: INVERSE
        self.inverse_panel = InverseModePanel()
        self.mode_stack.addWidget(self.inverse_panel)

        # Mode 1: TEACH
        self.teach_panel = TeachModePanel()
        self.mode_stack.addWidget(self.teach_panel)

        # Mode 2: TERMINAL
        self.terminal_panel = TerminalModePanel()
        self.mode_stack.addWidget(self.terminal_panel)

        # Mode 3: CALIBRATE (includes DH parameters)
        from calibration_panel import CalibrationPanel
        # gui_instance will be set later by BifrostGUI
        self.calibration_panel = CalibrationPanel(gui_instance=None)
        self.mode_stack.addWidget(self.calibration_panel)

        # Mode 4: FRAMES
        from frame_panel import FrameManagementPanel
        self.frames_panel = FrameManagementPanel()
        self.mode_stack.addWidget(self.frames_panel)

    def switch_mode(self, mode_index):
        """Switch to specified mode"""
        import logging
        logger = logging.getLogger(__name__)
        logger.info(f"Switching to mode {mode_index}, stack has {self.mode_stack.count()} widgets")
        self.mode_stack.setCurrentIndex(mode_index)
        current = self.mode_stack.currentWidget()
        if current:
            logger.info(f"Current widget after switch: {current.__class__.__name__}")
            current.show()  # Force show the current widget
            current.raise_()  # Bring to front

    def _fix_splitter_sizes(self):
        """Force splitter to correct sizes after window is shown"""
        if hasattr(self, 'content_splitter'):
            self.content_splitter.setSizes([480, 720])
            # Force all widgets to show
            self.top_bar.show()
            self.mode_selector.show()
            self.mode_stack.show()
            self.robot_state_panel.show()

    def setupUi(self, MainWindow):
        """Setup the modern UI directly in MainWindow"""
        # Setup window properties
        MainWindow.setWindowTitle("Bifrost - Thor Robot Control")
        MainWindow.setMinimumSize(1300, 800)
        MainWindow.resize(1400, 850)  # Default size to ensure everything fits

        # Create central widget
        self.centralwidget = QWidget()
        MainWindow.setCentralWidget(self.centralwidget)

        # Main layout
        main_layout = QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        # Top bar (connection status)
        self.top_bar = ConnectionBar()
        main_layout.addWidget(self.top_bar)

        # Mode selector bar
        self.mode_selector = ModeSelectorBar()
        main_layout.addWidget(self.mode_selector)

        # Main content area (split left/right)
        content_splitter = QtWidgets.QSplitter(Qt.Horizontal)

        # Left panel: Mode-specific controls (40%)
        self.mode_stack = QStackedWidget()
        content_splitter.addWidget(self.mode_stack)

        # Right panel: Unified robot state (60%)
        self.robot_state_panel = RobotStatePanel()
        content_splitter.addWidget(self.robot_state_panel)

        # Set splitter proportions and initial sizes
        content_splitter.setStretchFactor(0, 40)  # Left: 40%
        content_splitter.setStretchFactor(1, 60)  # Right: 60%

        # Force initial sizes (40% / 60% of 1200px window = 480px / 720px)
        content_splitter.setSizes([480, 720])

        # Set minimum width for left panel so it can't be collapsed
        self.mode_stack.setMinimumWidth(400)

        # Prevent splitter panels from collapsing
        content_splitter.setCollapsible(0, False)  # Left panel
        content_splitter.setCollapsible(1, False)  # Right panel

        main_layout.addWidget(content_splitter)

        # Store reference for later use
        self.content_splitter = content_splitter

        # Create mode panels
        self.setup_mode_panels()

        # Connect mode switching
        self.mode_selector.mode_changed.connect(self.switch_mode)

        # Default to JOG mode
        self.switch_mode(0)

        # CRITICAL FIX: Apply splitter sizes after Qt processes events
        # This ensures window is fully realized before setting sizes
        QtCore.QTimer.singleShot(10, self._fix_splitter_sizes)

        # Now forward widget references for bifrost.py compatibility

        # Connection bar widgets
        self.SerialPortComboBox = self.top_bar.SerialPortComboBox
        self.SerialPortRefreshButton = self.top_bar.SerialPortRefreshButton
        self.BaudRateComboBox = self.top_bar.BaudRateComboBox
        self.RobotStateDisplay = self.top_bar.RobotStateDisplay
        self.ConnectButton = self.top_bar.ConnectButton
        self.EmergencyStopButton = self.top_bar.EmergencyStopButton
        self.SettingsButton = self.top_bar.SettingsButton

        # Axis control column widgets (new vertical layout)
        axis_column = self.robot_state_panel.axis_column

        # Create hidden spinboxes for joint command values (bifrost.py expects these)
        # The actual display is read-only labels in the axis column
        # Pass MainWindow as parent to prevent floating windows
        self.SpinBoxArt1 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt1.setRange(-180, 180)
        self.SpinBoxArt1.setVisible(False)
        self.SpinBoxArt2 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt2.setRange(-90, 90)
        self.SpinBoxArt2.setVisible(False)
        self.SpinBoxArt3 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt3.setRange(-90, 90)
        self.SpinBoxArt3.setVisible(False)
        self.SpinBoxArt4 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt4.setRange(-180, 180)
        self.SpinBoxArt4.setVisible(False)
        self.SpinBoxArt5 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt5.setRange(-90, 90)
        self.SpinBoxArt5.setVisible(False)
        self.SpinBoxArt6 = QDoubleSpinBox(MainWindow)
        self.SpinBoxArt6.setRange(-180, 180)
        self.SpinBoxArt6.setVisible(False)
        self.SpinBoxGripper = QSpinBox(MainWindow)
        self.SpinBoxGripper.setRange(0, 100)
        self.SpinBoxGripper.setVisible(False)

        # Actual position labels - use the value_label from axis rows
        self.FKCurrentPosValueArt1 = axis_column.rows["J1"].value_label
        self.FKCurrentPosValueArt2 = axis_column.rows["J2"].value_label
        self.FKCurrentPosValueArt3 = axis_column.rows["J3"].value_label
        self.FKCurrentPosValueArt4 = axis_column.rows["J4"].value_label
        self.FKCurrentPosValueArt5 = axis_column.rows["J5"].value_label
        self.FKCurrentPosValueArt6 = axis_column.rows["J6"].value_label

        # Endstop indicators - use the endstop_indicator from axis rows
        self.endstopLabelArt1 = axis_column.rows["J1"].endstop_indicator
        self.endstopLabelArt2 = axis_column.rows["J2"].endstop_indicator
        self.endstopLabelArt3 = axis_column.rows["J3"].endstop_indicator
        self.endstopLabelArt4 = axis_column.rows["J4"].endstop_indicator
        self.endstopLabelArt5 = axis_column.rows["J5"].endstop_indicator
        self.endstopLabelArt6 = axis_column.rows["J6"].endstop_indicator

        # Store axis column reference for step-aware button connections
        self.axis_column = axis_column

        # Map +/- buttons from axis rows to bifrost expected names
        # The step size is controlled by the axis_column.current_step toggle
        joint_mapping = [("J1", "Art1"), ("J2", "Art2"), ("J3", "Art3"),
                         ("J4", "Art4"), ("J5", "Art5"), ("J6", "Art6")]
        for axis_name, joint_name in joint_mapping:
            row = axis_column.rows[axis_name]
            # Store the +/- buttons - they will use variable step from toggle
            setattr(self, f"FKMinusButton{joint_name}", row.minus_btn)
            setattr(self, f"FKPlusButton{joint_name}", row.plus_btn)

        # Gripper +/- buttons
        gripper_row = axis_column.rows["Gripper"]
        self.FKMinusButtonGripper = gripper_row.minus_btn
        self.FKPlusButtonGripper = gripper_row.plus_btn

        # Create dummy buttons for old fixed-increment buttons (hidden, for compatibility)
        for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
            for prefix in ['FKDec10Button', 'FKDec1Button', 'FKDec0_1Button',
                          'FKInc0_1Button', 'FKInc1Button', 'FKInc10Button']:
                btn = QPushButton()
                btn.setVisible(False)
                setattr(self, f"{prefix}{joint}", btn)

        # Gripper increment buttons (hidden dummies)
        self.FKDec10ButtonGripper = QPushButton()
        self.FKDec10ButtonGripper.setVisible(False)
        self.FKDec1ButtonGripper = QPushButton()
        self.FKDec1ButtonGripper.setVisible(False)
        self.FKInc1ButtonGripper = QPushButton()
        self.FKInc1ButtonGripper.setVisible(False)
        self.FKInc10ButtonGripper = QPushButton()
        self.FKInc10ButtonGripper.setVisible(False)

        # No gripper preset buttons in new UI
        self.gripper_close_btn = None
        self.gripper_open_btn = None

        # Individual Go buttons - create hidden dummy buttons
        for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
            btn = QPushButton()
            btn.setVisible(False)
            setattr(self, f"FKGoButton{joint}", btn)

        self.GoButtonGripper = QPushButton()
        self.GoButtonGripper.setVisible(False)

        # Sliders - create hidden dummy sliders for compatibility
        for joint in ['Art1', 'Art2', 'Art3', 'Art4', 'Art5', 'Art6']:
            slider = QtWidgets.QSlider()
            slider.setVisible(False)
            setattr(self, f"FKSlider{joint}", slider)

        # Gripper slider (hidden dummy)
        self.FKSliderGripper = QtWidgets.QSlider()
        self.FKSliderGripper.setVisible(False)
        self.SliderGripper = self.FKSliderGripper

        # 3D visualisation canvas and controls
        self.position_canvas = self.robot_state_panel.robot_3d_canvas
        self.show_trajectory_check = self.robot_state_panel.show_trajectory_check
        self.auto_rotate_check = self.robot_state_panel.auto_rotate_check

        # Gripper buttons also need non-FK aliases for compatibility
        self.Dec10ButtonGripper = self.FKDec10ButtonGripper
        self.Dec1ButtonGripper = self.FKDec1ButtonGripper
        self.Inc1ButtonGripper = self.FKInc1ButtonGripper
        self.Inc10ButtonGripper = self.FKInc10ButtonGripper

        # JOG mode widgets - now from axis column (JOG panel removed)
        # Quick command buttons from axis column
        self.HomeButton = axis_column.HomeButton
        self.ZeroPositionButton = axis_column.ZeroPositionButton

        # Movement type from axis column (G0/G1 buttons replaced radio buttons)
        # Create dummy radio buttons that mirror axis column state for compatibility
        # Pass MainWindow as parent to prevent floating windows
        self.G0MoveRadioButton = QRadioButton("G0", MainWindow)
        self.G0MoveRadioButton.setChecked(True)
        self.G0MoveRadioButton.setVisible(False)
        self.G1MoveRadioButton = QRadioButton("G1", MainWindow)
        self.G1MoveRadioButton.setVisible(False)

        # Feedrate from axis column
        self.FeedRateInput = axis_column.feedrate_spin
        self.FeedRateLabel = QLabel("F:", MainWindow)  # Dummy label
        self.FeedRateLabel.setVisible(False)

        # Jog mode checkbox - create in axis column
        self.JogModeCheckBox = axis_column.jog_mode_checkbox

        # Execute button - not needed since jog mode auto-executes
        self.ExecuteMovementButton = QPushButton("Go", MainWindow)
        self.ExecuteMovementButton.setVisible(False)

        # Map FKGoAllButton to ExecuteMovementButton for compatibility
        self.FKGoAllButton = self.ExecuteMovementButton

        # INVERSE mode widgets
        self.IKInputSpinBoxX = self.inverse_panel.IKInputSpinBoxX
        self.IKInputSpinBoxY = self.inverse_panel.IKInputSpinBoxY
        self.IKInputSpinBoxZ = self.inverse_panel.IKInputSpinBoxZ
        self.IKInputSpinBoxA = self.inverse_panel.IKInputSpinBoxA
        self.IKInputSpinBoxB = self.inverse_panel.IKInputSpinBoxB
        self.IKInputSpinBoxC = self.inverse_panel.IKInputSpinBoxC
        self.IkIncButtonX = self.inverse_panel.IkIncButtonX
        self.IkDecButtonX = self.inverse_panel.IkDecButtonX
        self.IkIncButtonY = self.inverse_panel.IkIncButtonY
        self.IkDecButtonY = self.inverse_panel.IkDecButtonY
        self.IkIncButtonZ = self.inverse_panel.IkIncButtonZ
        self.IkDecButtonZ = self.inverse_panel.IkDecButtonZ
        self.IkOutputValueFrame = self.inverse_panel.IkOutputValueFrame
        self.IkOutputValueX = self.inverse_panel.IkOutputValueX
        self.IkOutputValueY = self.inverse_panel.IkOutputValueY
        self.IkOutputValueZ = self.inverse_panel.IkOutputValueZ
        self.IkOutputValueA = self.inverse_panel.IkOutputValueA
        self.IkOutputValueB = self.inverse_panel.IkOutputValueB
        self.IkOutputValueC = self.inverse_panel.IkOutputValueC

        # Create dummy IK labels/widgets that bifrost.py checks for enabled status
        # Pass MainWindow as parent to prevent floating windows
        self.InverseKinematicsLabel = QLabel(MainWindow)  # Dummy label
        self.InverseKinematicsLabel.setVisible(False)
        self.IkOutputValueFrame_dummy = QFrame(MainWindow)  # Dummy frame (separate from the real one)
        self.IkOutputValueFrame_dummy.setVisible(False)

        # TEACH mode widgets
        self.sequencePointsList = self.teach_panel.sequencePointsList
        self.sequenceRecordButton = self.teach_panel.sequenceRecordButton
        self.sequenceAddManualButton = self.teach_panel.sequenceAddManualButton
        self.sequenceDeleteButton = self.teach_panel.sequenceDeleteButton
        self.sequenceClearButton = self.teach_panel.sequenceClearButton
        self.sequenceSaveButton = self.teach_panel.sequenceSaveButton
        self.sequenceLoadButton = self.teach_panel.sequenceLoadButton
        self.sequenceImportCsvButton = self.teach_panel.sequenceImportCsvButton

        # Add sequence playback widgets
        self.sequencePlayButton = self.teach_panel.sequencePlayButton
        self.sequencePauseButton = self.teach_panel.sequencePauseButton
        self.sequenceStopButton = self.teach_panel.sequenceStopButton
        self.sequenceSpeedSpinBox = self.teach_panel.sequenceSpeedSpinBox
        self.sequenceLoopCheckBox = self.teach_panel.sequenceLoopCheckBox
        self.sequenceDelaySpinBox = self.teach_panel.sequenceDelaySpinBox

        # TERMINAL mode widgets
        self.ConsoleInput = self.terminal_panel.ConsoleInput
        self.ConsoleButtonSend = self.terminal_panel.ConsoleButtonSend
        self.ConsoleOutput = self.terminal_panel.ConsoleOutput
        self.ConsoleShowVerbosecheckBox = self.terminal_panel.ConsoleShowVerbosecheckBox
        self.ConsoleShowOkRespcheckBox = self.terminal_panel.ConsoleShowOkRespcheckBox
        self.ConsoleClearButton = self.terminal_panel.ConsoleClearButton
        self.QuickM114Button = self.terminal_panel.QuickM114Button
        self.QuickM119Button = self.terminal_panel.QuickM119Button
        self.QuickG28Button = self.terminal_panel.QuickG28Button
