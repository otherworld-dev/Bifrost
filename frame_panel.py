"""
Frame Management Panel for Thor Robot Arm GUI

Provides UI for:
- Frame/tool selection
- Creating/deleting frames
- 3-point teaching workflow
- Frame configuration editing
"""

import logging
from typing import Optional, Callable, List

from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QLineEdit,
    QDoubleSpinBox, QGroupBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QWidget, QStackedWidget,
    QProgressBar, QTextEdit
)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5 import QtGui

from frame_controller import FrameController
from frame_teaching import TeachingProgress, TeachingState

logger = logging.getLogger(__name__)


class FrameSelectionWidget(QFrame):
    """Compact frame and tool selector for embedding in other panels"""

    frame_changed = pyqtSignal(str)
    tool_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # Frame selector
        layout.addWidget(QLabel("Frame:"))
        self.frame_combo = QComboBox()
        self.frame_combo.setMinimumWidth(100)
        self.frame_combo.currentTextChanged.connect(self._on_frame_changed)
        layout.addWidget(self.frame_combo)

        # Tool selector
        layout.addWidget(QLabel("Tool:"))
        self.tool_combo = QComboBox()
        self.tool_combo.setMinimumWidth(100)
        self.tool_combo.currentTextChanged.connect(self._on_tool_changed)
        layout.addWidget(self.tool_combo)

        layout.addStretch()

    def update_frames(self, frames: List[str]):
        """Update frame list"""
        current = self.frame_combo.currentText()
        self.frame_combo.blockSignals(True)
        self.frame_combo.clear()
        self.frame_combo.addItems(frames)
        if current in frames:
            self.frame_combo.setCurrentText(current)
        self.frame_combo.blockSignals(False)

    def update_tools(self, tools: List[str]):
        """Update tool list"""
        current = self.tool_combo.currentText()
        self.tool_combo.blockSignals(True)
        self.tool_combo.clear()
        self.tool_combo.addItems(tools)
        if current in tools:
            self.tool_combo.setCurrentText(current)
        self.tool_combo.blockSignals(False)

    def set_current_frame(self, frame: str):
        """Set current frame selection"""
        self.frame_combo.blockSignals(True)
        self.frame_combo.setCurrentText(frame)
        self.frame_combo.blockSignals(False)

    def set_current_tool(self, tool: str):
        """Set current tool selection"""
        self.tool_combo.blockSignals(True)
        self.tool_combo.setCurrentText(tool)
        self.tool_combo.blockSignals(False)

    def _on_frame_changed(self, text):
        if text:
            self.frame_changed.emit(text)

    def _on_tool_changed(self, text):
        if text:
            self.tool_changed.emit(text)


class FrameManagementPanel(QFrame):
    """Full frame management panel for FRAMES mode"""

    def __init__(self, frame_controller: Optional[FrameController] = None):
        super().__init__()
        self.frame_controller = frame_controller
        self.setFrameShape(QFrame.StyledPanel)

        self._setup_ui()

    def set_controller(self, controller: FrameController):
        """Set or update frame controller"""
        self.frame_controller = controller

        # Connect controller callbacks
        controller.on_frames_updated = self._on_frames_updated
        controller.on_tools_updated = self._on_tools_updated
        controller.on_workpieces_updated = self._on_workpieces_updated
        controller.on_teaching_progress = self._on_teaching_progress

        # Initialise UI state
        self._refresh_all_lists()

    def _setup_ui(self):
        """Setup the panel UI"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Title
        title = QLabel("Coordinate Frame Management")
        title_font = QtGui.QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        main_layout.addWidget(title)

        # Frame selection
        selection_group = QGroupBox("Active Frame Selection")
        selection_layout = QHBoxLayout(selection_group)

        selection_layout.addWidget(QLabel("Working Frame:"))
        self.frame_combo = QComboBox()
        self.frame_combo.setMinimumWidth(150)
        self.frame_combo.currentTextChanged.connect(self._on_frame_selected)
        selection_layout.addWidget(self.frame_combo)

        selection_layout.addWidget(QLabel("Active Tool:"))
        self.tool_combo = QComboBox()
        self.tool_combo.setMinimumWidth(120)
        self.tool_combo.currentTextChanged.connect(self._on_tool_selected)
        selection_layout.addWidget(self.tool_combo)

        selection_layout.addStretch()
        main_layout.addWidget(selection_group)

        # Workpiece frames section
        workpiece_group = QGroupBox("Workpiece Frames")
        workpiece_layout = QVBoxLayout(workpiece_group)

        # Workpiece list
        self.workpiece_table = QTableWidget()
        self.workpiece_table.setColumnCount(4)
        self.workpiece_table.setHorizontalHeaderLabels(["Name", "X", "Y", "Z"])
        self.workpiece_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.workpiece_table.setMaximumHeight(150)
        workpiece_layout.addWidget(self.workpiece_table)

        # Workpiece buttons
        wp_btn_layout = QHBoxLayout()
        self.btn_teach_workpiece = QPushButton("Teach New (3-Point)")
        self.btn_teach_workpiece.clicked.connect(self._start_workpiece_teaching)
        wp_btn_layout.addWidget(self.btn_teach_workpiece)

        self.btn_delete_workpiece = QPushButton("Delete Selected")
        self.btn_delete_workpiece.clicked.connect(self._delete_selected_workpiece)
        wp_btn_layout.addWidget(self.btn_delete_workpiece)

        wp_btn_layout.addStretch()
        workpiece_layout.addLayout(wp_btn_layout)

        main_layout.addWidget(workpiece_group)

        # Teaching panel (shown when teaching active)
        self.teaching_group = QGroupBox("Frame Teaching")
        teaching_layout = QVBoxLayout(self.teaching_group)

        # Teaching name input
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Frame Name:"))
        self.teaching_name_input = QLineEdit()
        self.teaching_name_input.setPlaceholderText("Enter frame name...")
        name_layout.addWidget(self.teaching_name_input)
        teaching_layout.addLayout(name_layout)

        # Teaching progress
        self.teaching_progress = QProgressBar()
        self.teaching_progress.setRange(0, 3)
        self.teaching_progress.setValue(0)
        teaching_layout.addWidget(self.teaching_progress)

        # Teaching message
        self.teaching_message = QLabel("Not teaching")
        self.teaching_message.setWordWrap(True)
        self.teaching_message.setStyleSheet("color: #666; padding: 5px;")
        teaching_layout.addWidget(self.teaching_message)

        # Teaching buttons
        teach_btn_layout = QHBoxLayout()
        self.btn_start_teaching = QPushButton("Start Teaching")
        self.btn_start_teaching.clicked.connect(self._start_teaching)
        teach_btn_layout.addWidget(self.btn_start_teaching)

        self.btn_record_point = QPushButton("Record Point")
        self.btn_record_point.setEnabled(False)
        self.btn_record_point.clicked.connect(self._record_teaching_point)
        teach_btn_layout.addWidget(self.btn_record_point)

        self.btn_finish_teaching = QPushButton("Finish")
        self.btn_finish_teaching.setEnabled(False)
        self.btn_finish_teaching.clicked.connect(self._finish_teaching)
        teach_btn_layout.addWidget(self.btn_finish_teaching)

        self.btn_cancel_teaching = QPushButton("Cancel")
        self.btn_cancel_teaching.setEnabled(False)
        self.btn_cancel_teaching.clicked.connect(self._cancel_teaching)
        teach_btn_layout.addWidget(self.btn_cancel_teaching)

        teaching_layout.addLayout(teach_btn_layout)
        main_layout.addWidget(self.teaching_group)

        # Tool frames section
        tool_group = QGroupBox("Tool Frames")
        tool_layout = QVBoxLayout(tool_group)

        # Tool list
        self.tool_table = QTableWidget()
        self.tool_table.setColumnCount(4)
        self.tool_table.setHorizontalHeaderLabels(["Name", "Z Offset", "Description", ""])
        self.tool_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tool_table.setMaximumHeight(120)
        tool_layout.addWidget(self.tool_table)

        # Add tool section
        add_tool_layout = QHBoxLayout()
        add_tool_layout.addWidget(QLabel("Name:"))
        self.tool_name_input = QLineEdit()
        self.tool_name_input.setMaximumWidth(100)
        add_tool_layout.addWidget(self.tool_name_input)

        add_tool_layout.addWidget(QLabel("Z Offset:"))
        self.tool_z_spin = QDoubleSpinBox()
        self.tool_z_spin.setRange(-200, 200)
        self.tool_z_spin.setSuffix(" mm")
        self.tool_z_spin.setMaximumWidth(100)
        add_tool_layout.addWidget(self.tool_z_spin)

        self.btn_add_tool = QPushButton("Add Tool")
        self.btn_add_tool.clicked.connect(self._add_tool)
        add_tool_layout.addWidget(self.btn_add_tool)

        add_tool_layout.addStretch()
        tool_layout.addLayout(add_tool_layout)

        main_layout.addWidget(tool_group)

        # Base frame section
        base_group = QGroupBox("Base Frame (Robot Mounting)")
        base_layout = QGridLayout(base_group)

        # Position
        base_layout.addWidget(QLabel("Position:"), 0, 0)
        self.base_x_spin = QDoubleSpinBox()
        self.base_x_spin.setRange(-10000, 10000)
        self.base_x_spin.setSuffix(" mm")
        base_layout.addWidget(QLabel("X:"), 0, 1)
        base_layout.addWidget(self.base_x_spin, 0, 2)

        self.base_y_spin = QDoubleSpinBox()
        self.base_y_spin.setRange(-10000, 10000)
        self.base_y_spin.setSuffix(" mm")
        base_layout.addWidget(QLabel("Y:"), 0, 3)
        base_layout.addWidget(self.base_y_spin, 0, 4)

        self.base_z_spin = QDoubleSpinBox()
        self.base_z_spin.setRange(-10000, 10000)
        self.base_z_spin.setSuffix(" mm")
        base_layout.addWidget(QLabel("Z:"), 0, 5)
        base_layout.addWidget(self.base_z_spin, 0, 6)

        # Orientation
        base_layout.addWidget(QLabel("Orientation:"), 1, 0)
        self.base_roll_spin = QDoubleSpinBox()
        self.base_roll_spin.setRange(-180, 180)
        self.base_roll_spin.setSuffix(" deg")
        base_layout.addWidget(QLabel("Roll:"), 1, 1)
        base_layout.addWidget(self.base_roll_spin, 1, 2)

        self.base_pitch_spin = QDoubleSpinBox()
        self.base_pitch_spin.setRange(-180, 180)
        self.base_pitch_spin.setSuffix(" deg")
        base_layout.addWidget(QLabel("Pitch:"), 1, 3)
        base_layout.addWidget(self.base_pitch_spin, 1, 4)

        self.base_yaw_spin = QDoubleSpinBox()
        self.base_yaw_spin.setRange(-180, 180)
        self.base_yaw_spin.setSuffix(" deg")
        base_layout.addWidget(QLabel("Yaw:"), 1, 5)
        base_layout.addWidget(self.base_yaw_spin, 1, 6)

        # Apply button
        self.btn_apply_base = QPushButton("Apply Base Frame")
        self.btn_apply_base.clicked.connect(self._apply_base_frame)
        base_layout.addWidget(self.btn_apply_base, 2, 0, 1, 7)

        main_layout.addWidget(base_group)

        main_layout.addStretch()

    def _refresh_all_lists(self):
        """Refresh all frame lists from controller"""
        if not self.frame_controller:
            return

        # Update frame combo
        frames = self.frame_controller.get_selectable_frames()
        self.frame_combo.blockSignals(True)
        self.frame_combo.clear()
        self.frame_combo.addItems(frames)
        self.frame_combo.setCurrentText(self.frame_controller.get_active_frame())
        self.frame_combo.blockSignals(False)

        # Update tool combo
        tools = self.frame_controller.get_tools()
        self.tool_combo.blockSignals(True)
        self.tool_combo.clear()
        self.tool_combo.addItems(tools)
        self.tool_combo.setCurrentText(self.frame_controller.get_active_tool())
        self.tool_combo.blockSignals(False)

        # Update tables
        self._update_workpiece_table()
        self._update_tool_table()

    def _update_workpiece_table(self):
        """Update workpiece frames table"""
        if not self.frame_controller:
            return

        workpieces = self.frame_controller.get_workpieces()
        self.workpiece_table.setRowCount(len(workpieces))

        for row, name in enumerate(workpieces):
            info = self.frame_controller.get_frame_info(name)
            if info:
                self.workpiece_table.setItem(row, 0, QTableWidgetItem(name))
                pos = info['position']
                self.workpiece_table.setItem(row, 1, QTableWidgetItem(f"{pos[0]:.1f}"))
                self.workpiece_table.setItem(row, 2, QTableWidgetItem(f"{pos[1]:.1f}"))
                self.workpiece_table.setItem(row, 3, QTableWidgetItem(f"{pos[2]:.1f}"))

    def _update_tool_table(self):
        """Update tool frames table"""
        if not self.frame_controller:
            return

        tools = self.frame_controller.get_tools()
        self.tool_table.setRowCount(len(tools))

        for row, name in enumerate(tools):
            info = self.frame_controller.get_frame_info(name)
            if info:
                self.tool_table.setItem(row, 0, QTableWidgetItem(name))
                pos = info['position']
                self.tool_table.setItem(row, 1, QTableWidgetItem(f"{pos[2]:.1f}"))
                self.tool_table.setItem(row, 2, QTableWidgetItem(info.get('description', '')))

                # Delete button
                if name != "default_tool":
                    btn = QPushButton("Delete")
                    btn.clicked.connect(lambda checked, n=name: self._delete_tool(n))
                    self.tool_table.setCellWidget(row, 3, btn)

    def _on_frames_updated(self, frames: List[str]):
        """Callback when frames list changes"""
        self.frame_combo.blockSignals(True)
        current = self.frame_combo.currentText()
        self.frame_combo.clear()
        self.frame_combo.addItems(frames)
        if current in frames:
            self.frame_combo.setCurrentText(current)
        self.frame_combo.blockSignals(False)

    def _on_tools_updated(self, tools: List[str]):
        """Callback when tools list changes"""
        self.tool_combo.blockSignals(True)
        current = self.tool_combo.currentText()
        self.tool_combo.clear()
        self.tool_combo.addItems(tools)
        if current in tools:
            self.tool_combo.setCurrentText(current)
        self.tool_combo.blockSignals(False)
        self._update_tool_table()

    def _on_workpieces_updated(self, workpieces: List[str]):
        """Callback when workpieces list changes"""
        self._update_workpiece_table()
        # Also refresh frame combo since workpieces are selectable
        if self.frame_controller:
            frames = self.frame_controller.get_selectable_frames()
            self._on_frames_updated(frames)

    def _on_teaching_progress(self, progress: TeachingProgress):
        """Callback for teaching progress updates"""
        self.teaching_progress.setValue(progress.points_recorded)
        self.teaching_message.setText(progress.message)

        is_teaching = progress.is_teaching
        self.btn_start_teaching.setEnabled(not is_teaching)
        self.teaching_name_input.setEnabled(not is_teaching)
        self.btn_record_point.setEnabled(is_teaching)
        self.btn_cancel_teaching.setEnabled(is_teaching)
        self.btn_finish_teaching.setEnabled(progress.state == TeachingState.COMPLETE)

        # Update message style based on state
        if progress.state == TeachingState.ERROR:
            self.teaching_message.setStyleSheet("color: red; padding: 5px; font-weight: bold;")
        elif progress.state == TeachingState.COMPLETE:
            self.teaching_message.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        else:
            self.teaching_message.setStyleSheet("color: #666; padding: 5px;")

    def _on_frame_selected(self, frame_name: str):
        """Handle frame selection change"""
        if self.frame_controller and frame_name:
            self.frame_controller.select_frame(frame_name)

    def _on_tool_selected(self, tool_name: str):
        """Handle tool selection change"""
        if self.frame_controller and tool_name:
            self.frame_controller.select_tool(tool_name)

    def _start_workpiece_teaching(self):
        """Start teaching a new workpiece frame"""
        self.teaching_name_input.setFocus()
        self.teaching_name_input.clear()

    def _start_teaching(self):
        """Start the teaching process"""
        name = self.teaching_name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Please enter a frame name")
            return

        if self.frame_controller:
            self.frame_controller.start_teaching_workpiece(name)

    def _record_teaching_point(self):
        """Record current TCP position"""
        if self.frame_controller:
            self.frame_controller.record_teaching_point()

    def _finish_teaching(self):
        """Finish teaching and create frame"""
        if self.frame_controller:
            frame = self.frame_controller.finish_teaching()
            if frame:
                QMessageBox.information(
                    self, "Success",
                    f"Created workpiece frame: {frame.name}"
                )

    def _cancel_teaching(self):
        """Cancel teaching process"""
        if self.frame_controller:
            self.frame_controller.cancel_teaching()

    def _delete_selected_workpiece(self):
        """Delete selected workpiece frame"""
        row = self.workpiece_table.currentRow()
        if row < 0:
            QMessageBox.warning(self, "Error", "Please select a workpiece to delete")
            return

        name_item = self.workpiece_table.item(row, 0)
        if name_item and self.frame_controller:
            name = name_item.text()
            reply = QMessageBox.question(
                self, "Confirm Delete",
                f"Delete workpiece frame '{name}'?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.frame_controller.delete_frame(name)

    def _add_tool(self):
        """Add new tool frame"""
        name = self.tool_name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Please enter a tool name")
            return

        z_offset = self.tool_z_spin.value()

        if self.frame_controller:
            success = self.frame_controller.create_tool_frame(
                name, offset_z=z_offset
            )
            if success:
                self.tool_name_input.clear()
                self.tool_z_spin.setValue(0)
            else:
                QMessageBox.warning(self, "Error", f"Failed to create tool '{name}'")

    def _delete_tool(self, name: str):
        """Delete a tool frame"""
        if self.frame_controller:
            reply = QMessageBox.question(
                self, "Confirm Delete",
                f"Delete tool '{name}'?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.frame_controller.delete_frame(name)

    def _apply_base_frame(self):
        """Apply base frame settings"""
        if not self.frame_controller:
            return

        self.frame_controller.update_base_frame(
            x=self.base_x_spin.value(),
            y=self.base_y_spin.value(),
            z=self.base_z_spin.value(),
            roll=self.base_roll_spin.value(),
            pitch=self.base_pitch_spin.value(),
            yaw=self.base_yaw_spin.value()
        )
        QMessageBox.information(self, "Success", "Base frame updated")


if __name__ == "__main__":
    # Test the panel
    import sys
    from PyQt5.QtWidgets import QApplication

    logging.basicConfig(level=logging.DEBUG)

    app = QApplication(sys.argv)

    # Create controller
    from frame_controller import FrameController
    controller = FrameController()

    # Create panel
    panel = FrameManagementPanel()
    panel.set_controller(controller)
    panel.setWindowTitle("Frame Management Panel Test")
    panel.resize(600, 700)
    panel.show()

    sys.exit(app.exec_())
