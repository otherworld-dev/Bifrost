"""
Position Visualization for Thor Robot Arm
Real-time plotting of joint positions using matplotlib
"""

import matplotlib
matplotlib.use('Qt5Agg')  # Use Qt5 backend for PyQt5 integration

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtCore
import numpy as np
import logging
from datetime import datetime

logger = logging.getLogger(__name__)


class PositionPlotCanvas(FigureCanvas):
    """
    Matplotlib canvas for embedding in PyQt5
    """

    def __init__(self, parent=None, width=8, height=6, dpi=100):
        """
        Args:
            parent: Parent Qt widget
            width: Figure width in inches
            height: Figure height in inches
            dpi: Dots per inch for the figure
        """
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.setParent(parent)

        # Adjust layout
        self.fig.tight_layout(pad=2.0)

        # Joint colours for consistent visualisation
        self.joint_colors = {
            'art1': '#FF6B6B',  # Red
            'art2': '#4ECDC4',  # Cyan
            'art3': '#45B7D1',  # Blue
            'art4': '#FFA07A',  # Light orange
            'art5': '#98D8C8',  # Mint
            'art6': '#F7DC6F',  # Yellow
        }

        self.axes = None
        self.lines = {}
        self.joint_names = []

        logger.info("Position plot canvas initialised")

    def setup_plot(self, joint_names):
        """
        Setup the plot with axes for given joints

        Args:
            joint_names: List of joint names to plot
        """
        self.joint_names = joint_names
        self.axes = self.fig.add_subplot(111)

        self.axes.set_xlabel('Time (seconds)')
        self.axes.set_ylabel('Position (degrees)')
        self.axes.set_title('Joint Position History')
        self.axes.grid(True, alpha=0.3)

        # Create line for each joint
        self.lines = {}
        for joint in joint_names:
            color = self.joint_colors.get(joint, '#000000')
            line, = self.axes.plot([], [], label=joint.upper(),
                                   color=color, linewidth=2, marker='o',
                                   markersize=3, alpha=0.8)
            self.lines[joint] = line

        self.axes.legend(loc='upper right')
        self.draw()

        logger.info(f"Plot setup complete for joints: {joint_names}")

    def update_plot(self, position_history, window_size=100, visible_joints=None):
        """
        Update the plot with new data from position history

        Args:
            position_history: PositionHistory object
            window_size: Number of recent points to display (0 = all)
            visible_joints: List of joint names to display (None = all joints)
        """
        if len(position_history) == 0:
            return

        # Get data for all joints
        all_data = position_history.get_all_joints_data()

        if len(all_data) == 0:
            return

        # If visible_joints not specified, show all
        if visible_joints is None:
            visible_joints = list(all_data.keys())

        # Setup plot if not already done or if joints changed
        if self.axes is None or set(self.joint_names) != set(visible_joints):
            self.fig.clear()
            self.setup_plot(visible_joints)

        # Get reference time (first timestamp)
        first_timestamp = None
        for timestamps, _ in all_data.values():
            if len(timestamps) > 0:
                first_timestamp = timestamps[0]
                break

        if first_timestamp is None:
            return

        # Update each line
        for joint in self.joint_names:
            if joint in all_data:
                timestamps, positions = all_data[joint]

                # Apply window size (window_size is in seconds now)
                if window_size > 0:
                    # Filter by time window
                    current_time = timestamps[-1] if len(timestamps) > 0 else first_timestamp
                    cutoff_time = current_time - window_size
                    filtered_data = [(t, p) for t, p in zip(timestamps, positions) if t >= cutoff_time]
                    if filtered_data:
                        timestamps, positions = zip(*filtered_data)
                    else:
                        timestamps, positions = [], []

                # Convert timestamps to relative seconds
                time_seconds = [t - first_timestamp for t in timestamps]

                # Update line data
                if joint in self.lines:
                    self.lines[joint].set_data(time_seconds, positions)

        # Adjust axes limits
        self.axes.relim()
        self.axes.autoscale_view()

        # Redraw
        self.draw()

    def clear_plot(self):
        """Clear all data from the plot"""
        for line in self.lines.values():
            line.set_data([], [])
        self.draw()

    def save_plot(self, filepath):
        """Save the current plot to a file"""
        try:
            self.fig.savefig(filepath, dpi=150, bbox_inches='tight')
            logger.info(f"Plot saved to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to save plot: {e}")
            return False


class PositionVisualizerWindow(QtWidgets.QDialog):
    """
    Standalone window for position visualization
    """

    def __init__(self, position_history, parent=None):
        super().__init__(parent)
        self.position_history = position_history
        self.timer = None
        self.update_interval = 1000  # milliseconds

        self.setWindowTitle("Position History Visualizer")
        self.setGeometry(100, 100, 1000, 700)

        self.setupUI()
        self.startUpdates()

    def setupUI(self):
        """Setup the UI layout"""
        layout = QtWidgets.QVBoxLayout(self)

        # Canvas for plotting
        self.canvas = PositionPlotCanvas(self, width=10, height=7, dpi=100)
        self.canvas.setup_plot(['art1', 'art2', 'art3', 'art4', 'art5', 'art6'])

        # Control panel
        control_layout = QtWidgets.QHBoxLayout()

        # Window size control
        control_layout.addWidget(QtWidgets.QLabel("Display Points:"))
        self.window_size_spin = QtWidgets.QSpinBox()
        self.window_size_spin.setRange(10, 1000)
        self.window_size_spin.setValue(100)
        self.window_size_spin.setSuffix(" pts")
        control_layout.addWidget(self.window_size_spin)

        # Update interval control
        control_layout.addWidget(QtWidgets.QLabel("Update Rate:"))
        self.update_rate_spin = QtWidgets.QSpinBox()
        self.update_rate_spin.setRange(100, 5000)
        self.update_rate_spin.setValue(1000)
        self.update_rate_spin.setSingleStep(100)
        self.update_rate_spin.setSuffix(" ms")
        self.update_rate_spin.valueChanged.connect(self.updateIntervalChanged)
        control_layout.addWidget(self.update_rate_spin)

        control_layout.addStretch()

        # Buttons
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.togglePause)
        control_layout.addWidget(self.pause_button)

        self.clear_button = QtWidgets.QPushButton("Clear History")
        self.clear_button.clicked.connect(self.clearHistory)
        control_layout.addWidget(self.clear_button)

        self.save_button = QtWidgets.QPushButton("Save Plot")
        self.save_button.clicked.connect(self.savePlot)
        control_layout.addWidget(self.save_button)

        self.export_button = QtWidgets.QPushButton("Export CSV")
        self.export_button.clicked.connect(self.exportCSV)
        control_layout.addWidget(self.export_button)

        # Info label
        self.info_label = QtWidgets.QLabel()
        self.updateInfoLabel()

        # Add to main layout
        layout.addWidget(self.canvas)
        layout.addLayout(control_layout)
        layout.addWidget(self.info_label)

    def startUpdates(self):
        """Start the timer for automatic updates"""
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start(self.update_interval)
        logger.info("Position visualizer updates started")

    def updatePlot(self):
        """Update the plot with latest data"""
        window_size = self.window_size_spin.value()
        self.canvas.update_plot(self.position_history, window_size)
        self.updateInfoLabel()

    def updateInfoLabel(self):
        """Update the information label"""
        total_points = len(self.position_history)
        duration = 0
        if total_points > 0:
            recent = self.position_history.get_recent(1)
            if len(recent) > 0:
                duration = recent[0].timestamp - self.position_history.start_time

        self.info_label.setText(
            f"Total Points: {total_points} | "
            f"Duration: {duration:.1f}s | "
            f"Recording: {'ON' if self.position_history.recording else 'OFF'}"
        )

    def updateIntervalChanged(self, value):
        """Handle update interval change"""
        self.update_interval = value
        if self.timer and self.timer.isActive():
            self.timer.setInterval(value)
            logger.info(f"Update interval changed to {value}ms")

    def togglePause(self):
        """Toggle pause/resume"""
        if self.timer.isActive():
            self.timer.stop()
            self.pause_button.setText("Resume")
            logger.info("Position visualizer paused")
        else:
            self.timer.start(self.update_interval)
            self.pause_button.setText("Pause")
            logger.info("Position visualizer resumed")

    def clearHistory(self):
        """Clear position history"""
        reply = QtWidgets.QMessageBox.question(
            self,
            'Clear History',
            'Are you sure you want to clear all position history?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )

        if reply == QtWidgets.QMessageBox.Yes:
            self.position_history.clear()
            self.canvas.clear_plot()
            self.updateInfoLabel()
            logger.info("Position history cleared from visualizer")

    def savePlot(self):
        """Save the current plot to file"""
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Plot",
            f"position_plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
            "PNG Files (*.png);;PDF Files (*.pdf);;All Files (*)"
        )

        if filename:
            if self.canvas.save_plot(filename):
                QtWidgets.QMessageBox.information(
                    self, "Success", f"Plot saved to:\n{filename}"
                )

    def exportCSV(self):
        """Export position history to CSV"""
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export CSV",
            f"position_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            if self.position_history.export_to_csv(filename):
                QtWidgets.QMessageBox.information(
                    self, "Success", f"Data exported to:\n{filename}"
                )
            else:
                QtWidgets.QMessageBox.critical(
                    self, "Error", "Failed to export data"
                )

    def closeEvent(self, event):
        """Handle window close"""
        if self.timer:
            self.timer.stop()
        event.accept()


if __name__ == "__main__":
    # Test the visualizer
    import sys
    from position_history import PositionHistory
    import time
    import math

    logging.basicConfig(level=logging.INFO, stream=sys.stdout)

    # Create test data
    history = PositionHistory(max_size=500)

    # Generate sine wave test data
    for i in range(200):
        t = i * 0.1
        history.add_snapshot(
            art1=20 * math.sin(t),
            art2=15 * math.cos(t),
            art3=10 * math.sin(2 * t),
            art4=5 * math.cos(2 * t),
            art5=8 * math.sin(t * 0.5),
            art6=6 * math.cos(t * 0.5)
        )

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create and show visualizer window
    window = PositionVisualizerWindow(history)
    window.show()

    sys.exit(app.exec_())
