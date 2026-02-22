"""
DH Parameters Panel
====================

Provides a UI panel for viewing and editing DH (Denavit-Hartenberg) parameters
for the Thor robot arm.
"""

import json
import logging
import paths
from PyQt5 import QtWidgets, QtCore, QtGui

logger = logging.getLogger(__name__)

DH_PARAMS_FILE = paths.get_data_dir() / 'dh_parameters.json'


class DHParametersPanel(QtWidgets.QWidget):
    """Panel for viewing and editing DH parameters"""

    parameters_changed = QtCore.pyqtSignal()  # Emitted when parameters are saved
    preview_changed = QtCore.pyqtSignal()  # Emitted when any value changes (for live preview)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.dh_params = None
        self.spinboxes = {}  # Store references to spinboxes for each parameter
        self._loading = False  # Flag to prevent signals during load
        self.setup_ui()
        self.load_parameters()

    def setup_ui(self):
        """Setup the UI layout"""
        self.setStyleSheet("DHParametersPanel { background-color: #f5f5f5; }")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title
        title = QtWidgets.QLabel("DH Parameters (Editable)")
        title_font = QtGui.QFont()
        title_font.setPointSize(11)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

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
        self.table.setColumnWidth(0, 50)

        # Style the table to match main app
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                alternate-background-color: #f9f9f9;
                color: #333333;
                gridline-color: #ddd;
                border: 1px solid #ccc;
            }
            QHeaderView::section {
                background-color: #e8e8e8;
                color: #333333;
                padding: 5px;
                border: 1px solid #ccc;
                font-weight: bold;
            }
            QTableWidget::item {
                padding: 5px;
            }
        """)

        # Create spinboxes for each cell
        for row in range(6):
            # Link number (read-only)
            link_item = QtWidgets.QTableWidgetItem(str(row + 1))
            link_item.setFlags(QtCore.Qt.ItemIsEnabled)
            link_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.table.setItem(row, 0, link_item)

            # theta_offset spinbox (column 1)
            spinbox = QtWidgets.QDoubleSpinBox()
            spinbox.setRange(-360, 360)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setAlignment(QtCore.Qt.AlignCenter)
            spinbox.setStyleSheet("""
                QDoubleSpinBox {
                    background-color: #ffffff;
                    color: #333333;
                    border: 1px solid #ccc;
                    padding: 2px;
                }
                QDoubleSpinBox:focus {
                    border: 1px solid #4CAF50;
                }
            """)
            spinbox.valueChanged.connect(self._on_value_changed)
            self.table.setCellWidget(row, 1, spinbox)
            self.spinboxes[(row, 'theta_offset')] = spinbox

            # d, a, alpha spinboxes (columns 2, 3, 4)
            for col, param in enumerate(['d', 'a', 'alpha'], start=2):
                spinbox = QtWidgets.QDoubleSpinBox()
                spinbox.setRange(-360, 360)
                spinbox.setDecimals(2)
                spinbox.setSingleStep(1.0)
                spinbox.setAlignment(QtCore.Qt.AlignCenter)
                spinbox.setStyleSheet("""
                    QDoubleSpinBox {
                        background-color: #ffffff;
                        color: #333333;
                        border: 1px solid #ccc;
                        padding: 2px;
                    }
                    QDoubleSpinBox:focus {
                        border: 1px solid #4CAF50;
                    }
                """)

                # Set appropriate range for d and a (lengths)
                if param in ['d', 'a']:
                    spinbox.setRange(-1000, 1000)

                spinbox.valueChanged.connect(self._on_value_changed)
                self.table.setCellWidget(row, col, spinbox)
                self.spinboxes[(row, param)] = spinbox

        layout.addWidget(self.table)

        # Buttons
        button_layout = QtWidgets.QHBoxLayout()

        self.load_button = QtWidgets.QPushButton("📂 Load")
        self.load_button.setToolTip("Reload parameters from file")
        self.load_button.setMinimumHeight(35)
        self.load_button.clicked.connect(self.load_parameters)
        self.load_button.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                color: #333;
                border: 1px solid #ccc;
                border-radius: 3px;
                padding: 5px 15px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        button_layout.addWidget(self.load_button)

        self.save_button = QtWidgets.QPushButton("💾 Save")
        self.save_button.setToolTip("Save parameters to file")
        self.save_button.setMinimumHeight(35)
        self.save_button.clicked.connect(self.save_parameters)
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                border: 2px solid #45a049;
                border-radius: 3px;
                padding: 5px 15px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        button_layout.addWidget(self.save_button)

        self.reset_button = QtWidgets.QPushButton("🔄 Reset to Default")
        self.reset_button.setToolTip("Reset to default Thor parameters")
        self.reset_button.setMinimumHeight(35)
        self.reset_button.clicked.connect(self.reset_to_default)
        self.reset_button.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                color: #333;
                border: 1px solid #ccc;
                border-radius: 3px;
                padding: 5px 15px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        button_layout.addWidget(self.reset_button)

        layout.addLayout(button_layout)

        # Status label
        self.status_label = QtWidgets.QLabel("Status: Ready")
        self.status_label.setStyleSheet("padding: 5px; background-color: #e8e8e8; border: 1px solid #ccc; border-radius: 3px; color: #333;")
        layout.addWidget(self.status_label)

    def _on_value_changed(self):
        """Handle spinbox value change - emit preview signal"""
        if not self._loading:
            self.preview_changed.emit()

    def load_parameters(self):
        """Load DH parameters from file"""
        self._loading = True  # Prevent preview signals during load
        try:
            if DH_PARAMS_FILE.exists():
                with open(DH_PARAMS_FILE, 'r') as f:
                    self.dh_params = json.load(f)

                # Update UI
                for link_data in self.dh_params['links']:
                    row = link_data['link'] - 1
                    self.spinboxes[(row, 'theta_offset')].setValue(link_data['theta_offset'])
                    self.spinboxes[(row, 'd')].setValue(link_data['d'])
                    self.spinboxes[(row, 'a')].setValue(link_data['a'])
                    self.spinboxes[(row, 'alpha')].setValue(link_data['alpha'])

                self.status_label.setText("Status: Loaded from file")
                self.status_label.setStyleSheet("padding: 5px; background-color: #c8e6c9; border: 1px solid #a5d6a7; border-radius: 3px; color: #2e7d32;")
                logger.info("Loaded DH parameters from file")
            else:
                self.reset_to_default()

        except Exception as e:
            logger.error(f"Error loading DH parameters: {e}")
            self.status_label.setText(f"Status: Error - {e}")
            self.status_label.setStyleSheet("padding: 5px; background-color: #ffcdd2; border: 1px solid #ef9a9a; border-radius: 3px; color: #c62828;")
        finally:
            self._loading = False
            self.preview_changed.emit()  # Initial preview after load

    def save_parameters(self):
        """Save DH parameters to file"""
        try:
            # Build parameters from UI
            if self.dh_params is None:
                self.dh_params = {
                    "version": "1.1",
                    "description": "Thor Robot DH Parameters with calibration",
                    "links": []
                }

            self.dh_params['date_modified'] = QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd")
            self.dh_params['links'] = []

            descriptions = ["Base rotation", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Wrist yaw / TCP"]

            for row in range(6):
                link_data = {
                    "link": row + 1,
                    "theta_offset": self.spinboxes[(row, 'theta_offset')].value(),
                    "d": self.spinboxes[(row, 'd')].value(),
                    "a": self.spinboxes[(row, 'a')].value(),
                    "alpha": self.spinboxes[(row, 'alpha')].value(),
                    "description": descriptions[row]
                }
                self.dh_params['links'].append(link_data)

            # Save to file
            with open(DH_PARAMS_FILE, 'w') as f:
                json.dump(self.dh_params, f, indent=4)

            self.status_label.setText("Status: Saved successfully!")
            self.status_label.setStyleSheet("padding: 5px; background-color: #c8e6c9; border: 1px solid #a5d6a7; border-radius: 3px; color: #2e7d32;")
            logger.info("Saved DH parameters to file")

            # Emit signal to notify FK needs to reload
            self.parameters_changed.emit()

            QtWidgets.QMessageBox.information(
                self,
                "DH Parameters Saved",
                "DH parameters saved successfully!\n\n"
                "The visualization will be updated with the new parameters."
            )

        except Exception as e:
            logger.error(f"Error saving DH parameters: {e}")
            self.status_label.setText(f"Status: Error - {e}")
            self.status_label.setStyleSheet("padding: 5px; background-color: #ffcdd2; border: 1px solid #ef9a9a; border-radius: 3px; color: #c62828;")

    def reset_to_default(self):
        """Reset to default Thor DH parameters"""
        self._loading = True  # Prevent multiple signals during reset
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

            self.status_label.setText("Status: Reset to defaults")
            self.status_label.setStyleSheet("padding: 5px; background-color: #fff9c4; border: 1px solid #fff176; border-radius: 3px; color: #f57f17;")
            logger.info("Reset DH parameters to defaults")
        finally:
            self._loading = False
            self.preview_changed.emit()  # Single preview signal after reset

    def get_parameters(self):
        """Get current DH parameters as a list of dicts"""
        params = []
        for row in range(6):
            params.append({
                "link": row + 1,
                "theta_offset": self.spinboxes[(row, 'theta_offset')].value(),
                "d": self.spinboxes[(row, 'd')].value(),
                "a": self.spinboxes[(row, 'a')].value(),
                "alpha": self.spinboxes[(row, 'alpha')].value()
            })
        return params


def load_dh_parameters():
    """
    Load DH parameters from file for use by FK module

    Returns:
        list: List of dicts containing DH parameters for each link
    """
    try:
        if DH_PARAMS_FILE.exists():
            with open(DH_PARAMS_FILE, 'r') as f:
                data = json.load(f)
            return data['links']
        else:
            logger.warning("DH parameters file not found, using defaults")
            return None
    except Exception as e:
        logger.error(f"Error loading DH parameters: {e}")
        return None
