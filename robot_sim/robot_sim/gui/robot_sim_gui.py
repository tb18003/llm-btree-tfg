from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QTextEdit, QGroupBox, QDial, QPushButton
from PyQt5.QtCore import Qt
import datetime

class RobotLoggerSimGUI(QWidget):

    def __init__(self):
        super().__init__()
        self._init_components()

    def set_ros_node(self, node):
        self.ros_node = node

    def _init_components(self):
        # Screen Layout
        sc_layout = QVBoxLayout(self)

        # Window Label
        title_label = QLabel()
        title_label.setText("Robot Logger Simulator")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        bw_widget = QWidget()
        bw_layout = QHBoxLayout(bw_widget)


        sc_layout.addWidget(title_label)
        sc_layout.addWidget(bw_widget)
        sc_layout.addWidget(self._logger_widget())

        self.setWindowTitle("Robot Logger Simulator")
    
    def _logger_widget(self):
        # Logger Container
        container = QGroupBox()
        container.setTitle("Robot Log")
        
        # Container layout
        b_layout = QHBoxLayout(container)
        
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setMinimumHeight(200)
        
        b_layout.addWidget(self.log_area)

        return container

    def log(self, message: str, tag: str = "info"):
        color = {
            "info": "#2e61b3",
            "error": "#c20e23",
            "warning": "#d19e1b",
            "debug": "#60b023"
        }
        self.log_area.append(f'<pre><span style="color: {color.get(tag, "#707070")}; font-size: 12px">{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")} <b>[{tag.upper()}] {message}</b></span></pre>')
