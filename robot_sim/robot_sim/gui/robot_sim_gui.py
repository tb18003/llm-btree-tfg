from PyQt5.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout, 
                             QTextEdit, QGroupBox, QPlainTextEdit, QPushButton)
from PyQt5.QtCore import Qt
import datetime

class RobotLoggerSimGUI(QWidget):

    def __init__(self):
        super().__init__()
        self._init_components()

    def set_ros_node(self, node):
        self.ros_node = node
        self._topic_name_field.setPlainText(self.ros_node.whisper_pub.topic_name)

    def _init_components(self):
        # Screen Layout
        sc_layout = QVBoxLayout(self)

        # Window Label
        title_label = QLabel()
        title_label.setText("Robot Simulator")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        sc_layout.addWidget(title_label)
        sc_layout.addWidget(self._whisper_sender_widget())
        sc_layout.addWidget(self._logger_widget())

        self.setWindowTitle("Robot Simulator - v0.1")
    
    def _logger_widget(self):
        # Logger Container
        container = QGroupBox()
        container.setTitle("Robot Log Output")
        
        # Container layout
        b_layout = QHBoxLayout(container)
        
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setMinimumHeight(200)
        
        b_layout.addWidget(self.log_area)

        return container
    
    def _whisper_sender_widget(self):
        # Whisper Container
        container = QGroupBox()
        container.setTitle("Robot Whisper Simulator")

        b_layout = QVBoxLayout(container)

        # Topic Information
        topic_info = QWidget()
        topic_layout = QHBoxLayout(topic_info)

        topic_label = QLabel()
        topic_label.setText("Topic name")
        topic_label.setMinimumWidth(60)

        self._topic_name_field = QPlainTextEdit()
        self._topic_name_field.setPlainText("Loading...")
        self._topic_name_field.setMaximumHeight(30)
        self._topic_name_field.setEnabled(False)
        self._topic_name_field.setToolTip("To change this value, you have to declare ros2 arg 'WHISPER_TOPIC' in command line")

        topic_layout.addWidget(topic_label)
        topic_layout.addWidget(self._topic_name_field)

        # Topic Text Field
        topic_sender = QWidget()
        topic_sender_layout = QHBoxLayout(topic_sender)

        topic_sender_label = QLabel()
        topic_sender_label.setText("Whisper order")
        topic_sender_label.setMinimumWidth(60)

        self._topic_field = QPlainTextEdit()
        self._topic_field.setPlaceholderText("Write an order to the robot")
        self._topic_field.setMaximumHeight(30)

        topic_sender_layout.addWidget(topic_sender_label)
        topic_sender_layout.addWidget(self._topic_field)

        # Send push button
        send_btn = QPushButton()
        send_btn.setText("Send")
        send_btn.clicked.connect(self._button_callback)

        b_layout.addWidget(topic_info)
        b_layout.addWidget(topic_sender)
        b_layout.addWidget(send_btn)

        return container

    def _button_callback(self):
        if len(self._topic_field.toPlainText().strip()) > 0:
            self.ros_node.whisper_message_sender(self._topic_field.toPlainText())
            self._topic_field.setPlainText("")


    def log(self, message: str, tag: str = "info"):
        color = {
            "info": "#2e61b3",
            "error": "#c20e23",
            "warning": "#d19e1b",
            "debug": "#60b023"
        }
        self.log_area.append(f'<pre><span style="color: {color.get(tag, "#707070")}; font-size: 12px">{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")} <b>[{tag.upper()}] {message}</b></span></pre>')
