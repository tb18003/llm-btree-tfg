import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QDial
from PyQt5.QtCore import Qt
from threading import Thread

class RobotSimServerGUI(QWidget):

    def __init__(self):
        super().__init__()
        self.init_components()

    def set_ros_node(self, node):
        self.ros_node = node
        self.dial.setEnabled(True)

    def init_components(self):
        # Screen Layout
        sc_layout = QVBoxLayout(self)

        # Window Label
        title_label = QLabel()
        title_label.setText("Robot Simulation Controller")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        sc_layout.addWidget(title_label)
        sc_layout.addWidget(self.battery_view())

        self.setWindowTitle("Robot Simulation Controller (Server Side)")

    def on_dial_change(self, value):
        self.bt_label.setText("%d %" % value)
        self.ros_node.publish_battery(value)

    def battery_view(self):
        # Battery Container
        container = QGroupBox()
        container.setWindowTitle("Battery Simulation")
        
        # Container layout
        b_layout = QHBoxLayout(container)
        
        self.dial = QDial()
        self.dial.setRange(0,100)
        self.dial.setValue(100)
        self.dial.valueChanged.connect(self.on_dial_change)
        self.dial.setEnabled(False)

        self.bt_label = QLabel()
        self.bt_label.setText("100 %")
        self.bt_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)

        b_layout.addWidget(self.dial)
        b_layout.addWidget(self.bt_label)

        return container


class RobotSimNode(Node):

    def __init__(self, view):
        super().__init__("robot_sim_node")

        self.declare_parameter("MOVE_TOPIC", "/not/found")
        self.declare_parameter("WHISPER_TOPIC", "/not/found")
        self.declare_parameter("BATTERY_TOPIC", "/not/found")

        self.publisher = self.create_publisher(String, "/robot_sim/test", 10)
        self.battery_pub = self.create_publisher(Int32, self.get_parameter("BATTERY_TOPIC").value, 10)
    
    def publish_action(self):
        m = String()
        m.data = self.get_parameter("MOVE_TOPIC").value
        self.publisher.publish(m)
    
    def publish_battery(self, b_level):
        m = Int32()
        m.data = b_level
        self.battery_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])
    gui = RobotSimServerGUI()

    node = RobotSimNode(gui)

    gui.set_ros_node(node)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

    gui.show()
    code = app.exec_()

    node.destroy_node()

    executor.shutdown()

    return code

if __name__ == "__main__":
    main()