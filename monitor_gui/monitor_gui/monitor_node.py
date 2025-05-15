import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .monitor_gui import MonitoringMainWindow, SplashScreen
import threading

from PyQt5.QtWidgets import QApplication

from std_msgs.msg import String

from json import JSONDecoder

class MonitorNode(Node):

    def __init__(self, view: MonitoringMainWindow):
        super().__init__('sancho_monitor_node')

        # Parameters
        whisper_topic_param = self.declare_parameter('WHISPER_TOPIC', '/robot/whisper')
        task_info_param = self.declare_parameter('TASK_INFO_TOPIC', '/task/info')

        self.whisper_sub = self.create_subscription(
            String,
            whisper_topic_param.value,
            self.whisper_callback,
            10
        )

        self.whisper_pub = self.create_publisher(
            String,
            whisper_topic_param.value,
            10
        )

        self.task_info_sub = self.create_subscription(
            String,
            task_info_param.value,
            self.task_info_callback,
            10
        )

        self.view = view        
    
    def check_systems(self, splash):
        components = 2
        product = (100 // components)

        splash.update_progress(1 * product, 'Checking ROS2 System...')
        if not rclpy.ok():
            return False

        services = self.get_service_names_and_types()
        topics = self.get_topic_names_and_types()

        # Checking LLM Service
        splash.update_progress(2 * product, 'Checking services...')
        if 'llm' not in [name for name, _ in services]:
            return False

        return True

    def whisper_callback(self, msg):
        pass

    def task_info_callback(self, msg):
        self.view.tasks_update_signal.emit(JSONDecoder().decode(msg.data))
        


def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])

    splash = SplashScreen()
    
    gui = MonitoringMainWindow()
    node = MonitorNode(gui)
    gui.set_node(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # GUI Init
    
    splash.show()
    node.check_systems(splash)
    splash.finish(gui)
    gui.show()

    c = app.exec_()

    executor.shutdown()


    

    