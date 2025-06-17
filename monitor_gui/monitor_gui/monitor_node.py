import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from .monitor_gui import MonitoringMainWindow, SplashScreen
import threading

from PyQt5.QtWidgets import QApplication, QMessageBox
from llm_bridge_interfaces.srv import LLMService # type: ignore

from std_msgs.msg import String

from json import JSONDecoder
from typing import Union
import time

class MonitorNode(Node):

    def __init__(self, view: MonitoringMainWindow):
        super().__init__('sancho_monitor_node')

        # Parameters
        whisper_topic_param = self.declare_parameter('WHISPER_TOPIC', '/robot/whisper')
        text_topic_param = self.declare_parameter('TEXT_TOPIC', '/robot/input')
        task_info_param = self.declare_parameter('TASK_INFO_TOPIC', '/task/info')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.text_pub = self.create_publisher(
            String,
            text_topic_param.value,
            10
        )

        self.whisper_sub = self.create_subscription(
            String,
            whisper_topic_param.value,
            self.whisper_callback,
            10
        )

        self.task_info_sub = self.create_subscription(
            String,
            task_info_param.value,
            self.task_info_callback,
            10
        )

        self.view = view

    def check_systems(self, splash) -> Union[bool,str]:
        time.sleep(0.1)  # Allow time for the splash screen to show
        components = 4
        product = (100 // components)
        done = 1

        splash.update_progress(done * product, 'Checking ROS2 System...')
        if not rclpy.ok():
            return (False, 'ROS2 is not running')

        topics = self.get_topic_names_and_types()

        done += 1

        # Checking LLM Service
        splash.update_progress(done * product, 'Waiting for LLM service...')
        c = self.create_client(
                LLMService,
                '/llm'
            )
        if not c.wait_for_service(timeout_sec=10.0):
            return (False, 'Timeout reached while waiting for LLM service')
        
        c.destroy()
        
        done += 1

        # Checking Task Info Topic
        splash.update_progress(done * product, 'Checking topics...')
        if '/task/info' not in [name for name, _ in topics]:
            return (False, 'Cannot find activity for task information topic')
        
        if '/task/input' not in [name for name, _ in topics]:
            return (False, 'Cannot find activity for task input topic')

        done += 1

        # Checking Navigation Action
        splash.update_progress(done * product, 'Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            return (False, 'Timeout reached while waiting for navigation action server')

        return (True, None)
    
    def send_whisper(self, msg: str):
        m = String(data=msg)

        if self.text_pub is not None:
            self.text_pub.publish(m)
        else:
            self.get_logger().error("Text publisher is not initialized, cannot send message")

    def whisper_callback(self, msg):
        self.view.order_update_signal.emit(msg.data)

    def task_info_callback(self, msg):
        self.view.tasks_update_signal.emit(JSONDecoder().decode(msg.data))
        


def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])

    splash = SplashScreen()
    
    gui = MonitoringMainWindow()
    node = MonitorNode(gui)
    gui.set_node(node)

    app.setWindowIcon(gui.windowIcon())
    app.setApplicationName('Sancho Monitor')
    app.setApplicationVersion('0.1.0')
    app.setApplicationDisplayName('Tasks Monitor')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # GUI Init
    splash.show()

    ok, msg = node.check_systems(splash)
    if not ok:
        QMessageBox.critical(
            None,
            'Error',
            'Cannot start the application. Error cause: %s' % msg
        )
        splash.finish(gui)
        gui.close()
        executor.shutdown()
        return

    splash.finish(gui)
    gui.show()

    node.get_logger().info('Monitor is ready!')
    node.get_logger().info('Whisper Topic: %s' % node.whisper_sub.topic_name)
    node.get_logger().info('Text Topic: %s' % node.text_pub.topic_name)
    node.get_logger().info('Task Topic: %s' % node.task_info_sub.topic_name)

    c = app.exec_()

    executor.shutdown()




    

    