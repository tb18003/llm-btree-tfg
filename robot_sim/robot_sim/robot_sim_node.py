import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point32
from PyQt5.QtWidgets import QApplication
from threading import Thread
from .gui.robot_sim_gui import RobotLoggerSimGUI


class RobotSimNode(Node):

    def __init__(self, view):
        super().__init__("robot_sim_node")

        self.declare_parameter("MOVE_TOPIC", "/robot/move")
        self.declare_parameter("TTS_TOPIC", "/robot/tts")
        self.declare_parameter("WHISPER_TOPIC", "/robot/whisper")

        self.view = view

        self.move_sub = self.create_subscription(Point32, self.get_parameter("MOVE_TOPIC").value, self.move_subscription_callback,10)
        self.tts_sub = self.create_subscription(String, self.get_parameter("TTS_TOPIC").value, self.tts_subscription_callback, 10)
        self.whisper_pub = self.create_publisher(String, self.get_parameter("WHISPER_TOPIC").value, 10)

        self.whisper_pub.topic_name

        view.log("ROS2 System Started.","system")

    def move_subscription_callback(self, message: Point32):
        self.view.log(f"Moving robot to position (X = {round(message.x, 3)}, Y = {round(message.y, 3)}, Z = {round(message.z, 3)})")
    
    def tts_subscription_callback(self, message: String):
        self.view.log(f"Robot said '{message.data}'")
    
    def whisper_message_sender(self, message: str):
        self.whisper_pub.publish(String(data=message))

def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])
    gui = RobotLoggerSimGUI()

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