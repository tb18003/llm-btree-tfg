import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MonitorNode(Node):

    def __init__(self):
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

        self.task_info_sub = self.create_subscription(
            String,
            task_info_param.value,
            self.whisper_callback,
            10
        )
    
    def check_systems(self):
        return rclpy.ok()

    def whisper_callback(self, msg):
        print(msg.data)

    def task_info_callback(self, msg):
        print(msg.data)

