import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import String
from llm_bridge_interfaces.srv import LLMService

import uuid

class LLMNode(Node):
    """
    This node listen to the whisper topic (which return what human said), and sends it to the LLM Service. The response
    from the service is sent to the task input topic with a JSON format.
    """

    def __init__(self):
        super().__init__("llm_node")

        whisper_topic_param = self.declare_parameter("WHISPER_TOPIC", "/robot/whisper")
        task_topic_param = self.declare_parameter("TASKS_TOPIC", "/task/input")
        llm_topic_param = self.declare_parameter("LLM_SERVICE_TOPIC", "/llm")

        self.whisper_sub = self.create_subscription(String, whisper_topic_param.value, self.whisper_callback,10)
        self.task_pub = self.create_publisher(String, task_topic_param.value, 10)

        self.llm_client = self.create_client(LLMService, llm_topic_param.value)

        self.get_logger().info("Waiting for service...")

        while not self.llm_client.wait_for_service(timeout_sec=1):
            pass

        self.get_logger().info("Whisper to LLM Node ready!")
        self.get_logger().info(f"Whisper topic subscription: {whisper_topic_param.value}")
        self.get_logger().info(f"Task topic publisher: {task_topic_param.value}")
        self.get_logger().info(f"LLM topic service: {llm_topic_param.value}")
        self.working = False
    
    def whisper_callback(self, msg: String):
        req = LLMService.Request()
        req.prompt = msg.data
        req.header.stamp = rclpy.time.Time().to_msg()

        self.get_logger().debug(f"Got prompt! '{msg.data}'")

        def _whisper_callback_future(future):
            res = future.result()

            if res.status_code == 0:
                self.get_logger().debug(f"LLM output: {res.response}")
                self.task_pub.publish(String(data=res.response))
            else:
                self.get_logger().error(f"There was an error with code {res.status_code} during LLM Service")

        self.llm_client.call_async(req).add_done_callback(_whisper_callback_future)

        

def main(args=None):

    rclpy.init(args=args)

    node = LLMNode()

    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()