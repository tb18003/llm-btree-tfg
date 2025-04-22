import rclpy
from rclpy.node import Node

from llm_bridge_interfaces.srv import LLMService

class LLMBridgeService(Node):

    def __init__(self):
        super().__init__("llm_bridge_service")

        self.service = self.create_service(LLMService, "/llm", self.callback_service)
        self.get_logger().info("LLM Bridge Service ready!")

    def callback_service(self, req, res):
        res.status_code = 0
        res.response = """{"task": "talk", "args": {"speech": "Hello, you said that '%s'"}}""" % req.prompt

        self.get_logger().debug(f"Prompt: {req.prompt}")

        return res

def main(args=None):
    rclpy.init(args=args)

    node = LLMBridgeService()

    rclpy.spin(node=node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()