import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from llm_bridge_interfaces.srv import LLMService


import json

class LLMNode(Node):
    """
    LLMNode is a ROS2 node that bridges input text (from Whisper or other sources)
    to a Large Language Model (LLM) service, and publishes the LLM's response as a task.

    Subscribes to:
        - Whisper topic (default: /robot/whisper)
        - Text topic (default: /robot/input)

    Publishes to:
        - Task topic (default: /task/input)

    Calls service:
        - LLMService (default: /llm)

    When a message is received on either input topic, it is sent as a prompt to the LLM service.
    The response (expected as JSON) is validated and published to the task topic.
    If the response is not valid JSON, the request is retried up to 5 times.
    """

    def __init__(self):
        super().__init__("llm_node")

        whisper_topic_param = self.declare_parameter("WHISPER_TOPIC", "/robot/whisper")
        text_topic_param = self.declare_parameter("TEXT_TOPIC", "/robot/input")
        task_topic_param = self.declare_parameter("TASKS_TOPIC", "/task/input")
        llm_topic_param = self.declare_parameter("LLM_SERVICE_TOPIC", "/llm")

        self.whisper_sub = self.create_subscription(String, whisper_topic_param.value, self._input_callback,10)
        self.text_sub = self.create_subscription(String, text_topic_param.value, self._input_callback,10)
        self.task_pub = self.create_publisher(String, task_topic_param.value, 10)

        self.llm_client = self.create_client(LLMService, llm_topic_param.value)

        
        self.get_logger().info("Waiting for LLM Service...")
        while not self.llm_client.wait_for_service(timeout_sec=1):
            pass

        self.get_logger().info("Whisper to LLM Node ready!")
        self.get_logger().info(f"Whisper topic subscription: {whisper_topic_param.value}")
        self.get_logger().info(f"Text topic subscription: {text_topic_param.value}")
        self.get_logger().info(f"Task topic publisher: {task_topic_param.value}")
        self.get_logger().info(f"LLM topic service: {llm_topic_param.value}")
        self.working = False
    
    def _input_callback(self, msg: String):
        """
        Callback que se ejecuta al recibir un mensaje de tipo String.

        Este método toma el mensaje recibido, lo utiliza como prompt para solicitar una respuesta a un servicio LLM,
        y publica la respuesta obtenida si la solicitud es exitosa. En caso de error, registra el código de error.

        Parameters
        ----------
            msg (String): Mensaje recibido que contiene el prompt para el LLM.
        """
        req = LLMService.Request()
        req.prompt = msg.data
        req.header.stamp.sec, req.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        self.get_logger().debug(f"Got prompt! '{msg.data}'")
        self._call_service(req)


    def _call_service(self, req, times=0):
        def _input_callback_future(future):
            res = future.result()

            try:
                r: str = res.response
                r = r.replace("```json", "").replace("```", "")

                json.JSONDecoder().decode(r)
            except json.JSONDecodeError:
                self.get_logger().error("LLM response is not a valid JSON, generating new response...")
                
                req.header.stamp.sec, req.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
                self._call_service(req,times+1)

            if res.status_code == 0:
                self.get_logger().debug(f"LLM output: {r}")
                self.task_pub.publish(String(data=r))
            else:
                self.get_logger().error(f"There was an error with code {res.status_code} during LLM Service")

        if times > 4:
            self.task_pub.publish(String(data="[]"))
            self.get_logger().error(f"Too many retries by this prompt, giving up")
            return

        self.llm_client.call_async(req).add_done_callback(_input_callback_future)
        
def main(args=None):

    rclpy.init(args=args)

    node = LLMNode()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == "__main__":
    main()