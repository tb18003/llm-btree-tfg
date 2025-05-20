import rclpy
from rclpy.node import Node

from llm_bridge_interfaces.srv import LLMService
from ament_index_python import get_package_share_directory
import os
import dotenv

import rclpy.time

# LLMs instances
from .models.llm_factory import LargeLanguageModelFactory

# XML Parser for Prompt
import xml.etree.ElementTree as ET

# Time 
import time

class LLMBridgeService(Node):

    def __init__(self):
        super().__init__("llm_bridge_service")

        llm_topic_param = self.declare_parameter("LLM_SERVICE_TOPIC", "/llm")
        llm_model_param = self.declare_parameter("LLM_MODEL_ID", "meta-llama/Llama-3.1-8B-Instruct")

        # Load secrets
        dotenv.load_dotenv(os.path.join(get_package_share_directory('llm_bridge'),'config','.env'))

        # LLM params
        self.max_new_tokens_param = self.declare_parameter("LLM_MAX_NEW_TOKENS", 700)
        self.temperature_param = self.declare_parameter("LLM_TEMPERATURE", 0.5)
        self.top_k_param = self.declare_parameter("LLM_TOP_K", 50)
        self.top_p_param = self.declare_parameter("LLM_TOP_P", 0.9)

        # System prompt file should be located on config folder and be added to 'setup.py'
        self.sys_prompt_file_param = self.declare_parameter("LLM_SYS_PROMPT_FILE", "sys.prompt")

        self.system_prompt = ET.parse(os.path.join(get_package_share_directory('llm_bridge'), 'config', self.sys_prompt_file_param.value)).getroot()

        load_time = time.time()

        self.get_logger().info("Loading model... (It may take a while)")
        self._model = LargeLanguageModelFactory.get_instance(llm_model_param.value, {
            'max_new_tokens': self.max_new_tokens_param.value,
            'temperature': self.temperature_param.value,
            'top_k': self.top_k_param.value,
            'top_p': self.top_p_param.value,
        })

        self.get_logger().info("Model loaded: %s" % self._model.get_model_name())

        self.get_logger().info("Creating service...")
        self.service = self.create_service(LLMService, llm_topic_param.value, self.callback_service)

        self.get_logger().info(f"LLM Bridge Service ready! Elapsed time: {round(time.time() - load_time, 4)}")

    def callback_service(self, req, res):
        prompt = {
            'system': self.system_prompt.findtext('./system'),
            'user': self.system_prompt.findtext('./user') % req.prompt
        }

        try:
            res.response = self._model.generate(prompt)
        except:
            res.status_code = 1
        finally:
            res.header.stamp = rclpy.time.Time().to_msg()
            return res

    def destroy_node(self):
        self.get_logger().info("Unloading model...")
        self._model.unload()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LLMBridgeService()

        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(e)
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()