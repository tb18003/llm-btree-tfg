import rclpy
from rclpy.node import Node

from llm_bridge_interfaces.srv import LLMService
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
import os
import dotenv


# LLMs instances
from .models.llm_factory import LargeLanguageModelFactory

# XML Parser for Prompt
import xml.etree.ElementTree as ET

# Time 
import time

class LLMBridgeService(Node):

    def __init__(self):
        super().__init__("llm_service_node")

        self.llm_topic_param = self.declare_parameter("LLM_SERVICE_TOPIC", "/llm")
        self.llm_model_param = self.declare_parameter("LLM_MODEL_ID", "gemini-2.0-flash")
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

        self.add_on_set_parameters_callback(self._set_params_callback)
        self.get_logger().info("LLM Bridge Service started!")
        self.get_logger().info(f"Note: use 'ros2 param set /{self.get_name()} LLM_MODEL_ID <model>' change the model!")

        if self.llm_model_param.value != "":
            self.load_model(self.llm_model_param.value)

    def callback_service(self, req, res):
        prompt = {
            'system': self.system_prompt.findtext('./system'),
            'user': self.system_prompt.findtext('./user') % req.prompt
        }

        try:
            res.response = self._model.generate(prompt)
        except Exception as e:
            res.status_code = 1
            res.response = "Couldn't generate a response, do you have the model loaded?"
            self.get_logger().info("Error occurred: %s" % e)
        finally:
            res.header.stamp.sec, res.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
            return res

    def load_model(self, model_id: str):
        t = time.time()
        self.get_logger().info("Loading model... (It may take a while)")
        self._model = LargeLanguageModelFactory.get_instance(model_id, {
            'max_new_tokens': self.max_new_tokens_param.value,
            'temperature': self.temperature_param.value,
            'top_k': self.top_k_param.value,
            'top_p': self.top_p_param.value,
        })
        if self._model is None:
            self.get_logger().error(f"Model {model_id} is not available!")
            return False
        
        self.get_logger().info(f"Model {self._model.get_model_name()} loaded! Elapsed time: {round(time.time() - t, 4)}")
        self.get_logger().info("Creating service...")
        self.service = self.create_service(LLMService, self.llm_topic_param.value, self.callback_service)
        self.get_logger().info("Service created!")
        return True

    def _set_params_callback(self, params: list):
        r = SetParametersResult()
        for param in params:
            if param.name == self.llm_model_param.name and param.type_ == self.llm_model_param.type_:
                r.successful = self._set_model(param.value)

        return r

    def _set_model(self, model_id: str):
        if hasattr(self, '_model') and self._model is not None:
            if self._model.get_model_name() == model_id:
                return True
            else:
                self.get_logger().warn("Changing model, service is actually off!")
                self.destroy_service(self.service)
                self._model.unload()

        return self.load_model(model_id)

    def destroy_node(self):
        if hasattr(self, '_model') and self._model:
            print("-\tUnloading model...")
            self._model.unload()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = LLMBridgeService()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        
        print("")

if __name__ == "__main__":
    main()