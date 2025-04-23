import rclpy
from rclpy.node import Node

from llm_bridge_interfaces.srv import LLMService
from ament_index_python import get_package_share_directory
import os

# UEDGE dependencies
import gc
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM



# Time 
import time

class LLMBridgeService(Node):

    def __init__(self):
        super().__init__("llm_bridge_service")

        llm_topic_param = self.declare_parameter("LLM_SERVICE_TOPIC", "/llm")
        llm_model_param = self.declare_parameter("LLM_MODEL_ID", "meta-llama/Llama-3.1-8B-Instruct")

        # LLM params
        self.max_new_tokens = self.declare_parameter("LLM_MAX_NEW_TOKENS", 700)
        self.temperature_param = self.declare_parameter("LLM_TEMPERATURE", 0.5)
        self.top_k_param = self.declare_parameter("LLM_TOP_K", 50)
        self.top_p_param = self.declare_parameter("LLM_TOP_P", 0.9)

        # System prompt file should be located on config folder and be added to 'setup.py'
        self.sys_prompt_file_param = self.declare_parameter("LLM_SYS_PROMPT_FILE", "sys.llama.prompt")

        self.system_prompt = open(os.path.join(get_package_share_directory('llm_bridge'), 'config', self.sys_prompt_file_param.value)).read()

        load_time = time.time()

        self.get_logger().info("Loading model... (It may take a while)")
        self._model = AutoModelForCausalLM.from_pretrained(llm_model_param.value, device_map="auto")

        self.get_logger().info("Loading tokenizer...")
        self._tokenizer = AutoTokenizer.from_pretrained(llm_model_param.value, device_map="auto")

        self.get_logger().info("Creating service...")
        self.service = self.create_service(LLMService, llm_topic_param.value, self.callback_service)

        self.get_logger().info(f"LLM Bridge Service ready! Elapsed time: {round(time.time() - load_time, 4)}")

    def callback_service(self, req, res):
        prompt = self.system_prompt % req.prompt

        inputs = self._tokenizer(prompt, return_tensors="pt")
        output = self._model.generate(
            inputs.input_ids.to("cuda"), 
            max_new_tokens=self.max_new_tokens.value,
            num_return_sequences=1,
            temperature=self.temperature_param.value,
            top_k=self.top_k_param.value,
            top_p=self.top_p_param.value,
            do_sample=True,
            pad_token_id=self._tokenizer.eos_token_id # To avoid warning: The attention mask and the pad token id were not set. As a consequence, you may observe unexpected behavior. Please pass your input's `attention_mask` to obtain reliable results.
        )
        res.response = self._tokenizer.decode(output[0], skip_special_tokens=False)

        marker = "<|start_header_id|>assistant<|end_header_id|>"
        txt_start = res.response.rfind(marker) + len(marker)
        res.response = res.response[txt_start:].split("<|eot_id|>")[0].strip()

        self.get_logger().info(res.response)

        return res

    def destroy_node(self):
        self.get_logger().info("Unloading model...")
        del self._model
        gc.collect()
        torch.cuda.empty_cache()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = LLMBridgeService()

    rclpy.spin(node=node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()