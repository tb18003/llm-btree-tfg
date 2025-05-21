import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from llm_bridge_interfaces.srv import LLMService

import datetime

class LLMEvaluatorNode(Node):

    def __init__(self):
        super().__init__('llm_bridge_stats_test')

        self.cli = self.create_client(SetParameters, '/llm_test_service_node/set_parameters')
        self.p_client = self.create_client(LLMService, '/llm')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('🕔 Waiting until parameters service is active...')

        self.data: dict[str, list[dict]] = {}
        self.models, self.prompts = self.declare_parameters(
            namespace='',
            parameters=[
                ("TEST_MODELS", ["gemini-2.0-flash"]),
                ("TEST_PROMPTS", ["Hola, ¿Quién eres?"])
            ]
        )

    def execute_all_tests(self):
        m_len = len(self.models.value)
        p_len = len(self.prompts.value)
        total = m_len * p_len
        self.get_logger().info(f"-- Test information: {m_len} models, {p_len} prompts = {total} tests")
        self.get_logger().info("🚀 Starting tests --")

        i = 0

        for m in self.models.value:
            self.get_logger().info(f"🚀 {m} Model Tests")
            self.get_logger().info("-----------------------------")
            self.data[m] = []
            self._load_model(m)
            i += 1
            self._model_test(m, i, total)

        self.get_logger().info("✔️ Test finished!")
        self.save_output()

    def _model_test(self, model, m_i, total):
        i = 0
        p_len = total // len(self.models.value)

        for p in self.prompts.value:
            self._single_test(p, model)
            i += 1
            self.get_logger().info(f"\t\t✅ Test done ({((p_len * m_i + i) * 100) // total}% completed)")
        
    def _load_model(self, model_id):
        req = SetParameters.Request()

        param = Parameter()
        param.name = "LLM_MODEL_ID"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = model_id
        req.parameters.append(param)

        self.get_logger().info("\t⏳ Loading model...")
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        self.get_logger().info("\t✅ Model loaded!")

        return f.result()
    
    def _single_test(self, prompt, model):
        self.get_logger().info("\t\t⏳ Test running...")
        req = LLMService.Request()
        req.header.stamp.sec, req.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        req.prompt = prompt

        while not self.p_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('🕔 Waiting until LLM service is active...')

        f = self.p_client.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        res = f.result()

        if res.status_code == 0:
            self.data[model].append({
                "input": prompt,
                "output": res.response.replace("\n", ""),
                "time": res.header.stamp.sec - req.header.stamp.sec + round((res.header.stamp.nanosec - req.header.stamp.nanosec) / 1000000000,4)
            })
        else:
            self.get_logger().error(f"\t\t❌ Test with prompt '{prompt}' failed :(")


    def data_to_str(self, data: dict):
        s = 'Model;Input;Output;Time\n'
        for (k,v) in data.items():
            s = s + ''.join([f'{k};{e["input"]};{e["output"]};{e["time"]}\n' for e in v ])

        return s

    def save_output(self):
        with open("/home/ubuntu/.logs/%s.csv" % datetime.datetime.now().strftime("%Y%m%d_%H_%M_%S"), "w+") as file:
            file.write(self.data_to_str(self.data))

    

def main(args=None):
    rclpy.init(args=args)

    node = LLMEvaluatorNode()

    try:
        node.execute_all_tests()

        node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()