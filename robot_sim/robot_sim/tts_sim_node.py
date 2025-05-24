import rclpy
from rclpy.node import Node
from robot_sim_interfaces.srv import TTSService
import pyttsx3

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.engine = pyttsx3.init()
        # Velocidad de habla
        self.engine.setProperty('rate', 150)

        for v in self.engine.getProperty('voices'):
            if v.languages and 'es' in v.languages:
                self.engine.setProperty('voice', v.id)
                break
        
        self.create_service(TTSService, '/robot/tts', self.speak_callback)
        self.get_logger().info(f"TTS node ready.")

    def speak_callback(self, request, response):
        text = request.text
        self.get_logger().info(f"Hablando: {text}")
        try:
            self.engine.say(text)
            self.engine.runAndWait()
            response.error = False
            response.error_message = ""
        except Exception as e:
            self.get_logger().error(f"Error al hablar: {e}")
            response.error = True
            response.error_message = str(e)
        finally:
            return response

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
