import rclpy
from rclpy.node import Node
from robot_sim_interfaces.srv import TTSService
from TTS.api import TTS
import tempfile
import os
import sounddevice as sd
import soundfile as sf

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Carga un modelo por defecto (puedes cambiarlo por otro más adelante)
        self.get_logger().info("Cargando modelo TTS...")
        self.tts = TTS(model_name="tts_models/es/tacotron2-DDC/ph", progress_bar=False, gpu=False)
        self.get_logger().info("Modelo TTS cargado.")

        self.create_service(TTSService, '/robot/tts', self.speak_callback)
        self.get_logger().info(f"TTS node ready.")

    def speak_callback(self, request, response):
        text = request.text
        self.get_logger().info(f"Hablando: {text}")

        try:
            # Generar audio en memoria
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                self.tts.tts_to_file(text=text, file_path=f.name)

                # Reproducir el archivo de audio generado
                data, samplerate = sf.read(f.name)
                sd.play(data, samplerate)
                sd.wait()

                os.remove(f.name)  # Limpia el archivo temporal

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
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()