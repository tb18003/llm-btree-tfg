import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import whisper
import numpy as np
import sounddevice as sd
import queue

import pyttsx3
import threading

# Configuración
WAKE_WORD = "sancho"
SAMPLE_RATE = 16000  # Whisper model default
RECORD_SECONDS = 5   # Tiempo para escuchar comando después de la palabra clave

class WhisperListenerNode(Node):
    def __init__(self):
        super().__init__('whisper_listener_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.engine = pyttsx3.init()
        self.model = whisper.load_model("base")  # Puedes usar "tiny", "small", "medium", etc.

        self.audio_queue = queue.Queue()
        self.running = True

        self.get_logger().info("Nodo de escucha con Whisper iniciado.")

        # Hilo para la escucha continua
        threading.Thread(target=self.audio_loop, daemon=True).start()

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def record_audio(self, duration=RECORD_SECONDS):
        self.get_logger().info(f"Grabando audio por {duration} segundos...")
        audio = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        return np.squeeze(audio)

    def audio_loop(self):
        while self.running:
            audio = self.record_audio(duration=3)
            result = self.model.transcribe(audio, language="es", fp16=False)

            transcript = result["text"].strip().lower()
            if transcript:
                self.get_logger().info(f"Escuchado: {transcript}")
            if WAKE_WORD in transcript:
                self.get_logger().info("¡Palabra clave detectada!")
                self.speak("Te escucho")

                # Escuchar comando
                command_audio = self.record_audio(duration=RECORD_SECONDS)
                result_cmd = self.model.transcribe(command_audio, language="es", fp16=False)
                command_text = result_cmd["text"].strip()

                if command_text:
                    self.get_logger().info(f"Comando: {command_text}")
                    msg = String()
                    msg.data = command_text
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info("No se detectó ningún comando.")

def main(args=None):
    rclpy.init(args=args)
    node = WhisperListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()