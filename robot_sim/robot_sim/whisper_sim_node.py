import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import whisper
import numpy as np
import sounddevice as sd
import queue

import pyttsx3
import threading
import soundfile as sf

import os

# Configuración
WAKE_WORD = "sancho"
SAMPLE_RATE = 16000  # Whisper model default
RECORD_SECONDS = 5   # Tiempo para escuchar comando después de la palabra clave
FOUND_SOUND_PATH = get_package_share_directory('robot_sim') + '/assets/sancho_found.mp3'
FINISHED_SOUND_PATH = get_package_share_directory('robot_sim') + '/assets/sancho_finished.mp3'

class WhisperListenerNode(Node):
    def __init__(self):
        super().__init__('whisper_listener_node')
        self.publisher_ = self.create_publisher(String, '/robot/whisper', 10)
        self.engine = pyttsx3.init()
        self.model = whisper.load_model("base")  # Puedes usar "tiny", "small", "medium", etc.

        self.audio_queue = queue.Queue()
        self.running = True
        self.fsound = sf.read(FOUND_SOUND_PATH, dtype='float32')
        self.finished_sound = sf.read(FINISHED_SOUND_PATH, dtype='float32')

        self.get_logger().info("Nodo de escucha con Whisper iniciado.")

        # Hilo para la escucha continua
        threading.Thread(target=self.audio_loop, daemon=True).start()

    def play_found_sound(self):
        sd.play(self.fsound[0], self.fsound[1])

    def play_finished_sound(self):
        sd.play(self.finished_sound[0], self.finished_sound[1])
        sd.wait()

    def record_audio(self, duration=RECORD_SECONDS):
        audio = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        return np.squeeze(audio)
    
    def listen_until_silence(self, max_duration=10, silence_threshold=0.05, silence_duration=1.5):
        """
        Escucha hasta que haya silencio continuo.
        - max_duration: límite de tiempo total (segundos)
        - silence_threshold: umbral RMS para considerar silencio
        - silence_duration: duración del silencio necesario para parar
        """
        self.get_logger().info("Escuchando hasta que haya silencio...")

        sample_rate = SAMPLE_RATE
        chunk_duration = 0.3  # segundos
        chunk_samples = int(sample_rate * chunk_duration)
        max_chunks = int(max_duration / chunk_duration)
        silence_chunks_required = int(silence_duration / chunk_duration)

        audio_buffer = []
        silent_chunks = 0

        try:
            with sd.InputStream(samplerate=sample_rate, channels=1, dtype='float32') as stream:
                for _ in range(max_chunks):
                    chunk, _ = stream.read(chunk_samples)
                    chunk = np.squeeze(chunk)
                    rms = np.sqrt(np.mean(chunk**2))

                    audio_buffer.append(chunk)

                    if rms < silence_threshold:
                        silent_chunks += 1
                        if silent_chunks >= silence_chunks_required:
                            self.get_logger().info("Silencio detectado, terminando grabación.")
                            break
                    else:
                        silent_chunks = 0
        except Exception as e:
            self.get_logger().error(f"Error al grabar: {e}")
            return None

        return np.concatenate(audio_buffer)


    def audio_loop(self):
        while self.running:
            audio = self.record_audio(duration=3)
            result = self.model.transcribe(audio, language="es", fp16=False)

            transcript = result["text"].strip().lower()
            if transcript:
                self.get_logger().info(f"Escuchado: {transcript}")
            if WAKE_WORD in transcript:
                self.get_logger().info("¡Palabra clave detectada!")
                self.play_found_sound()

                # Escuchar comando
                command_audio = self.listen_until_silence()

                self.play_finished_sound()
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