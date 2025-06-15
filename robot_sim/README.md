# Robot Simulator Package

Este paquete contiene principalmente funcionalidades para poder realizar una simulación completa del robot, tales como la <b>simulación en Gazebo</b> o las implementaciones del *speech to text* o el *text to speech*.

> **¡Ojo!** Las implementaciones del STT y TTS pueden utilizarse tanto para la simulación como para una ejecución en el robot real, ya que estas implementaciones son válidas para ambas ejecuciones.

## Ejecutables del paquete

- Launcher `tb3_launch.py`: Este *launcher* lanza el simulador de Gazebo junto al stack de navegación.

- Nodo: `tts_sim_node`: Este nodo lanza el modelo y servicio de *text to speech*.

- Nodo: `whisper_sim_node`: Este nodo lanza el modelo whisper y servicio de *speech to text*.

