# Large Language Model folder

Esta carpeta contiene dos paquetes relacionados con el puente entre el LLM y ROS2, siendo `llm_bridge` y `llm_bridge_interfaces`.

- **LLM Bridge**: Este paquete contiene el servicio que carga y conecta el LLM, y contiene un nodo que escucha la información que llega desde el topic del STT (`/robot/whisper`) o desde el topic de la entrada de teclado (`/robot/input`) y la procesa haciendo una llamada a la LLM.

- **LLM Bridge Interfaces**: Este paquete contiene la interfaz `LLMService` con la que se definen los mensajes que se enviarán durante las llamadas al servicio.

## Configuración previa

Si vas a ejecutar el servicio desde un edge, y el nodo intermedio desde el robot, antes hay que configurar ambas máquinas para habilitar la conexión por red desde ROS2, accediendo de dicha forma al servicio de LLM de forma remota.

## Ejecutables del paquete

### Ejecución del nodo

> **¡Recuerda!** El servicio de LLM (si lanza el modelo en la máquina) se debe ejecutar en un sistema potente dedicado al cómputo (en este caso, el Ultra Edge).

Para ejecutar los nodos habría que ejecutar el siguiente comando:

```sh
ros2 launch llm_bridge local_launch.py # el archivo `launch.py` contiene información sobre el robot físico
```

### Ejecución de batería de pruebas

Para ejecutar la batería de pruebas para las pruebas experimentales, debe ejecutar los siguientes comandos:

```sh
ros2 launch llm_brige test_launch.py
```