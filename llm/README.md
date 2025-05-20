# Large Language Model folder

Esta carpeta contiene dos paquetes relacionados con el puente entre el LLM y ROS2, siendo `llm_bridge` y `llm_bridge_interfaces`.

- **LLM Bridge**: Este paquete contiene el servicio que carga y conecta el LLM, y contiene un nodo que escucha la información que llega desde el topic de whisper (`/robot/whisper`) y la procesa haciendo una llamada a la LLM.

- **LLM Bridge Interfaces**: Este paquete contiene la interfaz `LLMService` con la que se definen los mensajes que se enviarán durante las llamadas al servicio.

## Configuración previa

Si vas a ejecutar el servicio desde un edge, y el nodo intermedio desde el robot, antes hay que configurar ambas máquinas para habilitar la conexión por red desde ROS2, accediendo de dicha forma al servicio de LLM de forma remota.

## Ejecución

> **¡Recuerda!** El servicio de LLM (si lanza el modelo en la máquina) se debe ejecutar en un sistema potente dedicado al cómputo (en nuestro caso, el Ultra Edge).

Para ejecutar los nodos habría que realizar los siguientes comandos:

```sh
ros2 run llm_bridge llm_bridge_service # Ejecutar en Ultra Edge
ros2 run llm_bridge llm_bridge_node
```

## Pruebas

Para ejecutar la batería de pruebas para las pruebas experimentales, debe ejecutar los siguientes comandos:

```sh
ros2 run llm_bridge llm_bridge_service # Ejecutar en Ultra Edge
ros2 run llm_bridge llm_bridge_test
```