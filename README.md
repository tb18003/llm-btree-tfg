# Antonio Blas Moral Sánchez TFG

> English

This repository contains a ROS2 package which identificate some tasks from a sentence using Large Language Model, and planificate their execution using Behaviour Trees.

> Español

Este repositorio contiene un paquete de ROS2 el cual identifica tareas a partir de una frase utilizando Modelos de Lenguaje a Gran Escala, y planifica su ejecución mediante Árboles de Comportamiento.

### Como ejecutar el paquete
Para ejecutar una simulación completa del paquete, es necesario ejecutar los siguientes comandos:

```sh
# Para la simulación física del robot
ros2 launch robot_sim tb3_launch.py

# Para el BT y las implementaciones de las tareas
ros2 run task_manager task_executor_node

# Para el servicio de LLM y el nodo que gestiona las peticiones enviadas desde el whisper
ros2 launch llm_bridge launch.py

# Para la monitorización y uso final del paquete
ros2 run monitor_gui monitor_node
```
