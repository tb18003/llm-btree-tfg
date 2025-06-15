# Task folder

Esta carpeta contiene todos los paquetes relativos a las tareas, incluyendo tanto  el <b>ejecutor de tareas</b> como los <b>modelos</b> e <b>implementaciones de las tareas</b>. Los dos paquetes contenidos son los siguientes:

- `task_manager`: Este paquete contiene el nodo encargado de ejecutar las tareas (*task_executor_node.py*) y el modelo abstracto de las tareas junto al generador de tareas (*task_model.py*). Las tareas las recibe por el topic `/task/input`, y envía el estado de las mismas por `/task/info`.

- `task_sim`: Este paquete contiene las implementaciones de las tareas, como pueden ser las tareas de movimiento o *text to speech*.

## Ejecutables

Este paquete contiene un solo ejecutable para el nodo que se dedica a ejecutar las tareas. Para ejecutarlo es necesario realizar el siguiente comando:
```sh
ros2 run task_manager task_executor_node
```