# Task folder

Para ejecutar el `task_executor` debemos realizar los siguientes pasos:

> **Advertencia** Este paquete depende de del paquete `robot_sim`

1. Compilar el proyecto dentro del workspace utilizando el comando
    ```{bash}
    cd ros2_ws
    colcon build --symlink-install
    ```

2. Ejecutar el launcher (recomendado)
    - Archivo de lanzamiento `te_launch.py`, ejecuta los dos nodos necesarios para su funcionamiento.
    ```{bash}
    ros2 launch robot_sim te_launch.py
    ```

3. Ejecutar los nodos necesarios por separado

    - Nodo `task_executor_node`, sirve para crear el BT y ejecutar las tareas según el orden especificado (la interfaz gráfica sirve para enviarle dicha información tal y como se la enviaría la IA).

    ```{bash}
    ros2 run task_manager task_executor_node
    ```

    - Nodo `robot_sim_node`, sirve para simular los registros y/o acciones que le llegarían al robot real.

    ```{bash}
    ros2 run robot_sim robot_sim_node
    ```

4. [BONUS] Terminar los procesos una vez finalizado su uso
    
    Para terminar con la aplicación, deberá cerrar las dos interfaces gráficas (usando Ctrl + C en consola no matará el proceso).