# Parcial3Robotica
Punto de planeador de trayectorias para robot SCARA a partir de un archivo .dxf y vista en simulación con rviz y xacro.

## Para ejecutar el codigo
- Copiar el repositorio en wsl en la carpeta ros2_ws (abrir terminal en windows, hacer "cd ros2_ws/src") con el comando "git clone ..."
- Compilar el paquete, haciendo "colcon build --packages-select dxf_parser" dentro de la carpeta ros2_ws (hacer "cd ros2_ws" primero)
- Actualizar el bash con "source install/setup.bash"
- Abrir terminator con "terminator"
- Abrir 7 terminales separadas en terminator
- Ejecutar el .xacro con "ros2 launch dxf_parser display.launch.py" en una terminal
- Ejecutar nodo extractor de coordenadas a partir de archivo .dxf con "ros2 run dxf_parser dxf_parser_node" en otra terminal
- Ejecutar el nodo de cinematica inversa con "ros2 run dxf_parser ik_node" en otra terminal
- Ejecutar el nodo planeador de trayectorias con "ros2 run dxf_parser trajectory_planner_node" en otra terminal
- Ejecutar el nodo de cinematica directa con "ros2 run dxf_parser fk_node" en otra terminal
- Visualizar el rqt_graph con "rqt_graph" en otra terminal
- Escuchar el topico resultante de la cinematica directa con "ros2 topic echo /joint_trajectory" en otra terminal

Asi, se tendria todo para visualizar la animacion del robot SCARA.


