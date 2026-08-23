# uned_swarm_driver

Paquete `ament_python` con nodos driver del lado robot usados específicamente en experimentos multi-agente/enjambre — distintos del paquete driver propio de cada plataforma (`uned_crazyflie_driver`, `uned_kheperaiv_driver`, ...), sobre el que se apoyan o junto al que trabajan los nodos de este paquete.

## Nodos

- **`swarm_driver`** (`swarm_driver.py`): `SwarmDriver`, se suscribe a `<id>/pose` de cada id de robot listado en su parámetro `robots` y publica el centroide de todo el grupo en `swarm/pose`, además de retransmitir órdenes desde `swarm/cf_order` al topic `<id>/order` propio de cada robot. Asume que cada robot ya publica su propio `pose` (p. ej. vía `uned_vicon_gazebo`/`uned_crazyflie_driver`/`uned_kheperaiv_driver`) — no conduce nada por sí mismo.
- **`tello_gazebo_driver`** (`tello_gazebo_driver.py`): `TelloDriver`, un driver completo de control de formación para un DJI Tello en Gazebo (físico o `digital_twin`). Habla con el propio nodo `tello_driver` del submódulo vía el servicio `tello_msgs/TelloAction` (`takeoff`/`land`) y publica `cmd_vel` desde un bucle PID de posición interno; soporta la misma ley de formación basada en distancia (`Agent`/`PIDController`, configuración `task.enable`/`relationship`) que el propio código de formación de `uned_crazyflie_driver` y `uned_kheperaiv_driver`.
- **`turtlebot_driver`** (`turtlebot_driver.py`): `TurtlebotDriver`, un controlador de posición para un TurtleBot3 — se suscribe a `<ns>/pose`/`odom`, publica `cmd_vel` desde un controlador simple de fuerza atractiva + orientación hacia `goal_pose`.

Ninguno de los tres declara actualmente un `<depend>` de `rclpy`/`geometry_msgs`/etc. en `package.xml` (solo `test_depend` de lint) pese a importarlos de verdad — ver `AUDIT.md` en la rama `doc`.

## Tests

Ninguno más allá de los tests de lint estándar `ament_copyright`/`ament_flake8`/`ament_pep257`. Los tres nodos son nodos `rclpy` completos con el cableado de topics/parámetros/servicios hecho directamente desde `__init__`; ninguna de sus matemáticas de control se ha extraído todavía a funciones puras testeables unitariamente (candidato a trabajo futuro, mismo patrón usado para `PIDController` en `uned_crazyflie_driver`/`uned_kheperaiv_driver`).
