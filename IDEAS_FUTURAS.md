# Ideas y trabajo futuro — uned_multi_agent_ros_pkg

Documento vivo para anotar ideas, mejoras y líneas de trabajo futuras que no son bugs ni pendientes de la auditoría actual (eso vive en `AUDIT.md`, en esta misma rama). Añade entradas libremente, con fecha, según vayan surgiendo.

## Ideas abiertas

- **(2026-08-23) Renombrar los paquetes internos `uned_swarm_*`.** El repositorio se renombró de `uned_swarm_ros_pkg` a `uned_multi_agent_ros_pkg` en 2026-08, pero los 3 paquetes ROS por dentro siguen llamándose `uned_swarm_config`/`uned_swarm_driver`/`uned_swarm_task` para no romper referencias existentes sin avisar (ver README raíz). Si en algún momento se hace ese renombrado, hay que actualizar en el mismo cambio: `roboticpark_config`/`install.sh` en `RoboticPark`, y cualquier `<depend>` cruzado de otros repositorios.
- **(2026-08-23) Reconstruir o retirar el experimento "AC10".** `AC10_Sensor_Gazebo.launch.py`/`AC10_Sensor_Webots.launch.py` están rotos porque les faltan sus `resources/`/`rviz/`/`worlds/` (ver `AUDIT.md`). Si el experimento sigue siendo relevante, la vía más limpia es migrarlo al esquema de `.yaml` de experiencia que ya usa `generic.launch.py`, en vez de reconstruir sus recursos originales tal cual.
- **(2026-08-23) Extraer y testear la lógica de control pura.** `PIDController` aparece de forma independiente en `tello_gazebo_driver.py` y en `centralized_formation_controller.py` — candidato a unificar en un módulo común y testear unitariamente, siguiendo el mismo patrón ya aplicado en `uned_crazyflie_driver`/`uned_kheperaiv_driver` (extraer del nodo ROS, testear con `pytest` sin necesitar un grafo en ejecución).
- **(2026-08-23) Documentar `tello_msgs` como dependencia real.** `tello_gazebo_driver.py` depende del paquete `tello_msgs` que viene del submódulo de `uned_tello_ros_pkg` — hoy no está mencionado en ningún README de este repositorio ni declarado en `package.xml`. Si `uned_multi_agent_ros_pkg` va a seguir combinando el Tello con otras plataformas, merece la pena dejar constancia explícita de esa dependencia entre repositorios (como ya se hace con `multi_agent_pkg` de `RoboticPark` en Crazyflie/Khepera).

## Explorado y descartado

_(sin entradas todavía)_
