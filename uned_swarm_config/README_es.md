# uned_swarm_config

Paquete `ament_cmake` con archivos de lanzamiento y configuración de entorno para experimentos multi-agente/enjambre que combinan varias plataformas de robot del laboratorio.

## Estructura

- **`launch/generic.launch.py`**: el archivo de lanzamiento principal, parametrizado — lee un `.yaml` de experiencia (vía `config_file`, resuelto desde `resources/`) con el mismo esquema de secciones `Operation`/`Architecture`/`Robots`/`CPU_Monitoring`/`Interface` que usan los propios `experience.launch.py`/`generic.launch.py` de `roboticpark_config`, `uned_crazyflie_config` y `uned_kheperaiv_config`. `Operation.tool` selecciona Webots o Gazebo; `Architecture.mode: centralized` lanza un único nodo de coordinación (`pkg`/`executable`/`name` desde el yaml, coincidiendo con los nodos de `uned_swarm_task`); las entradas por robot en `Robots` se distinguen por nombre (`dron*` → URDF de Crazyflie, `khepera*` → URDF de Khepera IV) y se lanzan como físicos o virtuales según corresponda.
- **`launch/AC10_Sensor_Gazebo.launch.py`** / **`launch/AC10_Sensor_Webots.launch.py`**: archivos de lanzamiento más antiguos, escritos a mano (no genéricos) para un experimento concreto "AC10" que combina robots Crazyflie y Khepera IV. **Rotos, no corregidos en esta pasada**: ambos tienen escrito a fuego `resources/AC10_RoboticPark.yaml` / `rviz/AC10_RoboticPark.rviz` / `worlds/AC10_RoboticPark.wbt`, ninguno de los cuales existe hoy en `resources/`/`rviz/`/`worlds/` de este paquete — ver `AUDIT.md` en la rama `doc`. `AC10_Sensor_Webots.launch.py` además referencia el paquete ROS `uned_crazyflie_webots`, que ya no existe (absorbido en `uned_crazyflie_driver` durante la propia reestructuración de ese repositorio).
- **`resources/`**: ficheros `.yaml` de experiencia para `generic.launch.py` (`MultiRobot10_*`, `RIAI2023_*`, más sus `*_topics.yaml` acompañantes), el URDF de los dos robots (`crazyflie.urdf`, `kheperaiv.urdf`), e imágenes del logo del laboratorio usadas por los mundos de Webots.
- **`rviz/`, `rqt/`**: configuraciones de RViz y perspectivas de RQT ligadas a experimentos concretos (`MultiRobot10`, `RIAI2023`), más un `default.perspective`.
- **`worlds/`**: mundos de Webots para los mismos experimentos, y sus mallas compartidas.

## Tests

Ninguno más allá de los tests de lint estándar `ament_lint_auto`/`ament_lint_common` — este paquete no tiene nodos propios, solo archivos de lanzamiento y recursos.
