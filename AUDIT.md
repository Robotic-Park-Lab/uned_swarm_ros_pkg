# Auditoría — uned_multi_agent_ros_pkg (2026-08-23)

Checklist fundamentada en inspección real de `humble-dev`: lectura completa de los 3 paquetes (`uned_swarm_config`, `uned_swarm_driver`, `uned_swarm_task`), sus `package.xml`/`setup.py`, los archivos de lanzamiento y `.yaml` de `resources/`, más un `colcon build`/`colcon test` real y aislado de los 3 paquetes. Primera auditoría de este repositorio dentro del pase repo-by-repo (a diferencia de Crazyflie/Khepera, no ha habido todavía una restructuración de código — esta pasada es solo documentación bilingüe + hallazgos, sin tocar código; los puntos de abajo están **sin resolver**, a la espera de que decidas cuáles abordar).

`benchmark` está bloqueada y fuera de alcance — no se ha tocado ni leído nada de ella en esta pasada.

## 1 — Documentación (resuelto en esta pasada)

- [x] **README raíz incompleto**: listaba `uned_swarm_config` y `uned_swarm_task` en su sección de estructura pero **no `uned_swarm_driver`**, pese a que el paquete existe en el árbol con 3 nodos reales (`swarm_driver`, `tello_gazebo_driver`, `turtlebot_driver`). Corregido: los 3 paquetes documentados en el README raíz (ahora en inglés, con `README_es.md` en español) y cada uno tiene su propio `README.md`/`README_es.md` por primera vez.

## 2 — Metadatos sin rellenar (los 3 paquetes)

- [ ] **`TODO: Package description`** / **`TODO: License declaration`** sin rellenar en los `package.xml` de `uned_swarm_driver` y `uned_swarm_task` (`uned_swarm_config` sí tiene licencia, aunque con el valor no estándar `BSD-3-Clause license` en vez de `BSD-3-Clause`). Mismo patrón en los `setup.py` de `uned_swarm_driver`/`uned_swarm_task` (`description='TODO: Package description'`, `license='BSD-3-Clause license'`).
- [ ] **Maintainer inconsistente con el resto del laboratorio**: los 3 `package.xml` declaran `<maintainer email="fjmanas@dia.uned.es">kiko</maintainer>` — el email es correcto pero el nombre no coincide con `Francisco José Mañas Álvarez`, usado en todos los demás repositorios activos del laboratorio.

## 3 — Dependencias no declaradas

- [ ] **`uned_swarm_driver`**: su `package.xml` no tiene ningún `<depend>` (solo `test_depend` de lint) pese a importar de verdad `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `tf2_ros`, `tf_transformations`, `yaml`, y el mensaje/servicio `tello_msgs` (`FlightData`... en realidad `TelloAction`, ver más abajo).
- [ ] **`uned_swarm_task`**: declara `rclpy`/`geometry_msgs`/`std_msgs` pero no `visualization_msgs` (marcadores de RViz en `centralized_formation_controller.py`), `nav_msgs` (`Path` en `open_loop_signal.py`), `tf2_ros`/`tf_transformations`/`builtin_interfaces` (usados en varios de los 4 nodos).
- [ ] **`tello_msgs`** (del submódulo de `uned_tello_ros_pkg`) es una dependencia cruzada real de `uned_swarm_driver/tello_gazebo_driver.py` (`from tello_msgs.srv import TelloAction`), no documentada en ningún README ni declarada en `package.xml`.

## 4 — Código muerto/rastro de git no limpiado

- [ ] **Ficheros `.pyc`/`__pycache__` versionados en git**: `uned_swarm_driver/uned_swarm_driver/__pycache__/tello_gazebo_driver.cpython-310.pyc`, `uned_swarm_task/uned_swarm_task/__pycache__/{__init__,centralized_formation_controller}.cpython-310.pyc` están commiteados en el repositorio, pese a que `.gitignore` ya tiene entradas `uned_swarm_task/uned_swarm_task/__pycache__/` y `uned_swarm_driver/uned_swarm_driver/__pycache__/` — se añadieron al índice antes de que existiera esa regla y nunca se retiraron con `git rm --cached`. Mismo patrón detectado en otros repositorios de la organización (ver `AUDIT.md` de `RoboticPark`).

## 5 — Archivos de lanzamiento rotos

- [ ] **`uned_swarm_config/launch/AC10_Sensor_Gazebo.launch.py`** y **`AC10_Sensor_Webots.launch.py`**: ambos referencian con ruta fija `resources/AC10_RoboticPark.yaml`, `rviz/AC10_RoboticPark.rviz` y `worlds/AC10_RoboticPark.wbt` — **ninguno de los tres existe** en el paquete actual (comprobado con `git ls-tree`; los ficheros reales en `resources/`/`rviz/`/`worlds/` son de las familias `MultiRobot10_*`/`RIAI2023_*`). Ambos lanzamientos fallarán con `FileNotFoundError` en cuanto se ejecuten.
- [ ] **`AC10_Sensor_Webots.launch.py`** además hace `get_package_share_directory('uned_crazyflie_webots')` — ese paquete ROS ya no existe, se absorbió en `uned_crazyflie_driver` durante la reestructuración del repositorio de Crazyflie (2026-08-22). Este lanzamiento fallaría también por esa razón, independientemente del punto anterior.
- [ ] No está claro si estos dos archivos son código muerto que conviene retirar (ya cubierto por el `generic.launch.py` parametrizado) o si el experimento "AC10" sigue siendo relevante y merece sus propios `resources/AC10_RoboticPark.*` reconstruidos — **decisión tuya**, no se ha tocado ninguno de los dos archivos en esta pasada.

## 6 — Sin tests funcionales, y lint en rojo (verificado con `colcon build`/`colcon test` reales)

Los 3 paquetes **compilan limpio** (`colcon build` real, sin errores). Pero `colcon test` real muestra que el lint está en rojo en 2 de los 3:

- [ ] **`uned_swarm_driver`**: falla `test_copyright` y falla `test_flake8` con **140 avisos de estilo** (principalmente en `tello_gazebo_driver.py`: imports sin usar, líneas >99 caracteres, espacios/operadores mal formateados, dos variables asignadas y nunca usadas — `delta` en `dt_pose_callback`, `L` en `IPC_controller`, la misma clase de hallazgo que ya se documentó como real en Crazyflie/Khepera, no simple ruido de estilo). `test_pep257` sí pasa.
- [ ] **`uned_swarm_task`**: falla `test_copyright`, falla `test_pep257`, y falla `test_flake8` con **410 avisos de estilo** — el peor de los 3 paquetes, concentrado sobre todo en `centralized_formation_controller.py` (669 líneas).
- [ ] **`uned_swarm_config`**: no tiene ningún test real registrado (ni siquiera se genera un resultado de test) — su `CMakeLists.txt` no activa `ament_lint_auto`/`ament_lint_common` de forma efectiva pese a declararlos en `package.xml`.
- [ ] Ninguna de las matemáticas de control (`PIDController` de `tello_gazebo_driver.py`/`centralized_formation_controller.py`, el controlador de posición de `turtlebot_driver.py`, la lógica de reconfiguración de `swarm_reconfiguration.py`) tiene cobertura unitaria — mismo tipo de deuda que ya se resolvió en Crazyflie/Khepera extrayendo la lógica pura y testeándola con `pytest`, no intentado aquí.

Verificado con un `colcon build`/`colcon test` real y aislado (workspace de verificación aparte, para no interferir con el `uned_swarm_ros_pkg` que todavía queda clonado localmente con el nombre antiguo) — no una suposición a partir de leer el código.
