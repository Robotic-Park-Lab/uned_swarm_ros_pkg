# uned_swarm_task

Paquete `ament_python` con nodos de coordinación de alto nivel para experimentos multi-agente/enjambre — la contrapartida centralizada de los nodos de formación distribuida propios de cada plataforma (`uned_crazyflie_missions`, `uned_kheperaiv_task`).

## Nodos

- **`centralized_formation_controller`** (`centralized_formation_controller.py`, nombre de nodo `formation_controller`): el nodo principal de formación centralizada, configurado enteramente desde un `.yaml` `config_file` (`Architecture.node` en el esquema de experiencia usado por `uned_swarm_config/launch/generic.launch.py`). Sigue cada robot como una `Agent` (se suscribe a `<id>/local_pose`, publica `<id>/goal_pose`) y cada relación por pares como un `Neighbour` (error de distancia, IAE, marcador de RViz), e incluye su propio `PIDController`.
- **`open_loop_signal`** (`open_loop_signal.py`, nombre de nodo `formation_controller`): `OpenLoop`, publica una señal de test en bucle abierto (`PoseStamped`/`Twist`/`Float64`, configurable vía `output`/`output_type`) y registra la señal de entrada correspondiente — usado para experimentos de identificación/caracterización de sistemas previos a `system_identification`.
- **`swarm_reconfiguration`** (`swarm_reconfiguration.py`): `SwarmReconfiguration`, reconfigura en tiempo de ejecución la topología del grafo de una formación (objetivos de distancia por agente `Edge`/`Neighbour`) y retransmite comandos de `swarm/order` al topic `<id>/order` de cada agente.
- **`system_identification`** (`system_identification.py`): `SystemIdentification`, registra un par de señales entrada/salida (tipos de mensaje configurables: `PoseStamped`/`Twist`/`Float64`/`Point`/`Float32`, con una ruta de campo como `position.x`) y publica un RMSE (`ident/rmse`) — la contrapartida de identificación de `open_loop_signal`.

## Dependencias

`rclpy`, `geometry_msgs`, `std_msgs` están declaradas; los nodos también importan `visualization_msgs` (marcadores de RViz), `nav_msgs` (`Path`), `tf2_ros`/`tf_transformations` y `builtin_interfaces` sin un `<depend>` correspondiente — ver `AUDIT.md` en la rama `doc`.

## Tests

Ninguno más allá de los tests de lint estándar `ament_copyright`/`ament_flake8`/`ament_pep257`. Los cuatro nodos son nodos `rclpy` completos con el cableado de topics/parámetros hecho directamente desde `__init__`; ninguna de sus matemáticas de control/identificación se ha extraído todavía a funciones puras testeables unitariamente.
