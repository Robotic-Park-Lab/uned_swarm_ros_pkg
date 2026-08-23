# uned_swarm_task

`ament_python` package with high-level coordination nodes for multi-agent/swarm experiments — the centralized counterpart to each platform's own distributed formation nodes (`uned_crazyflie_missions`, `uned_kheperaiv_task`).

## Nodes

- **`centralized_formation_controller`** (`centralized_formation_controller.py`, node name `formation_controller`): the main centralized formation node, configured entirely from a `config_file` `.yaml` (`Architecture.node` in the experience schema used by `uned_swarm_config/launch/generic.launch.py`). Tracks each robot as an `Agent` (subscribes to `<id>/local_pose`, publishes `<id>/goal_pose`) and each pairwise relationship as a `Neighbour` (distance error, IAE, RViz marker), and includes its own `PIDController`.
- **`open_loop_signal`** (`open_loop_signal.py`, node name `formation_controller`): `OpenLoop`, publishes an open-loop test signal (`PoseStamped`/`Twist`/`Float64`, configurable via `output`/`output_type`) and records the corresponding input signal — used for system identification / characterization experiments ahead of `system_identification`.
- **`swarm_reconfiguration`** (`swarm_reconfiguration.py`): `SwarmReconfiguration`, reconfigures a formation's graph topology at runtime (`Edge`/`Neighbour` per-agent distance targets) and relays `swarm/order` commands to each agent's `<id>/order` topic.
- **`system_identification`** (`system_identification.py`): `SystemIdentification`, records an input/output signal pair (configurable message types: `PoseStamped`/`Twist`/`Float64`/`Point`/`Float32`, with a field path like `position.x`) and publishes an RMSE (`ident/rmse`) — the identification counterpart to `open_loop_signal`.

## Dependencies

`rclpy`, `geometry_msgs`, `std_msgs` are declared; the nodes also import `visualization_msgs` (RViz markers), `nav_msgs` (`Path`), `tf2_ros`/`tf_transformations` and `builtin_interfaces` without a matching `<depend>` — see `AUDIT.md` on the `doc` branch.

## Tests

None beyond the standard `ament_copyright`/`ament_flake8`/`ament_pep257` lint tests. All four nodes are full `rclpy` nodes with topic/parameter wiring done straight from `__init__`; none of their control/identification math has been extracted into pure, unit-testable functions yet.
