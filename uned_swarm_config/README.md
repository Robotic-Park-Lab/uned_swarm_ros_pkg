# uned_swarm_config

`ament_cmake` package with launch files and environment configuration for multi-agent/swarm experiments combining several robot platforms from the lab.

## Structure

- **`launch/generic.launch.py`**: the main, parametrized launch file — reads an experience `.yaml` (via `config_file`, resolved from `resources/`) with the same `Operation`/`Architecture`/`Robots`/`CPU_Monitoring`/`Interface` section schema used by `roboticpark_config`, `uned_crazyflie_config` and `uned_kheperaiv_config`'s own `experience.launch.py`/`generic.launch.py`. `Operation.tool` selects Webots or Gazebo; `Architecture.mode: centralized` launches a single coordination node (`pkg`/`executable`/`name` from the yaml, matching `uned_swarm_task`'s nodes); per-robot entries in `Robots` are told apart by name (`dron*` → Crazyflie URDF, `khepera*` → Khepera IV URDF) and spawned physical vs. virtual accordingly.
- **`launch/AC10_Sensor_Gazebo.launch.py`** / **`launch/AC10_Sensor_Webots.launch.py`**: older, hand-written (non-generic) launch files for a specific "AC10" experiment combining Crazyflie and Khepera IV robots. **Broken, not fixed in this pass**: both hardcode `resources/AC10_RoboticPark.yaml` / `rviz/AC10_RoboticPark.rviz` / `worlds/AC10_RoboticPark.wbt`, none of which exist in this package's `resources/`/`rviz/`/`worlds/` today — see `AUDIT.md` on the `doc` branch. `AC10_Sensor_Webots.launch.py` additionally references the `uned_crazyflie_webots` ROS package, which no longer exists (absorbed into `uned_crazyflie_driver` during that repo's own restructuring).
- **`resources/`**: experience `.yaml` files for `generic.launch.py` (`MultiRobot10_*`, `RIAI2023_*`, plus their `*_topics.yaml` companions), the two robots' URDF (`crazyflie.urdf`, `kheperaiv.urdf`), and lab logo images used by Webots worlds.
- **`rviz/`, `rqt/`**: RViz configs and RQT perspectives tied to specific experiments (`MultiRobot10`, `RIAI2023`), plus a `default.perspective`.
- **`worlds/`**: Webots worlds for the same experiments, and their shared meshes.

## Tests

None beyond the standard `ament_lint_auto`/`ament_lint_common` lint tests — this package has no nodes of its own, only launch files and resources.
