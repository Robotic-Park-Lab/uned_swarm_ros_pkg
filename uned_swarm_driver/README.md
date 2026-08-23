# uned_swarm_driver

`ament_python` package with robot-side driver nodes used specifically in multi-agent/swarm experiments — distinct from each platform's own driver package (`uned_crazyflie_driver`, `uned_kheperaiv_driver`, ...), which this package's nodes sit on top of or work alongside.

## Nodes

- **`swarm_driver`** (`swarm_driver.py`): `SwarmDriver`, subscribes to `<id>/pose` for every robot id listed in its `robots` parameter and publishes the centroid of the whole group on `swarm/pose`, plus relaying orders from `swarm/cf_order` to each robot's own `<id>/order` topic. Assumes each robot already publishes `pose` itself (e.g. via `uned_vicon_gazebo`/`uned_crazyflie_driver`/`uned_kheperaiv_driver`) — it doesn't drive anything on its own.
- **`tello_gazebo_driver`** (`tello_gazebo_driver.py`): `TelloDriver`, a full formation-control driver for a DJI Tello in Gazebo (physical or `digital_twin`). Talks to the submodule's own `tello_driver` node via the `tello_msgs/TelloAction` service (`takeoff`/`land`) and publishes `cmd_vel` from an internal position PID loop; supports the same distance-based formation law (`Agent`/`PIDController`, `task.enable`/`relationship` config) as `uned_crazyflie_driver` and `uned_kheperaiv_driver`'s own formation code.
- **`turtlebot_driver`** (`turtlebot_driver.py`): `TurtlebotDriver`, a position controller for a TurtleBot3 — subscribes to `<ns>/pose`/`odom`, publishes `cmd_vel` from a simple attractive-force + heading controller towards `goal_pose`.

None of the three currently declare a `<depend>` on `rclpy`/`geometry_msgs`/etc. in `package.xml` (only lint `test_depend`s) despite importing them for real — see `AUDIT.md` on the `doc` branch.

## Tests

None beyond the standard `ament_copyright`/`ament_flake8`/`ament_pep257` lint tests. All three nodes are full `rclpy` nodes with topic/parameter/service wiring done straight from `__init__`; none of their control math has been extracted into pure, unit-testable functions yet (candidate follow-up, same pattern used for `PIDController` in `uned_crazyflie_driver`/`uned_kheperaiv_driver`).
