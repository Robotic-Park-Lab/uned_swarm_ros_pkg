# uned_multi_agent_ros_pkg

> 📖 To understand this repo's branches and its contribution guide, see the [`doc`](https://github.com/Robotic-Park-Lab/uned_multi_agent_ros_pkg/tree/doc) branch.

Repository for multi-agent functionality development in Robotic Park Lab: configuration and launch files that combine robots from different repositories (Crazyflie 2.1, Khepera IV, LIMO, DJI Tello...), and the task/control nodes that coordinate several agents at once. It was called `uned_swarm_ros_pkg` until 2026-08; it was renamed because the focus isn't exclusive to UAV swarms.

#### Structure
- **scripts**. Auxiliary files that are not part of any ROS package.
- **[uned_swarm_config](uned_swarm_config/README.md)**. ROS 2 package (`ament_cmake`). Launch and environment configuration for multi-agent experiments: `launch/generic.launch.py`, `launch/AC10_Sensor_Gazebo.launch.py`, `launch/AC10_Sensor_Webots.launch.py`.
- **[uned_swarm_driver](uned_swarm_driver/README.md)**. ROS 2 package (`ament_python`). Robot-side driver nodes used in multi-agent experiments: `swarm_driver` (centroid estimation over a set of robots already driven by something else), `tello_gazebo_driver` (DJI Tello formation control in Gazebo, via the `tello_msgs/TelloAction` service), `turtlebot_driver` (TurtleBot3 position control).
- **[uned_swarm_task](uned_swarm_task/README.md)**. ROS 2 package (`ament_python`). Coordination nodes: `centralized_formation_controller`, `system_identification`, `open_loop_signal`, `swarm_reconfiguration`.

> Internal packages are still named `uned_swarm_*` — the repo rename hasn't (yet) carried over to the ROS packages inside it, to avoid silently breaking existing references.

## Installation :book:
```
mkdir -p ~/multi_agent_ws/src
cd ~/multi_agent_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_multi_agent_ros_pkg.git
cd ..
colcon build --symlink-install --packages-select uned_swarm_config uned_swarm_driver uned_swarm_task
source install/setup.bash
```

## Usage 🔧
```
ros2 launch uned_swarm_config generic.launch.py config_file:=<experiment>.yaml
```
```
ros2 run uned_swarm_task centralized_formation_controller
```

See [uned_swarm_config/README.md](uned_swarm_config/README.md) for the experience `.yaml` schema and the current set of experiment config files in `resources/`.

## Authors ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Related publications :paperclip:
