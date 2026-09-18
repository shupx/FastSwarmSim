^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fss_sensing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* docs: add platform support and release changelog automation
* Contributors: Peixuan Shu

0.1.1 (2026-08-19 21:18)
------------------------
* chore: bump all packages to 0.1.1 for ROS release
* Contributors: Peixuan Shu

0.1.0 (2026-08-19 20:25)
------------------------
* chore: vendor marsim_render into repo (remove gitee submodule) for ROS release
* feat: add README files for fss_px4_sim and fss_sensing packages with usage instructions and features
* feat: enhance time coordination logic with new advancement checks and update real time request handling
* feat: switch to ROS time clock for local point cloud timer and remove timing debug logs
* feat: update RViz configurations, enhance point cloud rendering, and improve performance settings
* feat: add rclcpp_components support and refactor drone simulation nodes for improved modularity
* refactor: remove fss_time_coordinator_endpoint from launch files and update dependencies
* Enhance launch files with detailed descriptions and environment settings
  - Added descriptions to launch arguments in `perfect_mavros_drone_swarm.launch.py`, `px4_rotor_sim_multi.launch.py`, `px4_rotor_sim_single.launch.py`, `sensing.launch.py`, `parent_time_coordinator_example.launch.py`, `time_coordinator.launch.py`, `executor_time_test_case.launch.py`, `multi_node_sim_time_test_case.launch.py`, and `sleep_rate_test_case.launch.py` for better clarity.
  - Introduced `SetEnvironmentVariable` to force colorized output in logs across multiple launch files.
  - Updated launch arguments to include choices where applicable, improving validation and user experience.
  - Refactored the structure of launch files for consistency and readability.
* Refactor and add launch files for drone simulation and sensing
  - Removed the main.cpp file from marsim_render/test directory.
  - Added perfect_drone.launch.py to launch a single drone simulation with configurable parameters.
  - Introduced perfect_swarm.launch.py to launch multiple drone simulations based on user-defined count.
  - Created sensing.launch.py for launching the local point cloud simulation with configurable parameters.
  - Added time.yaml configuration for the time broker settings.
  - Implemented distributed_clock.launch.py to manage distributed clock synchronization with configurable parameters.
* add: Include marsim_render submodule for enhanced sensing capabilities
* Rename FastSwarmSim packages to fss prefix
* Contributors: Peixuan Shu, peixuan shu
