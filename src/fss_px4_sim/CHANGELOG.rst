^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fss_px4_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix(fss-px4-sim): stop motion after key repeat ends
* docs(fss-px4-sim): fix troubleshooting code blocks
* fix(fss-px4-sim): restore ROS namespace-based MAVROS names
* fix(fss-px4-sim): use explicit mavros topics and fix docs formatting
* chore(fss-px4-sim): remove keyboard control test
* feat(fss-px4-sim): add ROS 2 keyboard control
* fix(fss-px4-sim): disarm keyboard controller on exit
* fix(fss-px4-sim): handle keyboard controller shutdown
* docs(fss-px4-sim): fix keyboard screenshot path
* feat(fss-px4-sim): add ROS 2 keyboard control
* Contributors: Peixuan Shu

0.1.1 (2026-08-19 21:18)
------------------------
* chore: bump all packages to 0.1.1 for ROS release
* Contributors: Peixuan Shu

0.1.0 (2026-08-19 20:25)
------------------------
* feat: add README files for fss_px4_sim and fss_sensing packages with usage instructions and features
* feat: enhance time coordination logic with new advancement checks and update real time request handling
* feat: update CMakeLists.txt for fss_px4_sim and fss_time to include Release build type and optimize compiler flags
  feat: improve warning messages in MulticopterAttitudeControl and MulticopterPositionControl for loop period thresholds
* feat: enhance launch files and visualizer node for improved performance and modularity
* feat: update MAVROS node executor to MultiThreadedExecutor and add test launch file for multi-drone simulation
* feat: enhance intra-process communication for MAVROS components and update publisher options
* feat: add rclcpp_components support and refactor drone simulation nodes for improved modularity
* Add RViz configuration for PX4 drone with LIDAR visualization
  - Created a new RViz configuration file `sim_px4_drone_lidar_single.rviz`
  - Configured panels for Displays, Selection, Tool Properties, and Views
  - Set up visualization manager with grid, axes, odometry, robot model, markers, and path displays
  - Defined global options including background color and fixed frame
  - Included tools for interaction, camera movement, and goal setting
* refactor: remove fss_time_coordinator_endpoint from launch files and update dependencies
* feat: add startup batch size and delay parameters for drone launch configuration
* fix: update thrust scaling initialization and add error handling in setpoint_raw module
  refactor: improve QoS settings for publishers in sys_status module
* Enhance launch files with detailed descriptions and environment settings
  - Added descriptions to launch arguments in `perfect_mavros_drone_swarm.launch.py`, `px4_rotor_sim_multi.launch.py`, `px4_rotor_sim_single.launch.py`, `sensing.launch.py`, `parent_time_coordinator_example.launch.py`, `time_coordinator.launch.py`, `executor_time_test_case.launch.py`, `multi_node_sim_time_test_case.launch.py`, and `sleep_rate_test_case.launch.py` for better clarity.
  - Introduced `SetEnvironmentVariable` to force colorized output in logs across multiple launch files.
  - Updated launch arguments to include choices where applicable, improving validation and user experience.
  - Refactored the structure of launch files for consistency and readability.
* Refactor MAVLink transport to support direct ROS interface and UDP
  - Updated GlobalPosition, Imu, LocalPosition, SetpointRaw, SysStatus modules to accept MavrosLite instead of Core.
  - Introduced MavlinkUdpTransport class for handling UDP communication.
  - Modified MAVLINK class to support both UDP and direct ROS transport methods.
  - Enhanced PX4SITL to configure MAVLink transport type via parameters.
  - Implemented message exchange tests for direct transport without sockets.
  - Updated px4_rotor_sim_node to integrate MAVROS Lite for direct ROS communication.
* feat: add subscription count checks before publishing joint states and markers
* Add mavros Lite modules for PX4 simulation
  - Implement core functionality for MAVLink communication in `core.cpp`.
  - Create `global_position.cpp` to handle global positioning data and publish relevant messages.
  - Add `imu.cpp` for processing IMU data and publishing sensor readings.
  - Implement `local_position.cpp` to manage local position data and publish odometry and pose information.
  - Create `main.cpp` to initialize and run the MAVLink Lite node.
  - Add `setpoint_raw.cpp` for handling setpoint commands and publishing target messages.
  - Implement `sys_status.cpp` to manage system status, heartbeat, and battery state publishing.
* refactor: clean up comments and remove unnecessary annotations in MAVLINK module
* feat: add namespace to mavros node in launch file for improved organization
* feat: modify launch files for single and swarm drone simulation with parameter isolation
* feat: rename mavros_px4_quadrotor_sim_node to px4_rotor_sim_node and update launch files accordingly
* feat: scope included launches to isolate parameters and prevent leakage
* Refactor drone visualizer node for improved GPS projection and marker handling
  - Integrated geo library for map projection, replacing manual equirectangular calculations with MapProjection class.
  - Updated GPS position calculations to use projected coordinates.
  - Enhanced marker properties including frame ID, scale, and color for better visualization.
  - Adjusted path history management to limit size based on history duration.
  - Fixed marker publishing logic to ensure proper frame locking and timestamp handling.
* feat: enhance launch files and visualization for drone simulation
* improve fss_px4_sim launch and visualize
* feat: add launch arguments for PX4 SITL configuration in simulation
* Refactor PX4 simulation launch files and MAVLink communication
  - Updated `px4_rotor_sim_multi.launch.py` to include a new argument for `drones_per_row` and refactored the drone node creation to use `IncludeLaunchDescription` for better modularity.
  - Removed the deprecated `px4_rotor_sim_single_no_prefix.launch.py` file.
  - Enhanced `px4_rotor_sim_single.launch.py` to support additional launch arguments for MAVLink system and component IDs, and added conditions for enabling MAVROS and visualizer nodes.
  - Modified MAVLink communication in `mavlink_main.cpp`, `mavlink_receiver.cpp`, and related files to support dynamic system and component IDs, improving the handling of multiple PX4 instances.
  - Updated MAVLink message encoding to include the correct system and component IDs across various message streams.
  - Added tests in `test_mavlink_udp_bridge.cpp` to verify the correct encoding of MAVLink packets with dynamic IDs.
* feat: remove dependencies of agent_id for px4 param and uorb messges.
* Refactor MAVLink message handling and introduce UDP communication
  - Removed mavlink_msg_list.cpp and mavlink_msg_list.hpp as they are no longer needed for storing MAVLink messages.
  - Updated MavlinkReceiver and MavlinkStreamer to handle MAVLink messages directly via UDP.
  - Introduced MavlinkSender as a function type for sending MAVLink messages.
  - Refactored MAVLink stream classes to use the new sender mechanism for encoding and sending messages.
  - Implemented a new MAVLINK class to manage UDP communication, including sending and receiving MAVLink messages.
  - Added unit tests for MAVLink UDP communication to ensure proper message handling and acknowledgment.
* Add MAVROS setpoint raw and system status plugins
  - Implemented SetpointRawPlugin to handle MAVLink position and attitude targets, transforming coordinates between NED and ENU frames.
  - Added SystemStatusPlugin to publish various system status messages including state, extended state, and battery status.
  - Updated thread_time_participant to remove follows_real_time parameter and adjusted related functionality in ZeroMqTimeParticipantBackend.
  - Refactored registration logic for time participants to streamline real-time settings management.
* feat: finish migration of px4_rotor_sim to fss_px4_sim
* feat: add real time floor for time broker, and rename time broker as time coordinator
* trans to zeromq fss_time broker and participants
* Refactor and add launch files for drone simulation and sensing
  - Removed the main.cpp file from marsim_render/test directory.
  - Added perfect_drone.launch.py to launch a single drone simulation with configurable parameters.
  - Introduced perfect_swarm.launch.py to launch multiple drone simulations based on user-defined count.
  - Created sensing.launch.py for launching the local point cloud simulation with configurable parameters.
  - Added time.yaml configuration for the time broker settings.
  - Implemented distributed_clock.launch.py to manage distributed clock synchronization with configurable parameters.
* Refactor fss_time interfaces and implement HELICS-based time coordination
  - Removed obsolete message and service definitions: TimeIntent, TimeControl, TimeRequest, ClientRegister, ClientUnregister.
  - Introduced SimClockStatus message to encapsulate simulation clock state.
  - Updated SimClockControl service to include a message field for feedback.
  - Implemented HelicsBrokerBackend and HelicsThreadParticipantBackend for managing HELICS broker and federate interactions.
  - Created SimTimeBroker to manage simulation time and publish status updates.
  - Developed thread_time_participant for thread-specific time management.
  - Added ROS nodes for clock publishing and broker management.
  - Included tests for time participant behavior and broker functionality.
  - Documented the fss_time package and its dependencies.
* Rename FastSwarmSim packages to fss prefix
* Contributors: Peixuan Shu, peixuan shu
