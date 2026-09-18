^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fss_time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-09-18)
------------------
* docs: add platform support and release changelog automation
* Contributors: Peixuan Shu

0.1.1 (2026-08-19 21:18)
------------------------
* chore: bump all packages to 0.1.1 for ROS release
* test: remove outdated fss_time tests that hang waiting for ROS clock (`#598 <https://github.com/shupx/FastSwarmSim/issues/598>`_, `#615 <https://github.com/shupx/FastSwarmSim/issues/615>`_)
* Contributors: Peixuan Shu

0.1.0 (2026-08-19 20:25)
------------------------
* feat: update README.md for fss_time package with detailed ZeroMQ coordination explanation and usage instructions
* feat: enhance time coordination logic with new advancement checks and update real time request handling
* feat: update CMakeLists.txt for fss_px4_sim and fss_time to include Release build type and optimize compiler flags
  feat: improve warning messages in MulticopterAttitudeControl and MulticopterPositionControl for loop period thresholds
* feat: update RViz configurations, enhance point cloud rendering, and improve performance settings
* fix: revert version and description changes in package.xml for consistency
* Enhance launch files with detailed descriptions and environment settings
  - Added descriptions to launch arguments in `perfect_mavros_drone_swarm.launch.py`, `px4_rotor_sim_multi.launch.py`, `px4_rotor_sim_single.launch.py`, `sensing.launch.py`, `parent_time_coordinator_example.launch.py`, `time_coordinator.launch.py`, `executor_time_test_case.launch.py`, `multi_node_sim_time_test_case.launch.py`, and `sleep_rate_test_case.launch.py` for better clarity.
  - Introduced `SetEnvironmentVariable` to force colorized output in logs across multiple launch files.
  - Updated launch arguments to include choices where applicable, improving validation and user experience.
  - Refactored the structure of launch files for consistency and readability.
* feat: add clear zombie participants functionality and related UI updates, improve auto unregister success rate
* feat: refactor time coordinator for improved message handling and add UI parameter for always on top
* feat: scope included launches to isolate parameters and prevent leakage
* feat: enhance launch files and visualization for drone simulation
* feat: add configuration for large-scale fastdds participant limits
* feat: remove dependencies of agent_id for px4 param and uorb messges.
* feat: add test for TimeCoordinator to allow empty pub endpoint and verify advertisement
* feat: increase speed regulator step to improve real-time factor performance
* feat: implement asynchronous task handling in TimeCoordinator for improved performance
* feat: add simulation time to status updates in CoordinatorBridge and MainWindow
* feat: update minimum operation walltime constant and optimize clock update logic
* feat: improve coordinator performance by cancling replying OK for announce and using Debug msg enum.
* feat: enhance clock subscription with QoS settings and remove unused sim_time variable
* Add MAVROS setpoint raw and system status plugins
  - Implemented SetpointRawPlugin to handle MAVLink position and attitude targets, transforming coordinates between NED and ENU frames.
  - Added SystemStatusPlugin to publish various system status messages including state, extended state, and battery status.
  - Updated thread_time_participant to remove follows_real_time parameter and adjusted related functionality in ZeroMqTimeParticipantBackend.
  - Refactored registration logic for time participants to streamline real-time settings management.
* feat: finish migration of px4_rotor_sim to fss_px4_sim
* feat: add real-time following feature for time participants and enhance coordinator communication
* feat: add detailed documentation for parent time coordinator launch file
* feat: add parent time coordinator launch file and enhance UI namespace handling
* feat: add TCP endpoint parsing and replacement functions for improved socket handling
* feat: add UUID generation and enhance coordinator communication in time management
* feat: enhance time coordination with parent-child relationship and grant messaging
* fix: increase clock status timer interval and ensure minimum real time timer period
* feat: add real time floor for time broker, and rename time broker as time coordinator
* feat: Update endpoint references from sim_time_broker to fss_time_broker across multiple files for consistency
* fix: force use_sim_time=true if use_fss_sim_time=true in launch.py to avoid unset clock ROS time at node start and wrong wallclock time
* feat: Simplify ensure_use_sim_time_enabled by using declare_or_get_parameter and update spin call in FssRateTestNode
* feat: Replace rclcpp::spin with fss_time::spin for improved node handling
* feat: Refactor announce_next_safe_time to simplify logic and improve clarity
* feat: Update rate and thread time participant classes for improved time management and sleep functionality
* feat: Add testing nodes and utilities for fss_time functionality, including rate and timer sleep management
* feat: Enhance timer functionality by adjusting timer period based on index
* feat: Add detailed documentation for time management classes and methods
* feat: Add Rate and Sleep classes with functionality for time management and sleeping mechanisms
* refactor: Rename parameter retrieval function to include locking mechanism for thread safety
* fix:  fix the deadlock of the MultiThreadedExecutor by replacing the lock with try_to_lock
* feat: Add observed real time factor tracking and update SimClockStatus message
* feat: Enhance MultiThreadedExecutor run logic with improved work retrieval and time management
* feat: Improve SingleThreadedExecutor spin logic with enhanced time management and callback execution
* feat: Enhance SimTimeBroker UI with target RTF input and update scaling logic
* feat: Introduce min_operation_walltime and update speed regulator logic for improved time management
* feat: Update speed regulator step parameter in launch file for optimal performance and CPU usage
* feat: Add speed regulator step parameter and enhance timer functionality in executors
* feat: Add debug message functionality to SimTimeBroker and UI for enhanced status reporting
* refactor: Simplify clock update logic in SimTimeBroker and adjust launch configuration for executor nodes
* feat: Update SingleThreadedExecutor and MultiThreadedExecutor documentation for clarity; add logo icon to SimTimeBroker UI
* feat: Refactor SingleThreadedExecutor and MultiThreadedExecutor to inherit from fss_time::Executor for enhanced time support
* feat: Add executor time test node and launch configuration for testing SingleThreadedExecutor and MultiThreadedExecutor
* feat: Add spin function for SingleThreadedExecutor and implement tests for namespace spin functionality
* refactor: Rename detail namespace to fss_time_tools and update related function calls for clarity
* feat: Implement SingleThreadedExecutor and MultiThreadedExecutor classes with corresponding CMake and test updates
* feat: Add UI components for Sim Time Broker and implement layout adjustments
* feat: Add clock status timer and implement on_clock_status_tick for periodic status publishing
* feat: Update speed regulator step and enhance SimClockStatus message with new request participant count
* feat: Enhance safe time handling in thread_time_participant with infinite time support
* refactor: Rename speed regulator parameter and update related logic for clarity
* fix: Update sim_time_broker_ui and launch files for improved time handling and control requests
* feat: Implement thread-safe last safe time retrieval and clamping in thread_time_participant
* feat: Add conditional compilation for testing support in thread_time_participant
* trans to zeromq fss_time broker and participants
* feat: Enhance simulation time broker with participant query period and UI updates
* feat: Add multi-node simulation test cases and update launch configurations
* Refactor and add launch files for drone simulation and sensing
  - Removed the main.cpp file from marsim_render/test directory.
  - Added perfect_drone.launch.py to launch a single drone simulation with configurable parameters.
  - Introduced perfect_swarm.launch.py to launch multiple drone simulations based on user-defined count.
  - Created sensing.launch.py for launching the local point cloud simulation with configurable parameters.
  - Added time.yaml configuration for the time broker settings.
  - Implemented distributed_clock.launch.py to manage distributed clock synchronization with configurable parameters.
* feat: Update launch and configuration parameters for broker integration; add UI for sim time control
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
* fix: Update .gitignore and CMakeLists.txt for external dependencies; add fmt, spdlog, and ZeroMQ system support
* Refactor fss_time to integrate HELICS for distributed time coordination
  - Removed existing time transport implementation files: time_transport.cpp, time_transport_serialization.cpp, test_distributed_time_core.cpp, test_ecal_time_transport.cpp, test_time_transport_serialization.cpp.
  - Introduced HELICS-based time coordination with new files: helics_time_coordinator.hpp, helics_time_coordinator.cpp, helics_broker_node.cpp.
  - Added verification script for HELICS integration: verify_fss_time_helics.sh.
  - Created documentation for HELICS time coordination: fss_time_helics.md.
  - Implemented unit tests for HELICS time coordinator functionality: test_helics_time_coordinator.cpp.
* add HELICS thirdparty submodule
* feat: Integrate eCAL transport for fss_time
  - Added eCAL as a transport option for distributed logical-time messages in fss_time.
  - Implemented TimeTransport interface with eCAL-specific functionality.
  - Created serialization and deserialization methods for TimeIntent and TimeControl messages.
  - Updated TimeParticipant to utilize eCAL for publishing and subscribing to time messages.
  - Added tests for eCAL transport functionality, ensuring interprocess and same-process communication.
  - Documented eCAL transport setup and usage in fss_time_ecal.md.
* Rename FastSwarmSim packages to fss prefix
* Contributors: Peixuan Shu, peixuan shu
