^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fss_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-09-18)
------------------

0.1.1 (2026-08-19 21:18)
------------------------
* chore: bump all packages to 0.1.1 for ROS release
* Contributors: Peixuan Shu

0.1.0 (2026-08-19 20:25)
------------------------
* Add README.md for fss_bringup package with launch instructions
  - Introduced a new README.md file for the fss_bringup package.
  - Documented integrated FastSwarmSim launch files for PX4 and perfect-drone simulations.
  - Provided example commands for launching single and multi-drone setups with LiDAR and RViz.
  - Explained configuration options for time coordination, sensing, and RViz visualization.
* Refactor code structure for improved readability and maintainability
* feat: update RViz configurations, enhance point cloud rendering, and improve performance settings
* feat: enhance launch files and visualizer node for improved performance and modularity
* feat: update RViz configuration for PX4 drone with new point cloud topics and visualization settings
* Add RViz configuration for PX4 drone with LIDAR visualization
  - Created a new RViz configuration file `sim_px4_drone_lidar_single.rviz`
  - Configured panels for Displays, Selection, Tool Properties, and Views
  - Set up visualization manager with grid, axes, odometry, robot model, markers, and path displays
  - Defined global options including background color and fixed frame
  - Included tools for interaction, camera movement, and goal setting
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
* Refactor fss_time to integrate HELICS for distributed time coordination
  - Removed existing time transport implementation files: time_transport.cpp, time_transport_serialization.cpp, test_distributed_time_core.cpp, test_ecal_time_transport.cpp, test_time_transport_serialization.cpp.
  - Introduced HELICS-based time coordination with new files: helics_time_coordinator.hpp, helics_time_coordinator.cpp, helics_broker_node.cpp.
  - Added verification script for HELICS integration: verify_fss_time_helics.sh.
  - Created documentation for HELICS time coordination: fss_time_helics.md.
  - Implemented unit tests for HELICS time coordinator functionality: test_helics_time_coordinator.cpp.
* Rename FastSwarmSim packages to fss prefix
* Contributors: Peixuan Shu, peixuan shu
