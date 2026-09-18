^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fss_time_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.1 (2026-08-19 21:18)
------------------------
* chore: bump all packages to 0.1.1 for ROS release
* Contributors: Peixuan Shu

0.1.0 (2026-08-19 20:25)
------------------------
* feat: Add observed real time factor tracking and update SimClockStatus message
* feat: Introduce min_operation_walltime and update speed regulator logic for improved time management
* feat: Add debug message functionality to SimTimeBroker and UI for enhanced status reporting
* feat: Update speed regulator step and enhance SimClockStatus message with new request participant count
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
