# fss_sensing

This package provides a ROS 2 local point cloud simulator based on `marsim_render`. It renders a LiDAR point cloud from the current drone pose using a static PCD map.

The simulator supports the `fss_time` coordinated simulation clock and lock-step execution. The coordinated clock is enabled by default (`use_fss_sim_time:=true`), so the sensing timer advances with the global simulation time rather than wall time.

```bash
# Launch the local point cloud simulator with the default configuration.
ros2 launch fss_sensing fss_local_pointcloud_sim.launch.py

# Run with wall time instead of the FastSwarmSim coordinated simulation clock.
ros2 launch fss_sensing fss_local_pointcloud_sim.launch.py use_fss_sim_time:=false

# Run with a custom point cloud configuration file.
ros2 launch fss_sensing fss_local_pointcloud_sim.launch.py config_path:=/path/to/local_pointcloud_sim.yaml
```

## What it contains:

1. A local point cloud simulator node that subscribes to the drone pose or odometry and renders the point cloud visible from the drone using `marsim_render`.

2. A static global point cloud publisher. The `global_pc` topic uses reliable, transient-local QoS, so subscribers that start after the simulator still receive the map.

3. A configurable LiDAR model and map source. The default configuration is installed at `config/local_pointcloud_sim.yaml` and supports the sensing range, rate, field of view, angular resolution, downsampling resolution, input topics, and output topics.

4. `fss_time` lock-step simulation-time integration. Set `use_fss_sim_time:=false` to use the normal ROS clock (`/clock` when `use_sim_time` is enabled, otherwise wall time).

## Topics

The default configuration uses the following topics:

| Topic | Type | Description |
| --- | --- | --- |
| `mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | Input drone pose. Set `use_odom: true` to use `mavros/local_position/odom` instead. |
| `cloud_registered` | `sensor_msgs/msg/PointCloud2` | Local point cloud rendered at the configured sensing rate (10 Hz by default). |
| `global_pc` | `sensor_msgs/msg/PointCloud2` | Complete static map point cloud, published once at startup. |

Topic names, frame ID, map, and LiDAR parameters can be changed in `local_pointcloud_sim.yaml`.

## Trouble shooting

### Local point cloud is only published at 1 Hz

The default local point cloud rate is 10 Hz. It can also run at 2x real-time simulation speed. If `cloud_registered` is received at only about 1 Hz, the rendered point cloud is likely larger than the default DDS shared-memory segment or UDP receive buffer. Configure the active RMW implementation before launching ROS 2 nodes.

#### Fast DDS (`rmw_fastrtps_cpp`)

Fast DDS uses shared memory for local communication, but its default 256 KB segment is too small for large point clouds. Create `fastdds.xml` with a larger segment; 8 MB is a suitable starting point:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>shm_transport_8MB</transport_id>
        <type>SHM</type>
        <segment_size>8388608</segment_size>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>shm_transport_8MB</transport_id>
        </userTransports>
        <useBuiltinTransports>true</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

Then start ROS 2 with the profile enabled:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
# Required by older ROS 2 Humble releases; harmless to set as well.
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
ros2 daemon stop
```

Set `segment_size` according to the largest point cloud message. Every DDS participant (normally one per process) allocates a segment, so avoid setting it unnecessarily large.

#### Cyclone DDS (`rmw_cyclonedds_cpp`)

Cyclone DDS uses UDP by default. Large local point clouds can overflow the system UDP receive buffer, so increase it before launching the nodes:

```bash
sudo tee /etc/sysctl.d/60-cyclonedds.conf >/dev/null <<'EOF'
net.core.rmem_max=8388608
net.core.rmem_default=8388608
EOF
sudo sysctl --system

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop
```

The 8 MB values are a starting point and should be increased only when the largest point cloud requires it. Cyclone DDS shared-memory transport requires additional RouDi/mempool configuration and is not recommended here; use the UDP configuration above.

See [the large point cloud DDS configuration reference](https://blog.csdn.net/benchuspx/article/details/163828928) for background and further tuning considerations.
