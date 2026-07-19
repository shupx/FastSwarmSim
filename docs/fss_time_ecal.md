# fss_time eCAL Transport

`fss_time` uses eCAL as the only transport for distributed logical-time messages.
The time algorithm still lives in `fss_time`; eCAL only carries `TimeIntent` and
`TimeControl` payloads.

## Transport

`ecal` uses eCAL pub/sub for `/fss/time_intent` and `/fss/time_control`.
`fss_time` requires the eCAL SDK at build time.

On Ubuntu, install eCAL and the Protobuf development files before building:

```bash
sudo add-apt-repository ppa:ecal/ecal-latest
sudo apt update
sudo apt install -y ecal libprotobuf-dev protobuf-compiler
```

## Local Host

For multiple processes on one machine, eCAL runs in `local` communication mode
by default and prefers the SHM layer over UDP and TCP. No network setting is
required.

```bash
ros2 launch fss_bringup distributed_clock.launch.py
ros2 launch fss_bringup perfect_swarm.launch.py num_drones:=5
```

## LAN / UDP Multicast

For multiple machines, enable eCAL `network` communication mode on every host.
eCAL 6 loads `ecal.yaml` in this order on Ubuntu:

1. `$ECAL_DATA/ecal.yaml`
2. `$PWD/ecal.yaml`
3. `$HOME/.ecal/ecal.yaml`
4. `/etc/ecal/ecal.yaml`

The minimal setting is:

```yaml
communication_mode: "network"
```

In eCAL 6, network mode defaults to UDP before TCP. The stock UDP multicast
group is `239.0.0.1` and the payload port is `14002`.

After enabling it, run the clock bridge on one machine and simulation
participants on either the same or other LAN machines:

```bash
ros2 launch fss_bringup distributed_clock.launch.py max_speed_ratio:=10.0
ros2 launch fss_bringup perfect_drone.launch.py namespace:=uav1
```

Make sure the machines are in the same multicast-reachable network and the
firewall allows eCAL UDP multicast traffic.

## Verification

```bash
scripts/verify_fss_time_ecal.sh
```

The script builds `fss_time_interfaces` and `fss_time`, then runs:

- `test_distributed_time_core`
- `test_time_transport_serialization`
- `test_ecal_time_transport`

The eCAL test validates both same-process delivery and local interprocess
delivery. When the eCAL SDK is detected, the script also reruns the eCAL test
with a temporary `communication_mode: "network"` config. If `strace` is
available, it checks that eCAL joins the default UDP multicast group
`239.0.0.1`.

True end-to-end LAN verification still requires two machines with
`communication_mode: "network"` in their eCAL configuration.
