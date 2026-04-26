# sar_control

`ros2_control` `SystemInterface` for the real SAR robot. Talks to an Arduino
over USB serial (Boost.Asio) to send 4 mecanum wheel velocities (rad/s) and
read encoder feedback. Loaded by `sar_description` when `use_sim:=false`.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_control control.launch.py                  # real Arduino
ros2 launch sar_control control.launch.py mock:=true       # no hardware
```

If the Arduino isn't on `/dev/arduClone`, pass `device:=/dev/ttyACM0` (or
the path printed by `readlink -e /dev/serial/by-id/*Arduino*`). The udev
rule under `scripts/` pins our Arduino clone to `/dev/arduClone`.

Before flashing firmware the serial port must be writable:

```bash
sudo chmod a+rw /dev/arduClone
# or, for any Arduino:
sudo chmod a+rw $(readlink -enq /dev/serial/by-id/*Arduino*)
```

## Notes

- The controller manager is unstable on macOS; for hardware-interface
  development use the standalone tester: `ros2 run sar_control test_hardware`.
- Wheel velocities round-trip in **rad/s** between the controller and the
  hardware interface; the firmware converts to PWM internally.
- `config/controllers.yaml` is the real-robot config. `config/controllers_sim.yaml`
  is the Gazebo variant (different wheel geometry, multipliers, tighter
  velocity limits, y-axis zeroed for diff-drive behavior).

## Serial protocol

| Direction | Format | Example |
| --- | --- | --- |
| Pi → Arduino | `v:<fl>,<fr>,<rl>,<rr>\n` | `v:100,-50,100,-50\n` |
| Pi → Arduino | `e\n` (request encoders) | `e\n` |
| Arduino → Pi | `e:<fl>,<fr>,<rl>,<rr>\n` | `e:1234,-567,1234,-567\n` |
