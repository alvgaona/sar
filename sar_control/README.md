# sar_control

ros2_control hardware interface for the SAR robot. Communicates with an Arduino Uno/Mega over USB serial to command
4 mecanum wheel velocities (rad/s) and read encoder feedback.

## Build

```bash
colcon build --symlink-install --packages-select sar_control
source install/setup.bash
```
or
```bash
pixi run build-pkg sar_control
```

## Arduino configuration (Linux / Raspberry Pi)

To get the serial device name of your board, use this command:

```bash
readlink -e /dev/serial/by-id/*Arduino*
```

If you use the same Arduino clone as us, there's a udev rule in the repositorie's script folder. This makes it so the board is always accessible as `/dev/arduClone`.

If it's different than `/dev/arduClone`, append `device:=${yourDeviceNameHere}` to the launch call.

Before flashing the software onto the board, it needs to be writable, so use this command:

```bash
sudo chmod a+rw $(readlink -enq /dev/serial/by-id/*Arduino*)
```

## Test (macOS)

The controller manager crashes on macOS. Use the standalone test instead:

```bash
ros2 run sar_control test_hardware
```

## Launch (Linux / Raspberry Pi)

```bash
ros2 launch sar_control control.launch.py
```

To run in mock mode (no Arduino needed):

```bash
ros2 launch sar_control control.launch.py mock:=true
```

## Unit Conversion

The mecanum drive controller outputs and receives wheel velocities in rad/s.

## Serial Protocol

| Direction | Format | Example |
|-----------|--------|---------|
| Pi → Arduino | `v:<fl>,<fr>,<rl>,<rr>\n` | `v:100,-50,100,-50\n` |
| Pi → Arduino | `e\n` (request encoders) | `e\n` |
| Arduino → Pi | `e:<fl>,<fr>,<rl>,<rr>\n` | `e:1234,-567,1234,-567\n` |
