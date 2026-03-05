# sar_control

ros2_control hardware interface for the SAR robot. Communicates with an Arduino Uno over USB serial to command
4 mecanum wheel velocities (mm/s) and read encoder feedback.

## Build

```bash
pixi run build-pkg sar_control
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

## Serial Protocol

| Direction | Format | Example |
|-----------|--------|---------|
| Pi → Arduino | `v:<fl>,<fr>,<rl>,<rr>\n` | `v:100,-50,100,-50\n` |
| Pi → Arduino | `e\n` (request encoders) | `e\n` |
| Arduino → Pi | `e:<fl>,<fr>,<rl>,<rr>\n` | `e:1234,-567,1234,-567\n` |
