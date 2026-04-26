# sar_perception

ArUco marker detector. Subscribes to a camera image topic, detects markers
with OpenCV, and publishes detection + range info on `/aruco/detection`
(`sar_msgs/ArucoMsg`). Optionally also brings up the RealSense D435i driver.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_perception perception.launch.py                       # image-only
ros2 launch sar_perception perception.launch.py use_realsense:=true   # + RS driver
ros2 launch sar_perception perception.launch.py marker_size:=0.30 \
    aruco_dict:=DICT_6X6_1000 image_topic:=/oak/rgb/color
```

## Notes

- `image_topic` defaults to `/camera/color/image_raw` when `use_realsense`
  is true, otherwise `/oak/rgb/color`.
- `marker_size` is in meters; range estimation uses the calibrated
  `camera_fx/fy/cx/cy` args (defaults match a 1280×720 RealSense color
  intrinsic — pass overrides for any other camera).
- `ArucoMsg` carries `bool detected` + `float32 distance` (meters; 0 when
  not detected). Pose is not published yet.
