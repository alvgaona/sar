# sar_msgs

Custom ROS 2 messages used across the SAR project.

Workspace-level setup is in the [root README](../README.md).

## Messages

- **`ArucoMsg`** — `bool detected` + `float32 distance` (meters; 0 when no
  marker). Published by `sar_perception` on `/aruco/detection` and consumed
  by `sar_mission`.
