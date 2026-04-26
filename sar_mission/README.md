# sar_mission

Tiny mission-level node that watches `/aruco/detection` (`sar_msgs/ArucoMsg`)
while frontier exploration is running and pauses exploration when a marker
is found by publishing on `explore/resume`.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_mission explore_and_detect.launch.py
```

## Notes

- Standalone state machine (`EXPLORING → MARKER_FOUND → FINISHED`) — does
  not include the exploration or perception stacks. Run them separately
  (e.g. via `sar_bringup`) for the full loop.
- Pause/resume control follows `m-explore-ros2`'s convention: publish
  `std_msgs/Bool` on `explore/resume` (`false` to pause, `true` to resume).
