# Multi-camera BIEVR-LIVO

The separate Gamma launch accepts `camera_mode:=left`, `right`, or `both`. Start the node and play the data in separate ROS Noetic terminals.

```bash
roslaunch fast_livo mapping_gamma_multicam.launch camera_mode:=both rviz:=false \
  save_map:=true seq_name:=shield4_multicam map_dir:="$HOME/shield4_multicam_map"

rosbag play ~/data/GEODE/Shield_tunnel4_gamma.bag \
  --topics /livox/lidar /livox/imu /left_camera/image/compressed /right_camera/image/compressed
```

`multicam_config` selects a YAML file. Each entry under `multi_camera/cameras` names a camera namespace with the topic, raw or compressed transport, camera model, intrinsics, time offset, and LiDAR-to-camera calibration. `visual_feedback:=false` lets the images build the visual and RGB maps while publishing LiDAR–IMU poses. The standard `mapping_gamma.launch` remains available separately.

Each complete LiDAR scan is registered by BIEVR and publishes one pose stamped at the scan end. The camera queue picks the nearest timestamp within the configured association window for each view. Missing images expire after a bounded wait. Visual projection uses the camera's own time with a short motion compensation from the registered scan-end pose. The default 8 ms window is for the synchronized Gamma rig.

The cameras maintain separate image patches and exposure estimates. A view must pass the tracking gates and lower the photometric cost on the same support. The views share the scan's LiDAR prior. Consistent pose corrections are averaged according to their quality, with capped translation and rotation and reduced translation along the weak LiDAR direction. The feedback changes the pose while the LiDAR filter retains its inertial state and covariance. The cameras update their maps at the final shared pose. This uses independent visual maps in a common world frame.

In the Gamma right-only mode, `right_feedback_gain` defaults to 0.12. This reduced the right-only correction through the Shield4 turnaround. Left-only and dual-camera modes take the feedback gain from the YAML.

After a clean shutdown with map export enabled, `visual_landmarks.pcd` contains landmarks with normals and `all_downsampled_points.pcd` contains colorized XYZRGB points. `/cloud_registered_360` exposes the complete registered LiDAR cloud for scans with or without images.

## Sensor-time fusion

The event launch runs BIEVR at each complete LiDAR scan's end time. Camera observations are applied in corrected sensor timestamp order as the IMU propagates to the end of that scan. Views with exactly the same timestamp use a common prior. Each view keeps the earliest observation within the configured `duplicate_window_s` (20 ms by default). Exact timestamp ties use the smaller sequence number and then the decoded image bytes. A bounded arrival wait gives out-of-order callbacks time to arrive. Observations arriving after the interval was processed are dropped.

The latest accepted LiDAR scan provides fixed world-frame depth and plane normals for the next camera observations. The IMU-only motion history deskews the current scan, and the shared filter carries accepted visual pose corrections forward to its LiDAR update. Color projection and landmark insertion use the camera-event pose.

```bash
roslaunch fast_livo mapping_gamma_multicam_event.launch camera_mode:=both rviz:=false \
  save_map:=true map_dir:="$HOME/shield4_event_map"

roslaunch fast_livo mapping_retail_multicam_event.launch rviz:=false \
  save_map:=true map_dir:="$HOME/retail_event_map"

rosbag play ~/data/dataset/fastlivo2/Retail_Street.bag \
  --topics /livox/lidar /livox/imu /left_camera/image
```

Set `robust_vision:=false` for a timing-only run. The Gamma dual-camera robust launch defaults to a feedback gain of 0.18 and a weak LiDAR direction weight of 0.10. Override them with `robust_feedback_gain` and `robust_axial_weight`. With the default robust setting, Huber weighting uses the RMS residual of each patch, and coverage is measured over a 4×4 grid. The translation update is damped along low-information directions after marginalizing rotation and exposure. The LiDAR weak-direction limit also applies, and the final pose step must reduce the photometric cost on the original set of patches. Set `visual_feedback:=false` for visual-map and color export with the pure LiDAR–IMU trajectory.

With `scan_diagnostics:=true`, the mapper logs the native scan start and end timestamps, point count, and a signature of the ordered point fields before deskew. This makes the scan input comparable across camera drop, swap and playback tests.
