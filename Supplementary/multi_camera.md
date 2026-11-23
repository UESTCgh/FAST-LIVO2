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
