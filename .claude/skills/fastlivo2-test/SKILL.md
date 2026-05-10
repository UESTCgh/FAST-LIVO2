---
name: fastlivo2-test
description: Run FAST-LIVO2 SLAM tests with rosbag playback and evaluate degeneration/localization performance. Covers super1/super2 configs and the 0326 degeneration dataset.
version: 2.0.0
---

# FAST-LIVO2 Test Skill

## Workspace

- **WS**: `/home/gh/fastlivo2_ws`
- **Source**: `source /home/gh/fastlivo2_ws/devel/setup.bash`
- **Always use bash subshells**: `bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && <cmd>'`
- **Log/result**: `src/FAST-LIVO2/Log/result/<seq_name>.txt` (TUM format)

## Test Scenarios

### Scenario A — Degeneration Test (0326 bag, super1)

Tests algorithm robustness in degenerate geometries (corridor/tunnel/featureless).

```
bag:    /home/gh/data/0326_guandao/2026-03-26-15-31-29.bag
topics: /livox/imu /livox/lidar /camera/color/image_raw
rate:   2x
driver: roslaunch livox_ros_driver2 msg_MID360_sync.launch
launch: roslaunch fast_livo mapping_super1.launch rviz:=false
```

**Launch order** (each in separate terminal / background process):
1. `roscore`
2. `roslaunch livox_ros_driver2 msg_MID360_sync.launch` — wait 3s
3. `roslaunch fast_livo mapping_super1.launch rviz:=false` — wait 8s
4. `rosbag play /home/gh/data/0326_guandao/2026-03-26-15-31-29.bag --topics /livox/imu /livox/lidar /camera/color/image_raw -r 2 --clock`

### Scenario B — guandao2 (super2)

```
bag:    /home/gh/fastlivo2_ws/dataset/guandao2.bag
launch: roslaunch fast_livo mapping_super2.launch rviz:=false
```

**Launch order**:
1. `roscore`
2. `roslaunch fast_livo mapping_super2.launch rviz:=false` — wait 8s
3. `rosbag play /home/gh/fastlivo2_ws/dataset/guandao2.bag --clock`

## Enable Pose Output (before each run)

Edit the relevant config YAML (`config/super1_d435i.yaml` or `config/super2_d435i.yaml`):

```yaml
evo:
  seq_name: "<test_name>"   # e.g. "degen_0326_super1"
  pose_output_en: true
```

Restore `pose_output_en: false` after the run.

## Execution (bash subshells)

```bash
# Terminal 1 — roscore
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && roscore' &

# Terminal 2 — driver (Scenario A only)
sleep 3 && bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360_sync.launch' &

# Terminal 3 — FAST-LIVO2
sleep 6 && bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && roslaunch fast_livo mapping_super1.launch rviz:=false' 2>&1 | tee /tmp/fastlivo2.log &

# Terminal 4 — bag (after FAST-LIVO2 init)
sleep 14 && bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && rosbag play /home/gh/data/0326_guandao/2026-03-26-15-31-29.bag --topics /livox/imu /livox/lidar /camera/color/image_raw -r 2 --clock'
```

After bag finishes, wait 5s then: `killall -INT roslaunch`

## Degeneration Analysis

After run, check for degeneration signatures in `/tmp/fastlivo2.log`:

```bash
# Count VIO skips (degeneration indicator)
grep -c "Skip VIO Update" /tmp/fastlivo2.log

# Check LIO-only fallback
grep "ONLY_LIO\|lio_only\|vio_ok=0" /tmp/fastlivo2.log | tail -20

# Trajectory divergence check
TRAJ=src/FAST-LIVO2/Log/result/<seq_name>.txt
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && evo_traj tum '"$TRAJ"' --full_check'

# Find divergence frame (position > 500m from origin)
awk '{x=$2;y=$3;z=$4; d=sqrt(x*x+y*y+z*z); if(d>500) {print "line "NR" t="$1" dist="d; exit}}' "$TRAJ"
```

**Degeneration metrics to report**:
- VIO skip count / total frames (skip rate %)
- Frame where divergence starts (if any)
- Path length vs expected
- Max instantaneous velocity spike

## Evaluate Accuracy

```bash
# Trajectory stats (no GT)
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && evo_traj tum <traj.txt> --full_check --plot_mode xy --save_plot /tmp/traj.pdf'

# With ground truth
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && evo_ape tum <gt.txt> <traj.txt> --align --correct_scale'
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && evo_rpe tum <gt.txt> <traj.txt> --align --delta 1 --delta_unit m'
```

## Cleanup

```bash
killall -q roslaunch roscore rosmaster fastlivo_mapping rosbag 2>/dev/null; true
```

## Known Issues

- **0326 bag at 2x rate**: IMU pre-integration may accumulate error faster; if divergence occurs immediately, retry at `-r 1`.
- **livox_ros_driver2 required for Scenario A**: The `msg_MID360_sync.launch` sets up time-sync parameters needed by `mid360.yaml`; skipping it causes topic mismatch.
- **guandao1 divergence at ~75s**: Known issue, use guandao2 for super2 testing.
