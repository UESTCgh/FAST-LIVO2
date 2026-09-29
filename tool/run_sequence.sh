#!/usr/bin/env bash
# =============================================================================
# run_sequence.sh — Run FAST-LIVO2 on a rosbag and collect the TUM trajectory.
#
# Runs fastlivo_mapping directly with `rosrun` (NOT roslaunch), so it never
# conflicts with an already-running roslaunch/rviz session. Assumes roscore is
# up and, for compressed-image bags, an image_transport `republish` node is
# already publishing the raw image topic (the mapping_geode_*.launch files
# start one with respawn=true — leave it running between sequences).
#
# Usage:
#   run_sequence.sh <config.yaml> <camera.yaml> <bag> <seq_name> <out_dir> [bag_rate] [bag_duration_s]
#
#   bag_duration_s  play only the first N seconds of the bag (rosbag --duration);
#                   omit or 0 for the whole bag.
#
# Environment overrides (to run a different build, e.g. an upstream baseline
# checked out as a git worktree in its own catkin workspace):
#   FASTLIVO_WS       catkin workspace to source   (default: this package's ws)
#   FASTLIVO_PKG_DIR  package dir whose Log/result/ the binary writes to
#                     (= the ROOT_DIR baked in at compile time; default: this pkg)
#   FASTLIVO_BIN      absolute path of fastlivo_mapping
#                     (default: $FASTLIVO_WS/devel/lib/fast_livo/fastlivo_mapping)
#
# Example (modified build):
#   tool/run_sequence.sh config/geode_beta.yaml config/camera_geode_beta.yaml \
#       /home/gh/data/GEODE/Tunneling_tunnel5_beta.bag geode_tunnel5_beta \
#       tool/eval_results
#
# Example (upstream baseline built in /home/gh/fastlivo2_orig_ws):
#   FASTLIVO_WS=/home/gh/fastlivo2_orig_ws \
#   FASTLIVO_PKG_DIR=/home/gh/fastlivo2_orig_ws/src/FAST-LIVO2 \
#   tool/run_sequence.sh config/geode_beta.yaml config/camera_geode_beta.yaml \
#       /home/gh/data/GEODE/Tunneling_tunnel5_beta.bag tunnel5_original \
#       tool/eval_results
#
# Output:
#   <out_dir>/<seq_name>.txt      — TUM trajectory (copied from Log/result/)
#   <out_dir>/<seq_name>.log      — node console output
#
# The script waits for the bag to finish, then gives the node a few seconds to
# flush the trajectory file before killing it.
# =============================================================================
set -euo pipefail

if [[ $# -lt 5 ]]; then
  sed -n '2,25p' "$0"
  exit 1
fi

CONFIG_YAML="$1"
CAMERA_YAML="$2"
BAG="$3"
SEQ_NAME="$4"
OUT_DIR="$5"
BAG_RATE="${6:-1.0}"
BAG_DURATION="${7:-0}"

THIS_PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # src/FAST-LIVO2 (where this script lives)
PKG_DIR="${FASTLIVO_PKG_DIR:-$THIS_PKG_DIR}"                       # pkg whose Log/result/ the binary writes
WS_DIR="${FASTLIVO_WS:-$(cd "$THIS_PKG_DIR/../.." && pwd)}"         # catkin ws to source
BIN="${FASTLIVO_BIN:-$WS_DIR/devel/lib/fast_livo/fastlivo_mapping}"
RESULT_FILE="$PKG_DIR/Log/result/${SEQ_NAME}.txt"

# Resolve relative paths against the dir this script lives in (configs / outputs
# stay with the modified tree even when running the baseline binary)
[[ "$CONFIG_YAML" = /* ]] || CONFIG_YAML="$THIS_PKG_DIR/$CONFIG_YAML"
[[ "$CAMERA_YAML" = /* ]] || CAMERA_YAML="$THIS_PKG_DIR/$CAMERA_YAML"
[[ "$OUT_DIR"     = /* ]] || OUT_DIR="$THIS_PKG_DIR/$OUT_DIR"

for f in "$CONFIG_YAML" "$CAMERA_YAML" "$BAG" "$BIN"; do
  [[ -e "$f" ]] || { echo "[ERR] missing: $f"; exit 1; }
done
mkdir -p "$OUT_DIR" "$PKG_DIR/Log/result"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/devel/setup.bash"

NODE_LOG="$OUT_DIR/${SEQ_NAME}.log"

cleanup() {
  pkill -9 -f "rosbag play.*$(basename "$BAG")" 2>/dev/null || true
  pkill -9 -f "fastlivo_mapping __name:=laserMapping" 2>/dev/null || true
}
trap cleanup EXIT

echo "============================================================"
echo " seq_name : $SEQ_NAME"
echo " binary   : $BIN"
echo " ROOT_DIR : $PKG_DIR"
echo " config   : $CONFIG_YAML"
echo " camera   : $CAMERA_YAML"
echo " bag      : $BAG"
echo " rate     : $BAG_RATE"
echo " duration : ${BAG_DURATION}s (0 = whole bag)"
echo " out      : $OUT_DIR"
echo "============================================================"

# --- sanity: roscore + republish ------------------------------------------
rostopic list >/dev/null 2>&1 || { echo "[ERR] roscore not running"; exit 1; }
if ! rosnode list 2>/dev/null | grep -q '^/republish$'; then
  echo "[WARN] no /republish node — raw image topic may be missing (LIVO mode needs it)"
fi

# --- stale node cleanup ------------------------------------------------------
pkill -9 -f "fastlivo_mapping __name:=laserMapping" 2>/dev/null || true
sleep 1
echo y | rosnode cleanup >/dev/null 2>&1 || true

# --- params --------------------------------------------------------------------
rosparam load "$CONFIG_YAML"
rosparam load "$CAMERA_YAML" /laserMapping
rosparam set /evo/seq_name "$SEQ_NAME"
rosparam set /evo/pose_output_en true
rosparam set /use_sim_time false

rm -f "$RESULT_FILE"

# --- start node ----------------------------------------------------------------
# absolute path, not `rosrun`: with two chained workspaces both providing
# package `fast_livo`, rosrun's pick is ambiguous.
"$BIN" __name:=laserMapping > "$NODE_LOG" 2>&1 &
NODE_PID=$!
echo "[INFO] fastlivo_mapping pid=$NODE_PID  log=$NODE_LOG"

# wait until node subscribes to the lidar topic
LID_TOPIC="$(rosparam get /common/lid_topic)"
for _ in $(seq 1 30); do
  if rostopic info "$LID_TOPIC" 2>/dev/null | grep -q laserMapping; then break; fi
  sleep 0.5
done
sleep 2

# --- play bag ------------------------------------------------------------------
echo "[INFO] playing bag ..."
BAG_ARGS=(--rate "$BAG_RATE")
if [[ "$BAG_DURATION" != "0" ]]; then BAG_ARGS+=(--duration "$BAG_DURATION"); fi
rosbag play "${BAG_ARGS[@]}" "$BAG" > "$OUT_DIR/${SEQ_NAME}.bag.log" 2>&1 &
BAG_PID=$!

# progress: report pose count every 20s
while kill -0 "$BAG_PID" 2>/dev/null; do
  sleep 20
  if [[ -f "$RESULT_FILE" ]]; then
    printf '[INFO] %s poses so far\n' "$(wc -l < "$RESULT_FILE")"
  else
    echo "[INFO] waiting for first LIO update ..."
  fi
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "[ERR] fastlivo_mapping died — see $NODE_LOG"
    tail -20 "$NODE_LOG"
    exit 1
  fi
done

# let the node drain its buffers, then stop it
sleep 5
kill -INT "$NODE_PID" 2>/dev/null || true
sleep 2
kill -9 "$NODE_PID" 2>/dev/null || true

if [[ ! -s "$RESULT_FILE" ]]; then
  echo "[ERR] no trajectory written to $RESULT_FILE"
  echo "----- last 30 lines of node log -----"
  tail -30 "$NODE_LOG"
  exit 1
fi

cp "$RESULT_FILE" "$OUT_DIR/${SEQ_NAME}.txt"
echo "[DONE] $(wc -l < "$RESULT_FILE") poses -> $OUT_DIR/${SEQ_NAME}.txt"
