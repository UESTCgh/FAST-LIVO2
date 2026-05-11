#!/bin/bash
###############################################################################
# run_fastlivo2_test.sh
#
# 一键运行 FAST-LIVO2 并评估定位精度:
#   1. 启动 roscore
#   2. 启动 FAST-LIVO2 (mapping_super2.launch, 使用 mid360 + super2_d435i 配置)
#   3. 播放 bag 文件
#   4. 等待运行结束
#   5. 使用 evo 工具评估轨迹精度 (RPE + 轨迹统计)
#
# 用法:
#   ./run_fastlivo2_test.sh                          # 默认: guandao1.bag
#   ./run_fastlivo2_test.sh /path/to/your.bag        # 指定 bag 文件
#   ./run_fastlivo2_test.sh /path/to/your.bag gt.txt # 有 ground truth 时做 APE
#
# 依赖: ROS Noetic, fast_livo 已编译, evo (pip install evo)
###############################################################################

set -euo pipefail

# ---------- 配置 ----------
WS_DIR="/home/gh/fastlivo2_ws"
BAG_FILE="${1:-$WS_DIR/dataset/guandao1.bag}"
GT_FILE="${2:-}"
LAUNCH_FILE="mapping_super2.launch"
SEQ_NAME="test_run_$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="$WS_DIR/src/FAST-LIVO2/Log/result"
TRAJ_FILE="$RESULT_DIR/${SEQ_NAME}.txt"
REPORT_DIR="$WS_DIR/eval_results/${SEQ_NAME}"
BAG_RATE="${BAG_RATE:-1.0}"     # 播放速率, 可通过环境变量覆盖
RVIZ="${RVIZ:-true}"            # 是否启动 rviz
WAIT_AFTER_BAG="${WAIT_AFTER_BAG:-5}"  # bag 播完后等待秒数

# ---------- 颜色输出 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_err()   { echo -e "${RED}[ERROR]${NC} $*"; }

# ---------- 前置检查 ----------
if [ ! -f "$BAG_FILE" ]; then
    log_err "Bag 文件不存在: $BAG_FILE"
    exit 1
fi

if [ -n "$GT_FILE" ] && [ ! -f "$GT_FILE" ]; then
    log_err "Ground truth 文件不存在: $GT_FILE"
    exit 1
fi

if ! command -v roslaunch &>/dev/null; then
    log_err "ROS 未安装或未 source"
    exit 1
fi

if ! command -v evo_rpe &>/dev/null; then
    log_warn "evo 未安装, 将跳过精度评估 (pip install evo)"
fi

# ---------- 清理函数 ----------
PIDS=()
cleanup() {
    log_info "正在清理进程..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # 不杀 roscore, 让用户自行管理
    log_info "清理完成"
}
trap cleanup EXIT

# ---------- Source workspace ----------
source "$WS_DIR/devel/setup.bash"

# ---------- Step 1: roscore ----------
if ! rostopic list &>/dev/null 2>&1; then
    log_info "启动 roscore..."
    roscore &
    PIDS+=($!)
    sleep 3
    log_ok "roscore 已启动"
else
    log_ok "roscore 已在运行"
fi

# ---------- Step 2: 修改配置, 启用 pose 输出 ----------
log_info "设置 pose 输出: seq_name=$SEQ_NAME"
mkdir -p "$RESULT_DIR"

# 通过 rosparam 覆盖, 不修改 yaml 文件
rosparam set /evo/pose_output_en true
rosparam set /evo/seq_name "$SEQ_NAME"

# ---------- Step 3: 启动 FAST-LIVO2 ----------
log_info "启动 FAST-LIVO2 ($LAUNCH_FILE)..."
roslaunch fast_livo "$LAUNCH_FILE" rviz:="$RVIZ" &
LIVO_PID=$!
PIDS+=($LIVO_PID)
sleep 5

if ! kill -0 "$LIVO_PID" 2>/dev/null; then
    log_err "FAST-LIVO2 启动失败!"
    exit 1
fi
log_ok "FAST-LIVO2 已启动 (PID: $LIVO_PID)"

# ---------- Step 4: 播放 bag ----------
BAG_DURATION=$(rosbag info --yaml "$BAG_FILE" | grep "duration:" | head -1 | awk '{print $2}')
log_info "播放 bag: $(basename "$BAG_FILE") (时长: ${BAG_DURATION}s, 速率: ${BAG_RATE}x)"
log_info "预计运行时间: $(echo "$BAG_DURATION / $BAG_RATE" | bc -l | xargs printf '%.0f')s"

rosbag play "$BAG_FILE" -r "$BAG_RATE" --clock &
BAG_PID=$!
PIDS+=($BAG_PID)

# ---------- Step 5: 等待 bag 播放完成 ----------
log_info "等待 bag 播放完成..."
wait "$BAG_PID" 2>/dev/null || true
log_ok "Bag 播放结束"

log_info "等待 ${WAIT_AFTER_BAG}s 让 FAST-LIVO2 处理完剩余数据..."
sleep "$WAIT_AFTER_BAG"

# ---------- Step 6: 停止 FAST-LIVO2 ----------
log_info "停止 FAST-LIVO2..."
if kill -0 "$LIVO_PID" 2>/dev/null; then
    kill -INT "$LIVO_PID" 2>/dev/null || true
    sleep 3
fi

# ---------- Step 7: 检查输出轨迹 ----------
if [ ! -f "$TRAJ_FILE" ]; then
    log_err "未找到输出轨迹文件: $TRAJ_FILE"
    log_warn "请检查 FAST-LIVO2 是否正常运行, 以及 pose_output_en 是否生效"
    log_warn "可能需要在 yaml 配置文件中手动设置:"
    log_warn "  evo:"
    log_warn "    pose_output_en: true"
    log_warn "    seq_name: \"$SEQ_NAME\""
    exit 1
fi

TRAJ_LINES=$(wc -l < "$TRAJ_FILE")
log_ok "轨迹已保存: $TRAJ_FILE ($TRAJ_LINES 帧)"

# ---------- Step 8: 精度评估 ----------
mkdir -p "$REPORT_DIR"
cp "$TRAJ_FILE" "$REPORT_DIR/"

if command -v evo_rpe &>/dev/null; then
    echo ""
    log_info "========== 精度评估 =========="

    if [ -n "$GT_FILE" ]; then
        # --- 有 Ground Truth: APE + RPE ---
        log_info "[APE] 绝对位姿误差 (vs Ground Truth)"
        evo_ape tum "$GT_FILE" "$TRAJ_FILE" \
            --align --correct_scale \
            --save_results "$REPORT_DIR/ape_result.zip" \
            --plot_mode xy \
            --save_plot "$REPORT_DIR/ape_plot.pdf" \
            2>&1 | tee "$REPORT_DIR/ape_output.txt"

        echo ""
        log_info "[RPE] 相对位姿误差 (vs Ground Truth)"
        evo_rpe tum "$GT_FILE" "$TRAJ_FILE" \
            --align --correct_scale \
            --delta 1 --delta_unit m \
            --save_results "$REPORT_DIR/rpe_result.zip" \
            --plot_mode xy \
            --save_plot "$REPORT_DIR/rpe_plot.pdf" \
            2>&1 | tee "$REPORT_DIR/rpe_output.txt"
    else
        # --- 无 Ground Truth: 只做 RPE 自评估 + 轨迹统计 ---
        log_info "[RPE] 相对位姿误差 (自一致性评估, delta=1m)"
        evo_rpe tum "$TRAJ_FILE" "$TRAJ_FILE" \
            --delta 1 --delta_unit m \
            --save_results "$REPORT_DIR/rpe_self_result.zip" \
            2>&1 | tee "$REPORT_DIR/rpe_self_output.txt" || true

        echo ""
        log_info "[轨迹统计] evo_traj 分析"
        evo_traj tum "$TRAJ_FILE" \
            --full_check \
            --plot_mode xy \
            --save_plot "$REPORT_DIR/traj_plot.pdf" \
            2>&1 | tee "$REPORT_DIR/traj_stats.txt"
    fi

    echo ""
    log_ok "评估报告已保存到: $REPORT_DIR/"
    ls -la "$REPORT_DIR/"
else
    log_warn "evo 未安装, 跳过精度评估"
    log_warn "安装: pip install evo"
    log_warn "轨迹文件已保存, 可手动评估: $TRAJ_FILE"
fi

echo ""
log_ok "===== 测试完成 ====="
log_info "轨迹文件: $TRAJ_FILE"
log_info "评估报告: $REPORT_DIR/"
