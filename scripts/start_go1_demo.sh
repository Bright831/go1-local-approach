#!/usr/bin/env bash

set -u

# ============================================================
# Go1 autonomous demo + experiment logger
#
# Usage:
#
#   ~/start_go1_demo.sh 2
#
# Optional timeout:
#
#   ~/start_go1_demo.sh 2 120
#
# Arguments:
#   $1 = trial ID
#   $2 = timeout in seconds, default 120
# ============================================================


WORKSPACE="$HOME/catkin_ws"
VENV="$HOME/venvs/go1_yolo"

YOLO_LOG="$HOME/go1_yolo_vision.log"

TRIAL_ID="${1:-}"
TIMEOUT_S="${2:-120}"

YOLO_PID=""
LOGGER_PID=""
RUNTIME_PID=""


# ============================================================
# Check arguments
# ============================================================

if [ -z "$TRIAL_ID" ]; then
    echo
    echo "[ERROR] 缺少 Trial ID。"
    echo
    echo "用法："
    echo
    echo "    ~/start_go1_demo.sh <trial_id>"
    echo
    echo "例如："
    echo
    echo "    ~/start_go1_demo.sh 2"
    echo
    exit 1
fi


if ! [[ "$TRIAL_ID" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] Trial ID 必须是整数。"
    exit 1
fi


# ============================================================
# Cleanup
# ============================================================

cleanup() {
    echo
    echo "[INFO] 正在关闭 Trial ${TRIAL_ID}..."

    # Controller + depth runtime
    if [ -n "${RUNTIME_PID}" ]; then
        kill -INT "${RUNTIME_PID}" 2>/dev/null || true
        wait "${RUNTIME_PID}" 2>/dev/null || true
    fi

    # Logger
    if [ -n "${LOGGER_PID}" ]; then
        kill -INT "${LOGGER_PID}" 2>/dev/null || true
        wait "${LOGGER_PID}" 2>/dev/null || true
    fi

    # YOLO
    if [ -n "${YOLO_PID}" ]; then
        kill -INT "${YOLO_PID}" 2>/dev/null || true
        wait "${YOLO_PID}" 2>/dev/null || true
    fi

    echo "[INFO] Trial ${TRIAL_ID} 已结束。"
}

trap cleanup EXIT INT TERM


# ============================================================
# Environment
# ============================================================

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
source "$VENV/bin/activate"


# ============================================================
# Check ROS Master
# ============================================================

echo "[INFO] 检查 ROS Master..."

if ! rostopic list >/dev/null 2>&1; then
    echo
    echo "[ERROR] ROS Master 不可用。"
    echo
    echo "请先在 Terminal 1 启动："
    echo
    echo "    ~/start_go1_sim.sh"
    echo
    exit 1
fi

echo "[INFO] ROS Master OK."


# ============================================================
# Check RGB camera
# ============================================================

echo "[INFO] 检查 RGB camera..."

CAMERA_READY=0

for i in $(seq 1 20); do

    if rostopic list 2>/dev/null | \
       grep -q '^/camera_face/color/image_raw$'; then

        CAMERA_READY=1
        break
    fi

    sleep 1
done


if [ "$CAMERA_READY" -ne 1 ]; then

    echo
    echo "[ERROR] 找不到："
    echo
    echo "    /camera_face/color/image_raw"
    echo
    echo "请确认 Gazebo + Go1 已完全启动。"

    exit 1
fi

echo "[INFO] RGB camera OK."


# ============================================================
# Check depth camera
# ============================================================

echo "[INFO] 检查 Depth camera..."

DEPTH_READY=0

for i in $(seq 1 20); do

    if rostopic list 2>/dev/null | \
       grep -q '^/camera_face/depth/image_raw$'; then

        DEPTH_READY=1
        break
    fi

    sleep 1
done


if [ "$DEPTH_READY" -ne 1 ]; then

    echo
    echo "[ERROR] 找不到："
    echo
    echo "    /camera_face/depth/image_raw"

    exit 1
fi

echo "[INFO] Depth camera OK."


# ============================================================
# Check Gazebo model states
# ============================================================

echo "[INFO] 检查 Gazebo model states..."

if ! rostopic list 2>/dev/null | \
     grep -q '^/gazebo/model_states$'; then

    echo
    echo "[ERROR] 找不到 /gazebo/model_states"
    echo
    echo "trial_logger 无法记录初始位置和角度。"

    exit 1
fi

echo "[INFO] Gazebo model states OK."


# ============================================================
# Prevent accidental overwrite
# ============================================================

TRIAL_FILE="$HOME/go1_experiment_logs/trial_$(printf "%03d" "$TRIAL_ID").csv"

if [ -f "$TRIAL_FILE" ]; then

    echo
    echo "[ERROR] Trial ${TRIAL_ID} 已经存在："
    echo
    echo "    $TRIAL_FILE"
    echo
    echo "请使用新的 Trial ID。"

    exit 1
fi


# ============================================================
# Start YOLO
# ============================================================

echo
echo "[INFO] 启动 YOLO vision..."

rosrun go1_local_approach yolo_button_vision.py \
    >"$YOLO_LOG" 2>&1 &

YOLO_PID=$!


# ============================================================
# Wait for YOLO topics
# ============================================================

echo "[INFO] 等待 YOLO vision topics..."

VISION_READY=0

for i in $(seq 1 30); do

    if rostopic list 2>/dev/null | \
       grep -q '^/vision/target_valid$'; then

        VISION_READY=1
        break
    fi

    # YOLO died while starting
    if ! kill -0 "$YOLO_PID" 2>/dev/null; then

        echo
        echo "[ERROR] YOLO vision node 提前退出。"
        echo
        echo "[INFO] YOLO 日志末尾："

        tail -n 40 "$YOLO_LOG" 2>/dev/null || true

        exit 1
    fi

    sleep 1
done


if [ "$VISION_READY" -ne 1 ]; then

    echo
    echo "[ERROR] YOLO vision node 未准备完成。"
    echo
    echo "[INFO] YOLO 日志末尾："

    tail -n 40 "$YOLO_LOG" 2>/dev/null || true

    exit 1
fi

echo "[INFO] YOLO vision ready."


# ============================================================
# Start trial logger BEFORE controller
#
# Logger waits for /button_approach/state.
# Therefore timing begins when controller comes online.
# ============================================================

echo
echo "[INFO] 启动 Trial Logger..."

rosrun go1_local_approach trial_logger.py \
    _trial_id:="$TRIAL_ID" \
    _timeout_s:="$TIMEOUT_S" &

LOGGER_PID=$!


# Give logger time to subscribe
sleep 1


if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    echo "[ERROR] Trial logger 启动失败。"
    exit 1
fi


# ============================================================
# Start controller + depth
# ============================================================

echo
echo "============================================================"
echo " Go1 Autonomous Button Approach Experiment"
echo "============================================================"
echo
echo " Trial ID     : ${TRIAL_ID}"
echo " Timeout      : ${TIMEOUT_S} s"
echo
echo " YOLO         : running"
echo " Trial Logger : running"
echo " Depth        : starting"
echo " Controller   : starting"
echo
echo " Expected sequence:"
echo
echo "   WAIT_TARGET"
echo "       ↓"
echo "   SEARCH"
echo "       ↓"
echo "   ALIGN"
echo "       ↓"
echo "   FAR_APPROACH"
echo "       ↓"
echo "   DEPTH_APPROACH"
echo "       ↓"
echo "   READY"
echo
echo " Trial data:"
echo
echo "   ~/go1_experiment_logs/trial_$(printf "%03d" "$TRIAL_ID").csv"
echo "   ~/go1_experiment_logs/summary.csv"
echo
echo " Ctrl+C = manual failure / stop"
echo "============================================================"
echo


roslaunch go1_local_approach button_runtime.launch &

RUNTIME_PID=$!


# ============================================================
# Wait for logger
#
# READY:
#   logger saves SUCCESS and exits.
#
# TIMEOUT:
#   logger saves FAILURE and exits.
#
# Then this script exits and cleanup() shuts down controller
# and YOLO automatically.
# ============================================================

wait "$LOGGER_PID"

LOGGER_PID=""

echo
echo "============================================================"
echo "[INFO] Trial ${TRIAL_ID} logger 已完成。"
echo "[INFO] 正在自动关闭 autonomous stack..."
echo "============================================================"

exit 0