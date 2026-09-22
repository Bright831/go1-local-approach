#!/usr/bin/env bash

set -u

WORKSPACE="$HOME/catkin_ws"
VENV="$HOME/venvs/go1_yolo"

YOLO_LOG="$HOME/go1_yolo_vision.log"

YOLO_PID=""


cleanup() {
    echo
    echo "[INFO] 正在关闭 Go1 autonomous demo..."

    if [ -n "${YOLO_PID}" ]; then
        kill "${YOLO_PID}" 2>/dev/null || true
        wait "${YOLO_PID}" 2>/dev/null || true
    fi

    echo "[INFO] Demo 已结束。"
}


trap cleanup EXIT INT TERM


# ============================================================
# Environment
# ============================================================

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
source "$VENV/bin/activate"


# ============================================================
# Check ROS master
# ============================================================

echo "[INFO] 检查 ROS Master..."

if ! rostopic list >/dev/null 2>&1; then

    echo "[ERROR] ROS Master 不可用。"
    echo
    echo "请先在 Terminal 1 运行："
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

    echo "[ERROR] 找不到："
    echo "/camera_face/color/image_raw"
    echo
    echo "请确认 Gazebo 和 Go1 已完全启动。"

    exit 1
fi


echo "[INFO] RGB camera OK."


# ============================================================
# Start YOLO
# ============================================================

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

    sleep 1
done


if [ "$VISION_READY" -ne 1 ]; then

    echo "[ERROR] YOLO vision node 未正常启动。"
    echo
    echo "[INFO] YOLO 日志末尾："

    tail -n 30 "$YOLO_LOG" 2>/dev/null || true

    exit 1
fi


echo "[INFO] YOLO vision ready."


# ============================================================
# Start depth + button controller
#
# Keep this in foreground so controller logs remain visible.
# ============================================================

echo
echo "============================================================"
echo " Go1 Autonomous Button Approach Demo"
echo "============================================================"
echo
echo "[INFO] YOLO       : running"
echo "[INFO] RGB vision : running"
echo
echo "预期状态："
echo
echo " SEARCH"
echo "   ↓"
echo " TARGET ACQUIRED"
echo "   ↓"
echo " ALIGN"
echo "   ↓"
echo " FAR_APPROACH"
echo "   ↓"
echo " DEPTH_APPROACH"
echo "   ↓"
echo " READY"
echo
echo "[INFO] Ctrl+C 可关闭 Demo。"
echo "============================================================"
echo


roslaunch go1_local_approach button_runtime.launch
