#!/usr/bin/env bash
set -u
WORKSPACE="$HOME/catkin_ws"
LOG_FILE="$HOME/go1_gazebo.log"
ROSLAUNCH_PID=""
cleanup() {
    echo
    echo "[INFO] 正在关闭 Go1 仿真..."
    if [ -n "${ROSLAUNCH_PID}" ]; then
        kill "${ROSLAUNCH_PID}" 2>/dev/null || true
        wait "${ROSLAUNCH_PID}" 2>/dev/null || true
    fi
    echo "[INFO] 已结束。"
}
trap cleanup EXIT INT TERM
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GAZEBO_MODEL_DATABASE_URI=""
cd "$WORKSPACE"
echo "[INFO] 正在启动 Gazebo 和 Go1 模型..."
roslaunch unitree_guide gazeboSim.launch gui:=true \
    >"$LOG_FILE" 2>&1 &
ROSLAUNCH_PID=$!
echo "[INFO] 等待 12 个关节控制器加载..."
READY=0
for i in $(seq 1 60); do
    if rosservice list 2>/dev/null | \
       grep -q '/go1_gazebo/controller_manager/list_controllers'; then
        READY=1
        break
    fi
    sleep 1
done
if [ "$READY" -ne 1 ]; then
    echo "[ERROR] Gazebo 未在 60 秒内准备完成。"
    echo "[INFO] 日志末尾："
    tail -n 30 "$LOG_FILE" 2>/dev/null || true
    exit 1
fi
echo "[INFO] Gazebo 已就绪。"
echo "[INFO] 当前终端用于键盘控制；请使用英文输入法。"
echo "[INFO] 按 Ctrl+C 可关闭全部仿真进程。"
echo
"$WORKSPACE/devel/lib/unitree_guide/junior_ctrl"
