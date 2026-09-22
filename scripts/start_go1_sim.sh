#!/usr/bin/env bash

set -u

WORKSPACE="$HOME/catkin_ws"

GAZEBO_LOG="$HOME/go1_gazebo.log"
ELEVATOR_LOG="$HOME/go1_elevator.log"

ROSLAUNCH_PID=""
ELEVATOR_PID=""

cleanup() {
    echo
    echo "[INFO] 正在关闭 Go1 仿真..."

    if [ -n "${ELEVATOR_PID}" ]; then
        kill "${ELEVATOR_PID}" 2>/dev/null || true
        wait "${ELEVATOR_PID}" 2>/dev/null || true
    fi

    if [ -n "${ROSLAUNCH_PID}" ]; then
        kill "${ROSLAUNCH_PID}" 2>/dev/null || true
        wait "${ROSLAUNCH_PID}" 2>/dev/null || true
    fi

    echo "[INFO] 已结束。"
}

trap cleanup EXIT INT TERM


# ============================================================
# ROS environment
# ============================================================

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GAZEBO_MODEL_DATABASE_URI=""

cd "$WORKSPACE"


# ============================================================
# 1. Start Gazebo + Go1
# ============================================================

echo "[INFO] 正在启动 Gazebo 和 Go1..."

roslaunch unitree_guide gazeboSim.launch gui:=true \
    >"$GAZEBO_LOG" 2>&1 &

ROSLAUNCH_PID=$!


# ============================================================
# 2. Wait for Go1 controller manager
# ============================================================

echo "[INFO] 等待 Go1 控制器加载..."

READY=0

for i in $(seq 1 60); do

    if rosservice list 2>/dev/null | \
       grep -q '^/go1_gazebo/controller_manager/list_controllers$'; then

        READY=1
        break
    fi

    sleep 1
done


if [ "$READY" -ne 1 ]; then

    echo "[ERROR] Gazebo / Go1 未在 60 秒内准备完成。"
    echo
    echo "[INFO] Gazebo 日志末尾："

    tail -n 40 "$GAZEBO_LOG" 2>/dev/null || true

    exit 1
fi


echo "[INFO] Go1 controller manager 已就绪。"


# ============================================================
# 3. Wait for Gazebo spawn service
# ============================================================

echo "[INFO] 等待 Gazebo spawn service..."

SPAWN_READY=0

for i in $(seq 1 30); do

    if rosservice list 2>/dev/null | \
       grep -q '^/gazebo/spawn_sdf_model$'; then

        SPAWN_READY=1
        break
    fi

    sleep 1
done


if [ "$SPAWN_READY" -ne 1 ]; then

    echo "[ERROR] /gazebo/spawn_sdf_model 未准备完成。"

    exit 1
fi


echo "[INFO] Gazebo spawn service 已就绪。"


# ============================================================
# 4. Spawn simulated elevator
# ============================================================

echo "[INFO] 正在生成模拟电梯..."

rm -f "$ELEVATOR_LOG"

roslaunch go1_local_approach sim_elevator.launch \
    >"$ELEVATOR_LOG" 2>&1 &

ELEVATOR_PID=$!


# ============================================================
# 5. Verify elevator actually exists
# ============================================================

echo "[INFO] 等待 sim_elevator 出现在 Gazebo..."

ELEVATOR_READY=0

for i in $(seq 1 20); do

    if rosservice call /gazebo/get_world_properties 2>/dev/null | \
       grep -q 'sim_elevator'; then

        ELEVATOR_READY=1
        break
    fi

    sleep 1
done


if [ "$ELEVATOR_READY" -ne 1 ]; then

    echo
    echo "[ERROR] sim_elevator 未成功生成。"
    echo
    echo "[INFO] Elevator launch 日志："
    echo "----------------------------------------"

    cat "$ELEVATOR_LOG" 2>/dev/null || true

    echo "----------------------------------------"

    exit 1
fi


echo "[INFO] sim_elevator 已成功生成。"


# ============================================================
# 6. Start Unitree junior_ctrl in FOREGROUND
# ============================================================

echo
echo "============================================================"
echo " Go1 Simulation Ready"
echo "============================================================"
echo
echo "[INFO] Gazebo       : running"
echo "[INFO] Go1          : running"
echo "[INFO] sim_elevator : running"
echo
echo "[INFO] 当前终端属于 junior_ctrl。"
echo
echo "完整 Demo 请按："
echo
echo "    2"
echo "    5"
echo
echo "[INFO] 2 = FixedStand"
echo "[INFO] 5 = MOVE_BASE"
echo
echo "[INFO] 之后不要使用 WASD。"
echo
echo "[INFO] Terminal 2 再运行："
echo
echo "    ~/start_go1_demo.sh"
echo
echo "[INFO] Ctrl+C 关闭整个仿真。"
echo
echo "============================================================"
echo


"$WORKSPACE/devel/lib/unitree_guide/junior_ctrl"