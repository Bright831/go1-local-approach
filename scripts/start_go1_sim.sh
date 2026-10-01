#!/usr/bin/env bash

set -u

WORKSPACE="$HOME/catkin_ws"

GAZEBO_LOG="$HOME/go1_gazebo.log"
ELEVATOR_LOG="$HOME/go1_elevator.log"

ROSLAUNCH_PID=""
ELEVATOR_PID=""

cleanup() {
    echo
    echo "[INFO] Shutting down the Go1 simulation..."

    if [ -n "${ELEVATOR_PID}" ]; then
        kill "${ELEVATOR_PID}" 2>/dev/null || true
        wait "${ELEVATOR_PID}" 2>/dev/null || true
    fi

    if [ -n "${ROSLAUNCH_PID}" ]; then
        kill "${ROSLAUNCH_PID}" 2>/dev/null || true
        wait "${ROSLAUNCH_PID}" 2>/dev/null || true
    fi

    echo "[INFO] Simulation shutdown complete."
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
# 1. Start Gazebo and Go1
# ============================================================

echo "[INFO] Starting Gazebo and Go1..."

roslaunch unitree_guide gazeboSim.launch gui:=true \
    >"$GAZEBO_LOG" 2>&1 &

ROSLAUNCH_PID=$!


# ============================================================
# 2. Wait for the Go1 controller manager
# ============================================================

echo "[INFO] Waiting for the Go1 controller manager..."

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

    echo "[ERROR] Gazebo / Go1 did not become ready within 60 seconds."
    echo
    echo "[INFO] Last lines of the Gazebo log:"

    tail -n 40 "$GAZEBO_LOG" 2>/dev/null || true

    exit 1
fi


echo "[INFO] Go1 controller manager is ready."


# ============================================================
# 3. Wait for the Gazebo model-spawn service
# ============================================================

echo "[INFO] Waiting for the Gazebo model-spawn service..."

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

    echo "[ERROR] /gazebo/spawn_sdf_model did not become ready."

    exit 1
fi


echo "[INFO] Gazebo model-spawn service is ready."


# ============================================================
# 4. Spawn the simulated elevator
# ============================================================

echo "[INFO] Spawning the simulated elevator..."

rm -f "$ELEVATOR_LOG"

roslaunch go1_local_approach sim_elevator.launch \
    >"$ELEVATOR_LOG" 2>&1 &

ELEVATOR_PID=$!


# ============================================================
# 5. Verify that the elevator model exists in Gazebo
# ============================================================

echo "[INFO] Waiting for sim_elevator to appear in Gazebo..."

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
    echo "[ERROR] sim_elevator was not spawned successfully."
    echo
    echo "[INFO] Elevator launch log:"
    echo "----------------------------------------"

    cat "$ELEVATOR_LOG" 2>/dev/null || true

    echo "----------------------------------------"

    exit 1
fi


echo "[INFO] sim_elevator was spawned successfully."


# ============================================================
# 6. Start Unitree junior_ctrl in the foreground
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
echo "[INFO] This terminal is now attached to junior_ctrl."
echo
echo "To prepare Go1 for the autonomous demo, enter:"
echo
echo "    2"
echo "    5"
echo
echo "[INFO] 2 = FixedStand"
echo "[INFO] 5 = MOVE_BASE"
echo
echo "[INFO] Do not use manual WASD control after entering MOVE_BASE."
echo
echo "[INFO] In Terminal 2, run:"
echo
echo "    ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh <trial_id>"
echo
echo "Example:"
echo
echo "    ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh 901"
echo
echo "[INFO] Press Ctrl+C in this terminal to shut down the simulation."
echo
echo "============================================================"
echo


"$WORKSPACE/devel/lib/unitree_guide/junior_ctrl"
