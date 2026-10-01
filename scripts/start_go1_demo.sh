#!/usr/bin/env bash

set -u

# ============================================================
# Go1 autonomous demo with experiment logging
#
# Usage:
#
#   ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh 2
#
# Optional timeout:
#
#   ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh 2 120
#
# Arguments:
#   $1 = trial ID
#   $2 = timeout in seconds, default: 120
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
    echo "[ERROR] Missing Trial ID."
    echo
    echo "Usage:"
    echo
    echo "    ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh <trial_id>"
    echo
    echo "Example:"
    echo
    echo "    ~/catkin_ws/src/go1_local_approach/scripts/start_go1_demo.sh 2"
    echo
    exit 1
fi


if ! [[ "$TRIAL_ID" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] Trial ID must be an integer."
    exit 1
fi


# ============================================================
# Cleanup
# ============================================================

cleanup() {
    echo
    echo "[INFO] Shutting down Trial ${TRIAL_ID}..."

    # Controller and depth runtime
    if [ -n "${RUNTIME_PID}" ]; then
        kill -INT "${RUNTIME_PID}" 2>/dev/null || true
        wait "${RUNTIME_PID}" 2>/dev/null || true
    fi

    # Trial logger
    if [ -n "${LOGGER_PID}" ]; then
        kill -INT "${LOGGER_PID}" 2>/dev/null || true
        wait "${LOGGER_PID}" 2>/dev/null || true
    fi

    # YOLO vision node
    if [ -n "${YOLO_PID}" ]; then
        kill -INT "${YOLO_PID}" 2>/dev/null || true
        wait "${YOLO_PID}" 2>/dev/null || true
    fi

    echo "[INFO] Trial ${TRIAL_ID} has ended."
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

echo "[INFO] Checking ROS Master..."

if ! rostopic list >/dev/null 2>&1; then
    echo
    echo "[ERROR] ROS Master is not available."
    echo
    echo "Start the simulation first in Terminal 1:"
    echo
    echo "    ~/start_go1_sim.sh"
    echo
    exit 1
fi

echo "[INFO] ROS Master is available."


# ============================================================
# Check RGB camera
# ============================================================

echo "[INFO] Checking RGB camera..."

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
    echo "[ERROR] Required RGB topic was not found:"
    echo
    echo "    /camera_face/color/image_raw"
    echo
    echo "Make sure Gazebo and Go1 are fully started."

    exit 1
fi

echo "[INFO] RGB camera is ready."


# ============================================================
# Check depth camera
# ============================================================

echo "[INFO] Checking depth camera..."

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
    echo "[ERROR] Required depth topic was not found:"
    echo
    echo "    /camera_face/depth/image_raw"

    exit 1
fi

echo "[INFO] Depth camera is ready."


# ============================================================
# Check Gazebo model states
# ============================================================

echo "[INFO] Checking Gazebo model states..."

if ! rostopic list 2>/dev/null | \
     grep -q '^/gazebo/model_states$'; then

    echo
    echo "[ERROR] /gazebo/model_states was not found."
    echo
    echo "The trial logger cannot record the initial position and heading."

    exit 1
fi

echo "[INFO] Gazebo model states are available."


# ============================================================
# Prevent accidental overwrite
# ============================================================

TRIAL_FILE="$HOME/go1_experiment_logs/trial_$(printf "%03d" "$TRIAL_ID").csv"

if [ -f "$TRIAL_FILE" ]; then

    echo
    echo "[ERROR] Trial ${TRIAL_ID} already exists:"
    echo
    echo "    $TRIAL_FILE"
    echo
    echo "Please use a new Trial ID."

    exit 1
fi


# ============================================================
# Start YOLO vision
# ============================================================

echo
echo "[INFO] Starting YOLO vision..."

rosrun go1_local_approach yolo_button_vision.py \
    >"$YOLO_LOG" 2>&1 &

YOLO_PID=$!


# ============================================================
# Wait for YOLO topics
# ============================================================

echo "[INFO] Waiting for YOLO vision topics..."

VISION_READY=0

for i in $(seq 1 30); do

    if rostopic list 2>/dev/null | \
       grep -q '^/vision/target_valid$'; then

        VISION_READY=1
        break
    fi

    # Detect an early YOLO process failure during startup.
    if ! kill -0 "$YOLO_PID" 2>/dev/null; then

        echo
        echo "[ERROR] YOLO vision node exited during startup."
        echo
        echo "[INFO] Last lines of the YOLO log:"

        tail -n 40 "$YOLO_LOG" 2>/dev/null || true

        exit 1
    fi

    sleep 1
done


if [ "$VISION_READY" -ne 1 ]; then

    echo
    echo "[ERROR] YOLO vision node did not become ready."
    echo
    echo "[INFO] Last lines of the YOLO log:"

    tail -n 40 "$YOLO_LOG" 2>/dev/null || true

    exit 1
fi

echo "[INFO] YOLO vision is ready."


# ============================================================
# Start trial logger BEFORE the controller
#
# The logger waits for /button_approach/state.
# Therefore, trial timing begins when the controller comes online.
# ============================================================

echo
echo "[INFO] Starting trial logger..."

rosrun go1_local_approach trial_logger.py \
    _trial_id:="$TRIAL_ID" \
    _timeout_s:="$TIMEOUT_S" &

LOGGER_PID=$!


# Give the logger time to subscribe.
sleep 1


if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    echo "[ERROR] Trial logger failed to start."
    exit 1
fi


# ============================================================
# Start controller and depth runtime
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
echo " Expected state sequence:"
echo
echo "   WAIT_TARGET"
echo "       |"
echo "       v"
echo "   SEARCH"
echo "       |"
echo "       v"
echo "   ALIGN"
echo "       |"
echo "       v"
echo "   FAR_APPROACH"
echo "       |"
echo "       v"
echo "   DEPTH_APPROACH"
echo "       |"
echo "       v"
echo "   READY"
echo
echo " Trial data:"
echo
echo "   ~/go1_experiment_logs/trial_$(printf "%03d" "$TRIAL_ID").csv"
echo "   ~/go1_experiment_logs/summary.csv"
echo
echo " Ctrl+C = manual stop / trial failure"
echo "============================================================"
echo


roslaunch go1_local_approach button_runtime.launch &

RUNTIME_PID=$!


# ============================================================
# Wait for the trial logger
#
# READY:
#   The logger records SUCCESS and exits.
#
# TIMEOUT:
#   The logger records FAILURE and exits.
#
# The script then exits, and cleanup() automatically shuts down
# the controller, depth runtime, and YOLO vision node.
# ============================================================

wait "$LOGGER_PID"

LOGGER_PID=""

echo
echo "============================================================"
echo "[INFO] Trial ${TRIAL_ID} logger has completed."
echo "[INFO] Shutting down the autonomous stack automatically..."
echo "============================================================"

exit 0
