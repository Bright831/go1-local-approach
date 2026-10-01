# Vision-Guided Elevator Button Approach for Unitree Go1

## 1. Project Overview

This repository contains a ROS1 simulation pipeline for vision-guided elevator button approach using a Unitree Go1 quadruped robot.

The original goal of the delivery robot project was to enable indoor navigation and elevator interaction. The current implementation focuses on one key sub-task:

**Autonomously approaching an elevator call button to a suitable interaction distance using onboard perception and closed-loop motion control.**

The current system integrates:

- YOLO-based RGB target detection
- Image-space target alignment
- Go1 local motion control
- Front depth-camera distance estimation
- State-machine based approach behaviour

The current demo completes when the Go1 reaches the predefined interaction distance and enters the `READY` state.

The actual button pressing operation using a robotic arm is outside the current scope.

---

## 2. System Pipeline

```text
RGB Camera
      |
      v
YOLO Button Detection
      |
      v
Target Position Feedback
      |
      v
Approach State Machine
      |
      +----------------+
      |                |
      v                v
RGB Alignment     Depth Approach
      |                |
      +-------+--------+
              |
              v
            READY
```

The controller follows the sequence:

```text
WAIT_TARGET
      |
      v
SEARCH
      |
      v
ALIGN
      |
      v
FAR_APPROACH
      |
      v
DEPTH_APPROACH
      |
      v
READY
```

---

## 3. Main Contributions

### Vision Integration

The project implements a YOLO-based elevator call button perception module.

The vision system provides:

- target validity
- target class information
- horizontal target position
- detection confidence

The detected target position is converted into ROS messages and used by the motion controller for closed-loop alignment.

### Motion Control

A local controller was implemented for:

- target searching when the button is outside the camera view
- horizontal alignment using image-space feedback
- vision-guided far approach
- depth-guided final approach
- terminal `READY` state handling

### Simulation Evaluation

A formal Gazebo simulation evaluation was performed.

Experimental conditions:

- Initial distances:
  - 2.0 m
  - 3.0 m
- Initial heading offsets:
  - 0°
  - +30°
  - -30°
  - +60°
  - -60°
- Repetitions:
  - 2 trials per condition

Total:

```text
20 trials
```

---

## 4. Evaluation Results

The nominal simulation evaluation achieved:

| Metric | Result |
|---|---|
| Successful trials | 20 / 20 |
| Final state | READY |
| Mean final stopping distance | approximately 0.498 m |
| Mean completion time at 2 m | approximately 20.1 s |
| Mean completion time at 3 m | approximately 30.0 s |
| Vision loss events | 0 |
| Depth loss events | 0 |

The complete evaluation data is available at:

```text
evaluation/nominal_20_trials/summary.csv
```

These results represent nominal Gazebo simulation performance.

They do not represent real-world deployment validation or systematic robustness validation.

---

## 5. Repository Structure

```text
go1_local_approach/

├── config/
│   └── button_controller.yaml
│
├── docs/
│   └── VISION_INTERFACE.md
│
├── evaluation/
│   └── nominal_20_trials/
│       └── summary.csv
│
├── launch/
│   ├── button_runtime.launch
│   ├── button_test.launch
│   ├── camera_tf.launch
│   └── sim_elevator.launch
│
├── models/
│   ├── sim_elevator/
│   │   └── model.sdf
│   └── yolo/
│       └── best.pt
│
├── patches/
│   └── unitree_guide_move_base.patch
│
├── scripts/
│   ├── capture_sim_dataset.py
│   ├── check_sim_label.py
│   ├── prepare_yolo_dataset.py
│   ├── set_trial_pose.py
│   ├── start_go1_demo.sh
│   ├── start_go1_sim.sh
│   ├── trial_logger.py
│   └── yolo_button_vision.py
│
└── src/
    ├── button_approach_controller.cpp
    ├── depth_distance_node.cpp
    └── fake_button_vision.cpp
```

---

## 6. Tested Environment

The system has been tested with:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Unitree Go1 simulation
- `unitree_guide`
- `unitree_ros`
- `unitree_legged_msgs`

The project was developed in a catkin workspace:

```text
~/catkin_ws
```

---

## 7. Build Instructions

Place the package under:

```text
~/catkin_ws/src/go1_local_approach
```

Build the workspace:

```bash
cd ~/catkin_ws
catkin_make -j2
source ~/catkin_ws/devel/setup.bash
```

The `-j2` option was used in the tested environment to reduce memory pressure during compilation.

---

## 8. Running the Simulation

### Terminal 1: Start Go1 Simulation

```bash
cd ~/catkin_ws/src/go1_local_approach
./scripts/start_go1_sim.sh
```

After the simulation is ready, use `junior_ctrl` to switch Go1 into the required locomotion mode:

```text
2 = FixedStand
5 = MOVE_BASE
```

The autonomous controller should be run only after Go1 has entered `MOVE_BASE`.

### Terminal 2: Run Autonomous Approach Demo

Example:

```bash
cd ~/catkin_ws/src/go1_local_approach
./scripts/start_go1_demo.sh 1
```

An optional timeout can be supplied as the second argument:

```bash
./scripts/start_go1_demo.sh 1 120
```

The script automatically:

1. checks the ROS master and required camera topics
2. starts the YOLO vision node
3. starts the trial logger
4. launches the depth node and approach controller
5. waits for `READY` or timeout
6. saves the experiment result
7. shuts down the autonomous stack

Experiment results are saved under:

```text
~/go1_experiment_logs/
```

---

## 9. YOLO Vision Module

The YOLO vision node is implemented in:

```text
scripts/yolo_button_vision.py
```

The default model weight is included at:

```text
models/yolo/best.pt
```

The detector publishes:

- `/vision/target_valid`
- `/vision/target_type`
- `/vision/target_center_x`
- `/vision/target_confidence`

The controller accepts the target class:

```text
elevator_call_button
```

The default model path can be overridden using the private ROS parameter:

```text
_model_path:=/path/to/custom/model.pt
```

The detector output uses horizontal image position for steering. It does not provide a 3D target position to the controller.

---

## 10. Depth Estimation

The depth node is implemented in:

```text
src/depth_distance_node.cpp
```

It subscribes to:

```text
/camera_face/depth/image_raw
```

and publishes:

```text
/front_distance
```

The current simulation uses an adaptive horizontal ROI search around the image centre to obtain a valid front-surface depth estimate.

The valid depth range is limited to approximately:

```text
0.10 m to 1.50 m
```

This adaptive search is a simulation-specific workaround for RGB/depth misregistration in the current Gazebo setup.

A real RGB-D implementation should use appropriate sensor calibration and registered depth data.

---

## 11. Controller Behaviour

The main controller is implemented in:

```text
src/button_approach_controller.cpp
```

### SEARCH

Before the first valid target has been detected, the robot may rotate in place to search for the elevator call button.

Default search angular speed:

```text
0.25 rad/s
```

### ALIGN

If the horizontal target error exceeds the realignment threshold, the robot stops forward motion and rotates in place.

Default threshold:

```text
80 pixels
```

### FAR_APPROACH

Once the target is sufficiently aligned, the robot may move forward using visual steering while no valid depth has yet been acquired.

### DEPTH_APPROACH

After valid depth tracking begins, the robot uses `/front_distance` for forward approach while continuing to use visual horizontal error for steering.

### READY

The current demo stops when:

```text
front_distance <= 0.50 m
```

`READY` is implemented as a terminal state for the current controller run. Small depth fluctuations around the threshold cannot cause the robot to re-enter `DEPTH_APPROACH`.

---

## 12. Implemented Safety Behaviour

The controller includes conservative stop behaviour for perception loss.

### Vision loss after acquisition

If the target has previously been acquired and the visual target is subsequently lost:

```text
VISION_LOST
    |
    v
   STOP
```

The controller does not automatically restart `SEARCH`.

### Depth loss after acquisition

If valid depth tracking has already been established and depth later becomes stale:

```text
DEPTH_LOST
    |
    v
   STOP
```

The controller does not return automatically to `FAR_APPROACH`.

These behaviours are implemented in the controller, but the 20-trial nominal evaluation did not constitute a systematic fault-injection or robustness study.

---

## 13. Coordinate and Measurement Convention

The controller intentionally separates different measurement spaces.

### RGB vision

`target_center_x` is an image-pixel coordinate used only for horizontal steering.

### Depth

`/front_distance` is a scalar range measurement used for approach and stopping.

The controller does not directly subtract or combine:

- image pixel coordinates
- `base` coordinates
- `odom` coordinates
- camera-frame 3D coordinates

unless an appropriate transformation or interpretation is explicitly defined.

Trial setup and evaluation logging use Gazebo world-frame quantities consistently when computing robot/elevator position and heading relationships.

---

## 14. Controller Parameters

Runtime parameters are stored in:

```text
config/button_controller.yaml
```

Current key values include:

| Parameter | Value |
|---|---:|
| `image_width` | 640.0 |
| `center_threshold_px` | 40.0 |
| `realign_threshold_px` | 80.0 |
| `min_confidence` | 0.60 |
| `vision_timeout` | 0.5 s |
| `depth_timeout` | 0.5 s |
| `max_valid_depth` | 1.50 m |
| `stop_distance` | 0.50 m |
| `search_angular_speed` | 0.25 rad/s |
| `far_speed` | 0.08 m/s |
| `medium_speed` | 0.08 m/s |
| `near_speed` | 0.04 m/s |
| `kp_angular_align` | 0.002 |
| `kp_angular_move` | 0.001 |
| `max_align_angular` | 0.35 rad/s |
| `max_move_angular` | 0.12 rad/s |
| `yaw_sign` | 1.0 |

`center_threshold_px` is currently retained for future tuning; the main realignment gate uses `realign_threshold_px`.

---

## 15. Unitree MOVE_BASE Requirement

The project depends on Unitree `MOVE_BASE` support so that `/cmd_vel` can command Go1.

A patch is included at:

```text
patches/unitree_guide_move_base.patch
```

The relevant Unitree build configuration enables ROS and MOVE_BASE support.

The tested `junior_ctrl` operating sequence is:

```text
2 -> FixedStand
5 -> MOVE_BASE
```

---

## 16. Evaluation Dataset

The formal nominal evaluation data is stored in:

```text
evaluation/nominal_20_trials/summary.csv
```

The test matrix contains:

```text
2 distances
x 5 initial heading offsets
x 2 repetitions
= 20 trials
```

Conditions:

```text
Distance: 2 m, 3 m
Heading: 0°, +30°, -30°, +60°, -60°
```

All 20 nominal trials reached `READY`.

The +60° conditions consistently required `SEARCH` before target acquisition in the recorded nominal evaluation.

The results demonstrate repeatability in the tested Gazebo setup, not general real-world robustness.

---

## 17. Limitations

The current implementation is limited to simulation-based local elevator approach.

It does not yet include:

- real Go1 hardware validation
- robotic arm integration
- physical elevator button pressing
- button-press verification
- elevator-door state verification
- full indoor or campus navigation

The current 0.50 m stopping distance is a demonstration interaction distance rather than a final manipulator working distance.

---

## 18. Future Work

Planned extensions include:

- systematic perception-loss and depth-loss robustness evaluation
- real Go1 validation
- manipulator integration
- final interaction-distance calibration
- elevator button pressing
- door interaction
- integration with a larger indoor delivery navigation stack

---

## 19. Version Information

Formal evaluation baseline:

```text
v1.2.2-evaluation-final
```

Baseline commit:

```text
4916d70
```

Delivery version:

```text
v1.2.3-delivery
```

The delivery version contains repository cleanup, English comments/documentation, packaged evaluation data, and the YOLO model required by the current perception pipeline.

No intended changes were made to the evaluated controller behaviour during delivery cleanup.
