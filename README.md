# Go1 Local Elevator Button Approach

## 1. Project Overview

This ROS1 package implements the local elevator-button approach function for a Unitree Go1 robot.

The current demo is designed to perform the following tasks:

1. Receive elevator call-button detection results from an external RGB vision system.
2. Search for the target by rotating in place when the vision system is running but no button has been detected.
3. Align the robot horizontally with the detected elevator call button.
4. Approach the elevator panel using RGB guidance when depth information is unavailable.
5. Switch to Go1 front depth-camera guidance when valid depth becomes available.
6. Stop at a predefined interaction distance.
7. Stop safely if the visual target or depth information is lost after acquisition.

The current version does NOT press the elevator button.

The final pressing operation will be implemented later using a robotic arm.

The current demo therefore ends when the Go1 reaches the predefined interaction distance and enters the READY state.


============================================================
2. Tested Environment
============================================================

The current implementation has been tested with:

Ubuntu 20.04
ROS1 Noetic
Gazebo 11
Unitree Go1 simulation
unitree_guide
unitree_ros
unitree_legged_msgs

Development workspace:

~/catkin_ws

Package location:

~/catkin_ws/src/go1_local_approach


============================================================
3. Overall System Architecture
============================================================

External RGB Camera
        |
        v
PyTorch / YOLO Vision Node
        |
        | /vision/target_valid
        | /vision/target_type
        | /vision/target_center_x
        | /vision/target_confidence
        |
        v
button_approach_controller
        ^
        |
        | /front_distance
        |
depth_distance_node
        ^
        |
Go1 Front Depth Camera
        |
        v
button_approach_controller
        |
        | /cmd_vel
        v
Unitree MOVE_BASE Controller
        |
        v
       Go1


The RGB vision system is responsible for target detection and horizontal alignment information.

The Go1 front depth camera is responsible for measuring the distance to the elevator panel or front surface.

The local controller combines both sources and generates /cmd_vel commands.


============================================================
4. Core Nodes
============================================================

4.1 button_approach_controller

Source file:

src/button_approach_controller.cpp

This is the main local motion controller.

Subscribed topics:

/vision/target_valid
std_msgs/Bool

/vision/target_type
std_msgs/String

/vision/target_center_x
std_msgs/Float32

/vision/target_confidence
std_msgs/Float32

/front_distance
std_msgs/Float32

Published topic:

/cmd_vel
geometry_msgs/Twist


Main controller states:

WAIT_TARGET
SEARCH
ALIGN
FAR_APPROACH
DEPTH_APPROACH
READY
VISION_LOST
DEPTH_LOST


============================================================
4.2 depth_distance_node
============================================================

Source file:

src/depth_distance_node.cpp

Input:

/camera_face/depth/image_raw

Message type:

sensor_msgs/Image

Expected image format:

32FC1

Output:

/front_distance

Message type:

std_msgs/Float32


The node calculates the median depth inside a central region of interest of the Go1 front depth image.

Current ROI:

center +/- 30 pixels horizontally
center +/- 30 pixels vertically

Current valid depth range:

0.10 m to 1.50 m


Important:

/front_distance is a scalar depth measurement.

It is NOT directly combined with base-frame or odom-frame position coordinates.


============================================================
4.3 fake_button_vision
============================================================

Source file:

src/fake_button_vision.cpp

This node is only used for testing the local controller without a real YOLO detector.

It simulates the ROS topics that will eventually be published by the real vision system.

It should NOT be used during the final real-vision demo.

It must NOT run at the same time as the real YOLO vision node, because both would publish the same /vision/target_* topics.


============================================================
5. Vision Interface Specification
============================================================

The real ROS1 vision system must continuously publish four topics.


------------------------------------------------------------
5.1 Target Valid
------------------------------------------------------------

Topic:

/vision/target_valid

Type:

std_msgs/Bool


Meaning:

true

A valid target has been detected.


false

The vision node is running normally, but no valid elevator call button is currently detected.


Important:

false is different from publishing nothing.

If the vision node is alive but does not see a target, it must still publish:

target_valid = false


============================================================
5.2 Target Type
============================================================

Topic:

/vision/target_type

Type:

std_msgs/String


Required target class:

elevator_call_button


Recommended value when no target is detected:

none


The controller currently only accepts:

elevator_call_button


============================================================
5.3 Target Horizontal Position
============================================================

Topic:

/vision/target_center_x

Type:

std_msgs/Float32


The value represents the horizontal centre of the detected bounding box.

Current image width:

640 pixels

Image convention:

0 ---------------- 320 ---------------- 640
LEFT                 CENTER              RIGHT


Therefore:

target_center_x < 320

Target is on the left side of the image.


target_center_x approximately 320

Target is approximately centred.


target_center_x > 320

Target is on the right side of the image.


The controller uses this value only as an image-pixel coordinate.

It is NOT treated as a base-frame, odom-frame, or camera-frame 3D coordinate.


============================================================
5.4 Confidence
============================================================

Topic:

/vision/target_confidence

Type:

std_msgs/Float32

Expected range:

0.0 to 1.0

Current minimum accepted confidence:

0.60


============================================================
5.5 Vision Publishing Frequency
============================================================

All four topics should be published continuously while the vision node is running.

Recommended publishing frequency:

10 Hz or higher


Example when a button is detected:

target_valid = true
target_type = elevator_call_button
target_center_x = 415.0
target_confidence = 0.91


Example when no button is detected:

target_valid = false
target_type = none
target_center_x = 0.0
target_confidence = 0.0


This distinction is required by the controller.

Vision alive but no target:

SEARCH

Vision data stream missing or stale:

STOP


============================================================
6. Controller State Logic
============================================================

The controller follows the following state sequence:

WAIT_TARGET
     |
     v
SEARCH
     |
     v
TARGET ACQUIRED
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
     |
     v
STOP


============================================================
6.1 WAIT_TARGET
============================================================

Condition:

The vision data stream has not been received or has become stale.

Behaviour:

linear.x = 0
angular.z = 0

The robot stops.

This prevents the robot from rotating blindly if the camera, YOLO node, or ROS vision node is not operating.


============================================================
6.2 SEARCH
============================================================

Condition:

The vision system is running normally, but no valid elevator call button has ever been detected.

Behaviour:

linear.x = 0
angular.z = +0.25 rad/s

The Go1 performs a slow stepping rotation in place to search for the target.


Important:

The Unitree locomotion controller requires sufficient angular velocity to trigger stepping rotation.

The previously tested value of 0.15 rad/s caused the Go1 body to twist while the feet remained in stance.

The tested search value is therefore:

0.25 rad/s


During SEARCH:

The robot must NOT move forward.


============================================================
6.3 TARGET ACQUIRED
============================================================

When a valid elevator_call_button is detected for the first time:

The controller exits SEARCH.

The controller also clears depth information collected during SEARCH.

This is important because the depth camera may have observed unrelated walls or objects while the Go1 was rotating.

After target acquisition, the depth measurement must correspond to the direction of the detected target.


============================================================
6.4 ALIGN
============================================================

If the target is significantly away from the horizontal image centre, the Go1 rotates without moving forward.

Current realignment threshold:

80 pixels

During ALIGN:

linear.x = 0

Only angular correction is allowed.


============================================================
6.5 FAR_APPROACH
============================================================

Condition:

The visual target is valid and sufficiently aligned, but no valid depth measurement has ever been acquired.

This generally corresponds to the robot being outside the useful range of the Go1 front depth camera.

Current forward speed:

0.08 m/s

RGB target_center_x continues to provide steering correction.


============================================================
6.6 DEPTH_APPROACH
============================================================

Once a valid /front_distance measurement has been acquired, the controller enters DEPTH_APPROACH.

The depth measurement controls the forward approach.

RGB target_center_x continues to provide horizontal steering correction.

Current speeds:

If front_distance > 1.0 m:

linear speed = 0.08 m/s


If front_distance <= 1.0 m:

linear speed = 0.04 m/s


============================================================
6.7 READY
============================================================

Current demo stopping condition:

/front_distance <= 0.50 m

When this condition is reached:

linear.x = 0
angular.z = 0

The robot enters READY and stops.


The current 0.50 m value is only a demo interaction distance.

It is NOT a final robotic-arm working distance.

After the robotic arm is integrated, this value must be calibrated according to:

robotic-arm mounting position
robotic-arm workspace
camera mounting position
button position
safety margin


============================================================
6.8 VISION_LOST
============================================================

If the controller has already detected a valid target and the visual target is subsequently lost:

VISION_LOST
        |
        v
       STOP


The controller does NOT return automatically to SEARCH.

This is intentional.

The robot may already be close to the elevator panel, so automatically rotating after losing the target could be unsafe.


============================================================
6.9 DEPTH_LOST
============================================================

If valid depth has already been acquired and the depth stream later becomes stale:

DEPTH_LOST
       |
       v
      STOP


The controller does NOT return to FAR_APPROACH.

This prevents the Go1 from continuing forward after the depth sensor has failed.


============================================================
7. Important Coordinate and Measurement Rules
============================================================

The current controller intentionally separates visual direction and depth distance.


RGB vision:

target_center_x

This is an image-pixel coordinate.

It is used for horizontal steering only.


Depth:

/front_distance

This is a scalar distance measured using the front depth-camera ROI.

It is used for approach and stopping.


Do NOT directly subtract:

image pixel coordinates
base coordinates
odom coordinates
camera-frame 3D coordinates

unless an appropriate coordinate transformation has first been performed.


The current controller does not mix these coordinate systems.


============================================================
8. Controller Parameters
============================================================

Runtime controller parameters are stored in:

config/button_controller.yaml


Current parameters:

image_width: 640.0

center_threshold_px: 40.0

realign_threshold_px: 80.0

min_confidence: 0.60

vision_timeout: 0.5

depth_timeout: 0.5

max_valid_depth: 1.50

stop_distance: 0.50

search_angular_speed: 0.25

far_speed: 0.08

medium_speed: 0.08

near_speed: 0.04

kp_angular_align: 0.002

kp_angular_move: 0.001

max_align_angular: 0.35

max_move_angular: 0.12

yaw_sign: -1.0


These parameters can be changed without modifying and recompiling the C++ source code.


============================================================
9. Build Instructions
============================================================

Place this package under:

~/catkin_ws/src/


Expected path:

~/catkin_ws/src/go1_local_approach


Build:

cd ~/catkin_ws

catkin_make --pkg go1_local_approach -j2

source ~/catkin_ws/devel/setup.bash


-j2 is recommended for the tested environment to reduce memory usage.


Avoid running:

catkin_make clean

unless a complete workspace rebuild is intentionally required.

Cleaning the workspace removes previously compiled Unitree executables such as junior_ctrl.


============================================================
10. Unitree MOVE_BASE Requirement
============================================================

This project depends on Unitree MOVE_BASE control.

The tested control sequence is:

2 -> FixedStand

5 -> MOVE_BASE


After entering MOVE_BASE, /cmd_vel can control the Go1 through junior_ctrl.


The tested Unitree source required MOVE_BASE compilation to be enabled.

A patch is provided at:

patches/unitree_guide_move_base.patch


The important Unitree CMake configuration is:

add_definitions(-DCOMPILE_WITH_ROS)

add_definitions(-DCOMPILE_WITH_MOVE_BASE)


The patch modifies:

unitree_guide/CMakeLists.txt


The local KeyBoard.cpp contains an additional USERCMD debug print from development.

That debug print is NOT required for functionality and is intentionally not included in the final patch.


After applying the Unitree patch, rebuild the complete catkin workspace:

cd ~/catkin_ws

catkin_make -j2

source ~/catkin_ws/devel/setup.bash


Then verify:

Press 2

Go1 enters FixedStand.


Press 5

Go1 enters MOVE_BASE.


Do not continue with the local controller until MOVE_BASE has been verified.


============================================================
11. Go1 Simulation Startup
============================================================

A tested helper script is included:

scripts/start_go1_sim.sh


Run:

~/catkin_ws/src/go1_local_approach/scripts/start_go1_sim.sh


The script:

sources ROS Noetic

sources the catkin workspace

starts Gazebo and the Go1 model

waits for the Go1 joint controllers

runs junior_ctrl in the foreground


Important:

junior_ctrl must remain in the foreground because it requires keyboard input.


After Gazebo is ready:

Press 2

Then press 5


This enters:

FixedStand -> MOVE_BASE


============================================================
12. Runtime With Real Vision
============================================================

The final runtime launch file is:

launch/button_runtime.launch


This launch file does NOT start fake_button_vision.


The real ROS1 YOLO vision node must be started separately.


First verify that the vision system publishes:

rostopic echo /vision/target_valid

rostopic echo /vision/target_type

rostopic echo /vision/target_center_x

rostopic echo /vision/target_confidence


Then start:

roslaunch go1_local_approach button_runtime.launch


Expected operation:

Vision stream starts
        |
        v
No target
        |
        v
SEARCH
        |
        v
Button detected
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
        |
        v
STOP


============================================================
13. Self-Test Without Real YOLO
============================================================

A fake-vision testing launch is provided:

launch/button_test.launch


Run:

roslaunch go1_local_approach button_test.launch


This launch starts fake_button_vision automatically.


Do NOT use button_test.launch during the real YOLO demo.


Do NOT run fake_button_vision and the real vision node simultaneously.


============================================================
14. Runtime Interface Test Using Fake Vision
============================================================

To test the final runtime architecture without using the real YOLO node:

Start:

roslaunch go1_local_approach button_runtime.launch


Then start fake vision separately with the target disabled:

rosrun go1_local_approach fake_button_vision _enabled:=false


Expected controller behaviour:

SEARCH


Expected command:

linear.x = 0

angular.z = 0.25


Enable the simulated target:

rosparam set /fake_button_vision/enabled true


Expected sequence:

TARGET ACQUIRED

ALIGN

FAR_APPROACH

DEPTH_APPROACH

READY


============================================================
15. Verified Safety Behaviour
============================================================

The following behaviours have been tested successfully.


Case 1:

Vision node is not running.

Result:

WAIT_TARGET
STOP


Case 2:

Vision node is running but no valid button has been detected.

Result:

SEARCH


Case 3:

Valid button detected.

Result:

TARGET ACQUIRED
ALIGN
APPROACH


Case 4:

Visual target is lost after acquisition.

Result:

VISION_LOST
STOP


The controller must NOT return automatically to SEARCH.


Case 5:

Valid depth has been acquired and the depth node later fails.

Result:

DEPTH_LOST
STOP


The controller must NOT return to FAR_APPROACH.


Case 6:

front_distance <= stop_distance

Result:

READY
STOP


During SEARCH:

linear.x must equal 0.


During ALIGN:

linear.x must equal 0.


Only FAR_APPROACH and DEPTH_APPROACH may command forward motion.


============================================================
16. Useful Diagnostic Commands
============================================================

List vision topics:

rostopic list | grep vision


Inspect controller:

rosnode info /button_approach_controller


Inspect fake vision:

rosnode info /fake_button_vision


Monitor velocity commands:

rostopic echo /cmd_vel


Monitor front distance:

rostopic echo /front_distance


Check odometry:

rostopic echo -n 1 /odom


Check search angular speed:

rosparam get /button_approach_controller/search_angular_speed


Expected:

0.25


Check stop distance:

rosparam get /button_approach_controller/stop_distance


Expected:

0.5


Check depth image:

rostopic echo -n 1 /camera_face/depth/image_raw


============================================================
17. Important Launch Files
============================================================

button_runtime.launch

Final runtime configuration.

Uses real external vision.

Does NOT start fake vision.


button_test.launch

Development and self-test configuration.

Starts fake_button_vision.


camera_tf.launch

Legacy development TF configuration.

Not required by the current button runtime controller.


development.launch

Legacy development launch file.

Not used for the final button-approach demo.


============================================================
18. Legacy Development Source Files
============================================================

The src directory contains several nodes created during earlier development stages.

These files are retained for debugging and development history but are NOT required by the final runtime.

Examples include:

approach_controller.cpp

approach_controller_backup2.cpp

cmd_vel_test.cpp

fake_vision.cpp

goal_checker.cpp

task_manager.cpp

vision_to_goal.cpp

yaw_error_test.cpp

yaw_test.cpp


The final runtime primarily depends on:

button_approach_controller.cpp

depth_distance_node.cpp


Testing additionally uses:

fake_button_vision.cpp


============================================================
19. Vision Team Integration Checklist
============================================================

Before connecting the real vision system, verify the following:

ROS1 is being used.

The vision node publishes all four required topics.

The image width is 640 pixels, or the controller parameter is updated accordingly.

target_center_x increases from left to right.

The target class string is exactly:

elevator_call_button


Confidence is in the range:

0.0 to 1.0


All four topics continue publishing even when no target is detected.

Recommended publishing frequency is at least 10 Hz.

The real vision node and fake_button_vision are never running simultaneously.


============================================================
20. Handover Package Structure
============================================================

Expected package structure:

go1_local_approach/

CMakeLists.txt

package.xml

README.md

config/
    button_controller.yaml

docs/
    VISION_INTERFACE.md

launch/
    button_runtime.launch
    button_test.launch
    camera_tf.launch
    development.launch

patches/
    unitree_guide_move_base.patch

scripts/
    start_go1_sim.sh

src/
    button_approach_controller.cpp
    depth_distance_node.cpp
    fake_button_vision.cpp
    and legacy development source files


============================================================
21. Current Project Boundary
============================================================

Completed:

Go1 simulation

MOVE_BASE /cmd_vel control

Visual target search

Visual horizontal alignment

RGB-guided far approach

Go1 depth-camera distance extraction

Depth-guided near approach

0.50 m demo stopping

Vision-loss safety

Depth-loss safety

ROS1 visual interface specification


Not yet included:

Final trained YOLO model integration

Real Go1 hardware validation

Robotic arm control

Physical button pressing

Button-press verification

Elevator-door state verification

Global campus navigation


============================================================
22. Final Demo Objective
============================================================

The current local-control module is considered successful when the system performs:

Detect elevator call button
        |
        v
Search if necessary
        |
        v
Align Go1 with the target
        |
        v
Approach the elevator panel
        |
        v
Use front depth for near-range control
        |
        v
Stop at predefined interaction distance
        |
        v
READY FOR MANIPULATION


The robotic arm will be integrated after this stage.

End of README.