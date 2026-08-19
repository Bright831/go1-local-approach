# Vision Interface Specification

The button approach controller expects a ROS1 vision node to publish
the following four topics continuously.

## 1. Target valid

Topic:

/vision/target_valid

Message type:

std_msgs/Bool

Meaning:

true  = valid target detected
false = vision system is alive but no valid target is detected


## 2. Target type

Topic:

/vision/target_type

Message type:

std_msgs/String

Required button class:

elevator_call_button

When no target is detected, recommended value:

none


## 3. Target horizontal position

Topic:

/vision/target_center_x

Message type:

std_msgs/Float32

Image convention:

0 ---------------- 320 ---------------- 640
LEFT                 CENTER              RIGHT

Current controller assumes:

image_width = 640 pixels

The value must be the horizontal center of the detected bounding box.


## 4. Confidence

Topic:

/vision/target_confidence

Message type:

std_msgs/Float32

Range:

0.0 - 1.0

Current minimum accepted confidence:

0.60


# Publishing requirement

All four topics must continue to be published while the vision node
is running.

Recommended frequency:

10 Hz or higher.

Example when button is detected:

target_valid      = true
target_type       = elevator_call_button
target_center_x   = 415.0
target_confidence = 0.91

Example when no button is detected:

target_valid      = false
target_type       = none
target_center_x   = 0.0
target_confidence = 0.0


# Controller behavior

Vision stream missing/stale:
STOP

Vision alive but target never detected:
SEARCH

Valid target detected:
ALIGN / APPROACH

Target lost after acquisition:
STOP
