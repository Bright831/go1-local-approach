#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import torch
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32

from ultralytics import YOLO


class YoloButtonVision:
    def __init__(self):

        # ============================================================
        # Basic objects
        # ============================================================

        self.bridge = CvBridge()

        # ============================================================
        # ROS parameters
        # ============================================================

        default_model_path = os.path.join(
            os.path.dirname(
                os.path.dirname(__file__)
            ),
            "models/yolo/best.pt"
        )

        self.model_path = rospy.get_param(
            "~model_path",
            default_model_path
        )

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera_face/color/image_raw"
        )

        self.conf_threshold = float(
            rospy.get_param(
                "~conf_threshold",
                0.60
            )
        )

        self.controller_image_width = float(
            rospy.get_param(
                "~controller_image_width",
                640.0
            )
        )

        self.imgsz = int(
            rospy.get_param(
                "~imgsz",
                640
            )
        )

        self.device = str(
            rospy.get_param(
                "~device",
                "0"
            )
        )

        self.target_class_name = rospy.get_param(
            "~target_class_name",
            "elevator_call_button"
        )

        self.publish_debug = bool(
            rospy.get_param(
                "~publish_debug",
                True
            )
        )

        # ============================================================
        # Load YOLO model
        # ============================================================

        rospy.loginfo(
            "Loading YOLO model: %s",
            self.model_path
        )

        try:
            self.model = YOLO(
                self.model_path
            )

        except Exception as e:
            rospy.logfatal(
                "Failed to load YOLO model: %s",
                str(e)
            )
            raise

        rospy.loginfo(
            "YOLO model loaded successfully"
        )

        rospy.loginfo(
            "Image topic: %s",
            self.image_topic
        )

        rospy.loginfo(
            "Confidence threshold: %.2f",
            self.conf_threshold
        )

        rospy.loginfo(
            "Controller image width: %.1f",
            self.controller_image_width
        )

        rospy.loginfo(
            "Inference image size: %d",
            self.imgsz
        )

        rospy.loginfo(
            "CUDA available: %s",
            str(torch.cuda.is_available())
        )

        if torch.cuda.is_available():
            rospy.loginfo(
                "GPU: %s",
                torch.cuda.get_device_name(0)
            )

        # ============================================================
        # Publishers
        # ============================================================

        self.valid_pub = rospy.Publisher(
            "/vision/target_valid",
            Bool,
            queue_size=1
        )

        self.type_pub = rospy.Publisher(
            "/vision/target_type",
            String,
            queue_size=1
        )

        self.center_x_pub = rospy.Publisher(
            "/vision/target_center_x",
            Float32,
            queue_size=1
        )

        self.confidence_pub = rospy.Publisher(
            "/vision/target_confidence",
            Float32,
            queue_size=1
        )

        self.debug_image_pub = rospy.Publisher(
            "/vision/debug_image",
            Image,
            queue_size=1
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.loginfo(
            "YOLO button vision node started"
        )

    # ================================================================
    # Publish "no target"
    # ================================================================

    def publish_no_target(self):

        self.valid_pub.publish(
            Bool(data=False)
        )

        self.type_pub.publish(
            String(data="none")
        )

        self.center_x_pub.publish(
            Float32(data=0.0)
        )

        self.confidence_pub.publish(
            Float32(data=0.0)
        )

    # ================================================================
    # Image callback
    # ================================================================

    def image_callback(self, msg):

        # ------------------------------------------------------------
        # ROS Image -> OpenCV BGR image
        # ------------------------------------------------------------

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except CvBridgeError as e:

            rospy.logerr_throttle(
                1.0,
                "CvBridge input conversion error: %s",
                str(e)
            )

            self.publish_no_target()
            return

        except Exception as e:

            rospy.logerr_throttle(
                1.0,
                "Unexpected image conversion error: %s",
                str(e)
            )

            self.publish_no_target()
            return

        # Make sure image is standard uint8 BGR contiguous memory
        image = np.ascontiguousarray(
            image,
            dtype=np.uint8
        )

        image_height, image_width = image.shape[:2]

        # ------------------------------------------------------------
        # YOLO inference
        # ------------------------------------------------------------

        try:
            results = self.model.predict(
                source=image,
                imgsz=self.imgsz,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False
            )

        except Exception as e:

            rospy.logerr_throttle(
                1.0,
                "YOLO inference error: %s",
                str(e)
            )

            self.publish_no_target()

            if self.publish_debug:
                self.publish_debug_image(
                    image,
                    msg
                )

            return

        if not results:

            self.publish_no_target()

            if self.publish_debug:
                self.publish_debug_image(
                    image,
                    msg
                )

            return

        result = results[0]

        # ------------------------------------------------------------
        # Find highest-confidence class-0 detection
        # ------------------------------------------------------------

        best_confidence = -1.0
        best_box = None

        if result.boxes is not None:

            for box in result.boxes:

                class_id = int(
                    box.cls[0].item()
                )

                confidence = float(
                    box.conf[0].item()
                )

                # Our model has only one class:
                #
                # class 0 = elevator_call_button

                if class_id != 0:
                    continue

                if confidence > best_confidence:

                    best_confidence = confidence

                    best_box = (
                        box.xyxy[0]
                        .detach()
                        .cpu()
                        .numpy()
                    )

        # ------------------------------------------------------------
        # No valid detection
        # ------------------------------------------------------------

        if best_box is None:

            self.publish_no_target()

            rospy.loginfo_throttle(
                1.0,
                "YOLO: no valid button"
            )

            if self.publish_debug:

                self.publish_debug_image(
                    image,
                    msg
                )

            return

        # ------------------------------------------------------------
        # Valid detection
        # ------------------------------------------------------------

        x1, y1, x2, y2 = best_box

        raw_center_x = (
            float(x1) + float(x2)
        ) / 2.0

        # ------------------------------------------------------------
        # Coordinate scaling
        #
        # IMPORTANT:
        #
        # YOLO center_x is an IMAGE PIXEL coordinate.
        #
        # The C++ controller assumes:
        #
        # image width = 640
        # image center = 320
        #
        # So if Gazebo camera width is not 640, convert proportionally.
        #
        # This is NOT a robot/base/odom coordinate.
        # ------------------------------------------------------------

        scaled_center_x = (
            raw_center_x
            * self.controller_image_width
            / float(image_width)
        )

        # Clamp for safety
        scaled_center_x = max(
            0.0,
            min(
                self.controller_image_width,
                scaled_center_x
            )
        )

        # ------------------------------------------------------------
        # Publish standard vision interface
        # ------------------------------------------------------------

        self.valid_pub.publish(
            Bool(data=True)
        )

        self.type_pub.publish(
            String(
                data=self.target_class_name
            )
        )

        self.center_x_pub.publish(
            Float32(
                data=scaled_center_x
            )
        )

        self.confidence_pub.publish(
            Float32(
                data=best_confidence
            )
        )

        # ------------------------------------------------------------
        # Debug information
        # ------------------------------------------------------------

        rospy.loginfo_throttle(
            0.5,
            "YOLO BUTTON | "
            "conf=%.3f | "
            "raw_x=%.1f/%d | "
            "controller_x=%.1f/640",
            best_confidence,
            raw_center_x,
            image_width,
            scaled_center_x
        )

        # ------------------------------------------------------------
        # Draw detection for debug image
        # ------------------------------------------------------------

        if self.publish_debug:

            debug_image = image.copy()

            x1_i = int(
                max(
                    0,
                    min(
                        image_width - 1,
                        x1
                    )
                )
            )

            y1_i = int(
                max(
                    0,
                    min(
                        image_height - 1,
                        y1
                    )
                )
            )

            x2_i = int(
                max(
                    0,
                    min(
                        image_width - 1,
                        x2
                    )
                )
            )

            y2_i = int(
                max(
                    0,
                    min(
                        image_height - 1,
                        y2
                    )
                )
            )

            cv2.rectangle(
                debug_image,
                (x1_i, y1_i),
                (x2_i, y2_i),
                (0, 255, 0),
                2
            )

            label_text = (
                "{} {:.2f}".format(
                    self.target_class_name,
                    best_confidence
                )
            )

            text_y = max(
                y1_i - 10,
                20
            )

            cv2.putText(
                debug_image,
                label_text,
                (x1_i, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA
            )

            # Draw raw image center
            cv2.line(
                debug_image,
                (
                    int(image_width / 2),
                    0
                ),
                (
                    int(image_width / 2),
                    image_height - 1
                ),
                (255, 0, 0),
                1
            )

            # Draw detected target center
            cv2.circle(
                debug_image,
                (
                    int(raw_center_x),
                    int(
                        (
                            float(y1)
                            + float(y2)
                        ) / 2.0
                    )
                ),
                5,
                (0, 0, 255),
                -1
            )

            self.publish_debug_image(
                debug_image,
                msg
            )

    # ================================================================
    # OpenCV -> ROS Image
    #
    # IMPORTANT:
    #
    # We intentionally DO NOT use:
    #
    # self.bridge.cv2_to_imgmsg(...)
    #
    # because the current ROS Noetic cv_bridge and the pip-installed
    # OpenCV version produce:
    #
    # KeyError: 16
    #
    # So we construct sensor_msgs/Image manually.
    # ================================================================

    def publish_debug_image(
        self,
        image,
        original_msg
    ):

        try:

            image = np.ascontiguousarray(
                image,
                dtype=np.uint8
            )

            height, width = image.shape[:2]

            debug_msg = Image()

            debug_msg.header = (
                original_msg.header
            )

            debug_msg.height = height
            debug_msg.width = width

            debug_msg.encoding = "bgr8"

            debug_msg.is_bigendian = 0

            # BGR8 = 3 bytes per pixel
            debug_msg.step = (
                width * 3
            )

            debug_msg.data = (
                image.tobytes()
            )

            self.debug_image_pub.publish(
                debug_msg
            )

        except Exception as e:

            rospy.logwarn_throttle(
                1.0,
                "Debug image publish failed: %s",
                str(e)
            )


def main():

    rospy.init_node(
        "yolo_button_vision"
    )

    try:

        YoloButtonVision()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    except Exception as e:

        rospy.logfatal(
            "YOLO button vision node fatal error: %s",
            str(e)
        )

        raise


if __name__ == "__main__":
    main()