#!/usr/bin/env python3

import os
import time

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class SimDatasetCollector:
    def __init__(self):
        self.bridge = CvBridge()

        self.output_dir = os.path.expanduser(
            rospy.get_param(
                "~output_dir",
                "~/go1_sim_vision_dataset/raw"
            )
        )

        self.capture_rate = rospy.get_param(
            "~capture_rate",
            2.0
        )

        self.min_button_area = rospy.get_param(
            "~min_button_area",
            5.0
        )

        self.save_negatives = rospy.get_param(
            "~save_negatives",
            True
        )

        self.images_dir = os.path.join(
            self.output_dir,
            "images"
        )

        self.labels_dir = os.path.join(
            self.output_dir,
            "labels"
        )

        os.makedirs(
            self.images_dir,
            exist_ok=True
        )

        os.makedirs(
            self.labels_dir,
            exist_ok=True
        )

        self.last_capture_time = 0.0
        self.counter = 0
        self.positive_count = 0
        self.negative_count = 0

        self.sub = rospy.Subscriber(
            "/camera_face/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo(
            "Synthetic dataset collector started"
        )

        rospy.loginfo(
            "Output directory: %s",
            self.output_dir
        )

        rospy.loginfo(
            "Capture rate: %.2f Hz",
            self.capture_rate
        )


    def detect_magenta_button(self, image):
        """
        Detect the synthetic magenta elevator button.

        Returns:
            bbox = (x, y, w, h)
            or None
        """

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )

        # Magenta / purple range.
        lower_magenta = np.array(
            [135, 70, 60],
            dtype=np.uint8
        )

        upper_magenta = np.array(
            [175, 255, 255],
            dtype=np.uint8
        )

        mask = cv2.inRange(
            hsv,
            lower_magenta,
            upper_magenta
        )

        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.dilate(
            mask,
            kernel,
            iterations=1
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(
            largest
        )

        if area < self.min_button_area:
            return None

        x, y, w, h = cv2.boundingRect(
            largest
        )

        # Small padding.
        padding = 2

        x = max(
            0,
            x - padding
        )

        y = max(
            0,
            y - padding
        )

        w = min(
            image.shape[1] - x,
            w + 2 * padding
        )

        h = min(
            image.shape[0] - y,
            h + 2 * padding
        )

        return x, y, w, h


    def write_yolo_label(
        self,
        label_path,
        bbox,
        image_width,
        image_height
    ):
        if bbox is None:
            # Empty YOLO label = negative image.
            open(
                label_path,
                "w"
            ).close()

            return

        x, y, w, h = bbox

        center_x = (
            x + w / 2.0
        ) / image_width

        center_y = (
            y + h / 2.0
        ) / image_height

        norm_w = (
            w / image_width
        )

        norm_h = (
            h / image_height
        )

        with open(
            label_path,
            "w"
        ) as f:

            # class 0 = elevator_call_button
            f.write(
                "0 "
                f"{center_x:.6f} "
                f"{center_y:.6f} "
                f"{norm_w:.6f} "
                f"{norm_h:.6f}\n"
            )


    def image_callback(self, msg):
        current_time = time.time()

        interval = 1.0 / self.capture_rate

        if (
            current_time
            - self.last_capture_time
            < interval
        ):
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except CvBridgeError as e:
            rospy.logerr(
                "CvBridge error: %s",
                str(e)
            )

            return

        bbox = self.detect_magenta_button(
            image
        )

        # If no target and negative images
        # are disabled, skip this frame.
        if (
            bbox is None
            and not self.save_negatives
        ):
            return

        timestamp = int(
            time.time() * 1000
        )

        base_name = (
            f"sim_{timestamp}_{self.counter:06d}"
        )

        image_path = os.path.join(
            self.images_dir,
            base_name + ".jpg"
        )

        label_path = os.path.join(
            self.labels_dir,
            base_name + ".txt"
        )

        cv2.imwrite(
            image_path,
            image
        )

        height, width = image.shape[:2]

        self.write_yolo_label(
            label_path,
            bbox,
            width,
            height
        )

        if bbox is None:
            self.negative_count += 1

            state = "NEGATIVE"

        else:
            self.positive_count += 1

            state = "POSITIVE"

        self.counter += 1
        self.last_capture_time = current_time

        rospy.loginfo(
            "Saved %s | "
            "total=%d positive=%d negative=%d",
            state,
            self.counter,
            self.positive_count,
            self.negative_count
        )


def main():
    rospy.init_node(
        "capture_sim_dataset"
    )

    SimDatasetCollector()

    rospy.spin()


if __name__ == "__main__":
    main()
