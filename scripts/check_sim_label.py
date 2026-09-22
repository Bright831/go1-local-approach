#!/usr/bin/env python3

import os
import glob
import cv2


dataset = os.path.expanduser(
    "~/go1_sim_vision_dataset/raw"
)

images = sorted(
    glob.glob(
        os.path.join(
            dataset,
            "images",
            "*.jpg"
        )
    )
)

for image_path in images:
    base = os.path.splitext(
        os.path.basename(
            image_path
        )
    )[0]

    label_path = os.path.join(
        dataset,
        "labels",
        base + ".txt"
    )

    image = cv2.imread(
        image_path
    )

    height, width = image.shape[:2]

    if os.path.exists(label_path):
        with open(label_path) as f:
            lines = f.readlines()

        for line in lines:
            values = line.strip().split()

            if len(values) != 5:
                continue

            _, cx, cy, nw, nh = values

            cx = float(cx)
            cy = float(cy)
            nw = float(nw)
            nh = float(nh)

            box_w = nw * width
            box_h = nh * height

            x1 = int(
                cx * width
                - box_w / 2
            )

            y1 = int(
                cy * height
                - box_h / 2
            )

            x2 = int(
                cx * width
                + box_w / 2
            )

            y2 = int(
                cy * height
                + box_h / 2
            )

            cv2.rectangle(
                image,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

    cv2.imshow(
        "YOLO Label Check",
        image
    )

    print(
        image_path
    )

    key = cv2.waitKey(0)

    if key == 27 or key == ord("q"):
        break


cv2.destroyAllWindows()
