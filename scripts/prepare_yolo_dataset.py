#!/usr/bin/env python3

import os
import shutil
from pathlib import Path


SOURCE_ROOT = Path(
    os.path.expanduser(
        "~/go1_sim_vision_dataset/sessions"
    )
)

OUTPUT_ROOT = Path(
    os.path.expanduser(
        "~/go1_sim_vision_dataset/yolo"
    )
)


TRAIN_SESSIONS = [
    "S01_far_center",
    "S02_mid_center",
    "S04_target_left",
    "S05_target_right",
    "S06_edge_left",
    "S08_mixed",
    "S09_negative_no_target",
]


VAL_SESSIONS = [
    "S03_near_center",
    "S07_edge_right",
    "S10_negative_elevator",
]


def prepare_directories():
    if OUTPUT_ROOT.exists():
        print(
            f"Removing existing output directory: "
            f"{OUTPUT_ROOT}"
        )

        shutil.rmtree(
            OUTPUT_ROOT
        )

    for split in [
        "train",
        "val"
    ]:
        (
            OUTPUT_ROOT
            / "images"
            / split
        ).mkdir(
            parents=True,
            exist_ok=True
        )

        (
            OUTPUT_ROOT
            / "labels"
            / split
        ).mkdir(
            parents=True,
            exist_ok=True
        )


def copy_session(
    session_name,
    split
):
    session_dir = (
        SOURCE_ROOT
        / session_name
    )

    images_dir = (
        session_dir
        / "images"
    )

    labels_dir = (
        session_dir
        / "labels"
    )

    if not images_dir.exists():
        raise RuntimeError(
            f"Missing images directory: "
            f"{images_dir}"
        )

    if not labels_dir.exists():
        raise RuntimeError(
            f"Missing labels directory: "
            f"{labels_dir}"
        )

    images = sorted(
        images_dir.glob(
            "*.jpg"
        )
    )

    positive = 0
    negative = 0

    for image_path in images:
        label_path = (
            labels_dir
            / (
                image_path.stem
                + ".txt"
            )
        )

        if not label_path.exists():
            raise RuntimeError(
                f"Missing label for "
                f"{image_path}"
            )

        # Prefix with session name
        # to guarantee unique filenames.
        new_base = (
            session_name
            + "__"
            + image_path.stem
        )

        output_image = (
            OUTPUT_ROOT
            / "images"
            / split
            / (
                new_base
                + ".jpg"
            )
        )

        output_label = (
            OUTPUT_ROOT
            / "labels"
            / split
            / (
                new_base
                + ".txt"
            )
        )

        shutil.copy2(
            image_path,
            output_image
        )

        shutil.copy2(
            label_path,
            output_label
        )

        if label_path.stat().st_size == 0:
            negative += 1
        else:
            positive += 1

    return (
        len(images),
        positive,
        negative
    )


def prepare_split(
    sessions,
    split
):
    total = 0
    positive = 0
    negative = 0

    print()
    print(
        f"===== {split.upper()} ====="
    )

    for session in sessions:
        count, pos, neg = (
            copy_session(
                session,
                split
            )
        )

        total += count
        positive += pos
        negative += neg

        print(
            f"{session}: "
            f"total={count}, "
            f"positive={pos}, "
            f"negative={neg}"
        )

    print(
        f"{split.upper()} TOTAL: "
        f"{total}"
    )

    print(
        f"{split.upper()} POSITIVE: "
        f"{positive}"
    )

    print(
        f"{split.upper()} NEGATIVE: "
        f"{negative}"
    )

    return (
        total,
        positive,
        negative
    )


def write_yaml():
    yaml_path = (
        OUTPUT_ROOT
        / "data.yaml"
    )

    content = f"""path: {OUTPUT_ROOT}
train: images/train
val: images/val

names:
  0: elevator_call_button
"""

    yaml_path.write_text(
        content
    )

    print()
    print(
        f"Created: {yaml_path}"
    )


def main():
    print(
        "Preparing YOLO dataset..."
    )

    print(
        f"Source: {SOURCE_ROOT}"
    )

    print(
        f"Output: {OUTPUT_ROOT}"
    )

    prepare_directories()

    train_stats = prepare_split(
        TRAIN_SESSIONS,
        "train"
    )

    val_stats = prepare_split(
        VAL_SESSIONS,
        "val"
    )

    write_yaml()

    print()
    print(
        "===== FINAL SUMMARY ====="
    )

    print(
        f"Train images: "
        f"{train_stats[0]}"
    )

    print(
        f"Train positive: "
        f"{train_stats[1]}"
    )

    print(
        f"Train negative: "
        f"{train_stats[2]}"
    )

    print()

    print(
        f"Val images: "
        f"{val_stats[0]}"
    )

    print(
        f"Val positive: "
        f"{val_stats[1]}"
    )

    print(
        f"Val negative: "
        f"{val_stats[2]}"
    )

    print()

    print(
        "YOLO dataset preparation "
        "completed."
    )


if __name__ == "__main__":
    main()
