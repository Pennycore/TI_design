#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault(
    "YOLO_CONFIG_DIR",
    str(Path("vision/.ultralytics").resolve()),
)

from ultralytics import YOLO


def write_yolo_labels(label_path, boxes):
    lines = []
    if boxes is not None:
        for box in boxes.xywhn.cpu().tolist():
            x, y, width, height = box
            lines.append("0 %.6f %.6f %.6f %.6f" % (x, y, width, height))
    label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="ascii")


def main():
    parser = argparse.ArgumentParser(
        description="Pre-label clean K230 hard-case frames with the current model."
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=Path("vision/models/steel_ball_yolo11n.pt"),
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("vision/hard_cases/review_manifest.csv"),
    )
    parser.add_argument(
        "--labels",
        type=Path,
        default=Path("vision/hard_cases/review/labels"),
    )
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--conf", type=float, default=0.15)
    args = parser.parse_args()

    args.labels.mkdir(parents=True, exist_ok=True)
    positive_rows = []
    with args.manifest.open("r", encoding="utf-8", newline="") as manifest:
        for row in csv.DictReader(manifest):
            if row["negative"] == "0":
                positive_rows.append(row)

    model = YOLO(str(args.weights))
    image_paths = [row["image"] for row in positive_rows]
    results = model.predict(
        source=image_paths,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=0.55,
        max_det=10,
        device=0,
        verbose=False,
        stream=True,
    )

    image_count = 0
    box_count = 0
    empty_count = 0
    for row, result in zip(positive_rows, results):
        label_path = args.labels / (Path(row["image"]).stem + ".txt")
        write_yolo_labels(label_path, result.boxes)
        count = 0 if result.boxes is None else len(result.boxes)
        image_count += 1
        box_count += count
        if count == 0:
            empty_count += 1

    print("Pre-labeled positive candidates:", image_count)
    print("Generated boxes:", box_count)
    print("Images requiring boxes from scratch:", empty_count)
    print("All labels must be reviewed manually before training.")


if __name__ == "__main__":
    main()
