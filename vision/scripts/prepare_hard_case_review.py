#!/usr/bin/env python3
import argparse
import csv
import shutil
from pathlib import Path

import cv2


def count_overlay_pixels(image_path):
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise RuntimeError("Cannot read image: %s" % image_path)

    blue, green, red = cv2.split(frame)
    green_overlay = (
        (green > 150)
        & (green > red * 1.35)
        & (green > blue * 1.35)
    )
    red_overlay = (
        (red > 145)
        & (red > green * 1.35)
        & (red > blue * 1.15)
    )
    orange_overlay = (
        (red > 180)
        & (green > 65)
        & (green < 205)
        & (blue < 100)
    )
    return int((green_overlay | red_overlay | orange_overlay).sum())


def main():
    parser = argparse.ArgumentParser(
        description="Keep negative frames and positive frames without burned-in OSD."
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("vision/hard_cases/manifest.csv"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("vision/hard_cases/review"),
    )
    parser.add_argument("--max-overlay-pixels", type=int, default=20)
    args = parser.parse_args()

    output_dir = args.output.resolve()
    image_dir = output_dir / "images"
    label_dir = output_dir / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)

    selected_rows = []
    with args.manifest.resolve().open("r", encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            image_path = Path(row["image"])
            negative = row["negative"] == "1"
            overlay_pixels = 0 if negative else count_overlay_pixels(image_path)
            if not negative and overlay_pixels >= args.max_overlay_pixels:
                continue

            target_image = image_dir / image_path.name
            shutil.copy2(image_path, target_image)
            if negative:
                (label_dir / (target_image.stem + ".txt")).write_text(
                    "", encoding="ascii"
                )

            selected = dict(row)
            selected["image"] = target_image.as_posix()
            selected["overlay_pixels"] = overlay_pixels
            selected_rows.append(selected)

    review_manifest = output_dir.parent / "review_manifest.csv"
    with review_manifest.open("w", encoding="utf-8", newline="") as target:
        fieldnames = list(selected_rows[0].keys())
        writer = csv.DictWriter(target, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(selected_rows)

    negative_count = sum(row["negative"] == "1" for row in selected_rows)
    print("Review images:", len(selected_rows))
    print("Negative images:", negative_count)
    print("Clean positive candidates:", len(selected_rows) - negative_count)
    print("Review manifest:", review_manifest)


if __name__ == "__main__":
    main()
