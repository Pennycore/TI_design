#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import cv2


VIDEO_SUFFIXES = {".mp4", ".avi", ".mov", ".mkv"}
NEGATIVE_MARKERS = ("backgrd", "background", "empty", "no_ball", "noball")


def frame_signature(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA)


def mean_frame_difference(first, second):
    return float(cv2.absdiff(first, second).mean())


def is_negative_video(path):
    name = path.stem.lower()
    return any(marker in name for marker in NEGATIVE_MARKERS)


def extract_video(video_path, image_dir, label_dir, target_fps, min_difference):
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError("Cannot open video: %s" % video_path)

    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if source_fps <= 0:
        source_fps = 30.0
    sample_step = max(1, int(round(source_fps / target_fps)))
    negative = is_negative_video(video_path)
    kept = []
    previous_signature = None
    frame_index = 0

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % sample_step != 0:
            frame_index += 1
            continue

        signature = frame_signature(frame)
        if (
            previous_signature is not None
            and mean_frame_difference(signature, previous_signature) < min_difference
        ):
            frame_index += 1
            continue

        timestamp_ms = capture.get(cv2.CAP_PROP_POS_MSEC)
        filename = "%s_f%06d_t%07d.jpg" % (
            video_path.stem,
            frame_index,
            int(timestamp_ms),
        )
        image_path = image_dir / filename
        if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError("Cannot write image: %s" % image_path)

        if negative:
            (label_dir / (image_path.stem + ".txt")).write_text("", encoding="ascii")

        kept.append(
            {
                "image": image_path.as_posix(),
                "source_video": video_path.name,
                "frame_index": frame_index,
                "timestamp_ms": int(timestamp_ms),
                "negative": int(negative),
            }
        )
        previous_signature = signature
        frame_index += 1

    capture.release()
    return kept


def main():
    parser = argparse.ArgumentParser(
        description="Extract diverse frames from K230 hard-case videos."
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("vision/hard_cases"))
    parser.add_argument(
        "--include",
        default="*",
        help="Filename glob used to select videos, for example clean_*.mp4.",
    )
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--min-difference", type=float, default=1.5)
    args = parser.parse_args()

    input_dir = args.input.resolve()
    output_dir = args.output.resolve()
    image_dir = output_dir / "images"
    label_dir = output_dir / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)

    videos = sorted(
        path
        for path in input_dir.rglob("*")
        if (
            path.is_file()
            and path.suffix.lower() in VIDEO_SUFFIXES
            and path.match(args.include)
        )
    )
    if not videos:
        raise FileNotFoundError("No videos found in %s" % input_dir)

    rows = []
    for video_path in videos:
        extracted = extract_video(
            video_path,
            image_dir,
            label_dir,
            args.fps,
            args.min_difference,
        )
        rows.extend(extracted)
        print(
            "%s: %d frames%s"
            % (
                video_path.name,
                len(extracted),
                " (negative)" if is_negative_video(video_path) else "",
            )
        )

    manifest_path = output_dir / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as manifest:
        writer = csv.DictWriter(
            manifest,
            fieldnames=(
                "image",
                "source_video",
                "frame_index",
                "timestamp_ms",
                "negative",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)

    negative_count = sum(row["negative"] for row in rows)
    print("Extracted %d frames" % len(rows))
    print("Negative frames with empty labels: %d" % negative_count)
    print("Manifest:", manifest_path)


if __name__ == "__main__":
    main()
