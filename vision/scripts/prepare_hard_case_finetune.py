#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
FRAME_PATTERN = re.compile(r"^(?P<source>.+)_f(?P<frame>\d+)_t(?P<time>\d+)$")


@dataclass(frozen=True)
class Sample:
    image: Path
    label: Path
    source: str
    frame: int
    timestamp_ms: int
    boxes: int


def parse_name(image: Path) -> tuple[str, int, int]:
    match = FRAME_PATTERN.match(image.stem)
    if not match:
        return image.stem, 0, 0
    return (
        match.group("source"),
        int(match.group("frame")),
        int(match.group("time")),
    )


def count_and_validate_boxes(label: Path) -> int:
    if not label.exists():
        raise FileNotFoundError(f"Missing label for {label.stem}: {label}")

    boxes = 0
    for line_number, raw_line in enumerate(label.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{label}:{line_number}: expected 5 fields, got {len(fields)}")
        class_id = int(fields[0])
        coords = [float(value) for value in fields[1:]]
        if class_id != 0:
            raise ValueError(f"{label}:{line_number}: only class 0 is supported")
        if any(value < 0.0 or value > 1.0 for value in coords):
            raise ValueError(f"{label}:{line_number}: coordinates must be in [0, 1]")
        if coords[2] <= 0.0 or coords[3] <= 0.0:
            raise ValueError(f"{label}:{line_number}: box width and height must be positive")
        boxes += 1
    return boxes


def collect_samples(review: Path) -> list[Sample]:
    image_dir = review / "images"
    label_dir = review / "labels"
    images = sorted(
        path for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not images:
        raise RuntimeError(f"No images found under {image_dir}")

    samples = []
    for image in images:
        label = label_dir / f"{image.stem}.txt"
        source, frame, timestamp_ms = parse_name(image)
        samples.append(
            Sample(
                image=image,
                label=label,
                source=source,
                frame=frame,
                timestamp_ms=timestamp_ms,
                boxes=count_and_validate_boxes(label),
            )
        )
    return samples


def temporal_split(
    samples: list[Sample], val_ratio: float, min_group_for_val: int
) -> tuple[list[Sample], list[Sample]]:
    grouped = defaultdict(list)
    for sample in samples:
        grouped[sample.source].append(sample)

    train = []
    val = []
    for source in sorted(grouped):
        group = sorted(grouped[source], key=lambda item: (item.timestamp_ms, item.frame, item.image.name))
        if len(group) < min_group_for_val:
            train.extend(group)
            continue
        val_count = min(len(group) - 1, max(1, math.ceil(len(group) * val_ratio)))
        train.extend(group[:-val_count])
        val.extend(group[-val_count:])
    return train, val


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def materialize(
    samples: list[Sample], out: Path, split: str, repeats: int, manifest_rows: list[dict]
) -> None:
    for sample in samples:
        for repeat_index in range(repeats):
            repeat_suffix = "" if repeat_index == 0 else f"__r{repeat_index + 1}"
            output_stem = f"{sample.image.stem}{repeat_suffix}"
            output_image = out / "images" / split / f"{output_stem}{sample.image.suffix.lower()}"
            output_label = out / "labels" / split / f"{output_stem}.txt"
            link_or_copy(sample.image, output_image)
            output_label.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(sample.label, output_label)
            manifest_rows.append(
                {
                    "split": split,
                    "source": sample.source,
                    "frame": sample.frame,
                    "timestamp_ms": sample.timestamp_ms,
                    "repeat": repeat_index + 1,
                    "boxes": sample.boxes,
                    "source_image": sample.image.as_posix(),
                    "output_image": output_image.as_posix(),
                }
            )


def reset_generated_dirs(out: Path) -> None:
    for relative in ("images/train", "images/val", "labels/train", "labels/val"):
        path = out / relative
        if path.exists():
            shutil.rmtree(path)


def write_data_yamls(out: Path, vision_root: Path) -> tuple[Path, Path]:
    relative_out = out.resolve().relative_to(vision_root.resolve()).as_posix()
    data_yaml = out / "data.yaml"
    data_yaml.write_text(
        "\n".join(
            [
                f"path: {vision_root.resolve().as_posix()}",
                "train:",
                "  - dataset/images/train",
                f"  - {relative_out}/images/train",
                "val:",
                "  - dataset/images/val",
                f"  - {relative_out}/images/val",
                "names:",
                "  0: steel_ball",
                "",
            ]
        ),
        encoding="utf-8",
    )
    hard_val_yaml = out / "hard_val.yaml"
    hard_val_yaml.write_text(
        "\n".join(
            [
                f"path: {vision_root.resolve().as_posix()}",
                f"train: {relative_out}/images/train",
                f"val: {relative_out}/images/val",
                "names:",
                "  0: steel_ball",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return data_yaml, hard_val_yaml


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Prepare reviewed K230 hard cases for YOLO fine-tuning."
    )
    parser.add_argument("--review", default="vision/hard_cases/review")
    parser.add_argument("--out", default="vision/hard_cases/finetune")
    parser.add_argument("--vision-root", default="vision")
    parser.add_argument("--val-ratio", type=float, default=0.20)
    parser.add_argument("--min-group-for-val", type=int, default=5)
    parser.add_argument(
        "--train-repeat",
        type=int,
        default=3,
        help="Repeat hard-case training images so they are not drowned out by the base dataset.",
    )
    args = parser.parse_args()

    if not 0.0 < args.val_ratio < 0.5:
        raise ValueError("--val-ratio must be between 0 and 0.5")
    if args.train_repeat < 1:
        raise ValueError("--train-repeat must be at least 1")

    review = Path(args.review).resolve()
    out = Path(args.out).resolve()
    vision_root = Path(args.vision_root).resolve()
    if not (vision_root / "dataset/images/train").is_dir():
        raise RuntimeError(f"Base training images not found under {vision_root / 'dataset/images/train'}")
    out.relative_to(vision_root)

    samples = collect_samples(review)
    train, val = temporal_split(samples, args.val_ratio, args.min_group_for_val)
    reset_generated_dirs(out)

    manifest_rows = []
    materialize(train, out, "train", args.train_repeat, manifest_rows)
    materialize(val, out, "val", 1, manifest_rows)

    out.mkdir(parents=True, exist_ok=True)
    manifest = out / "split_manifest.csv"
    with manifest.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=manifest_rows[0].keys())
        writer.writeheader()
        writer.writerows(manifest_rows)
    data_yaml, hard_val_yaml = write_data_yamls(out, vision_root)

    print(f"Reviewed samples: {len(samples)}")
    print(f"Hard-case train: {len(train)} unique, {len(train) * args.train_repeat} after repeat")
    print(f"Hard-case val:   {len(val)}")
    print(f"Boxes:           {sum(sample.boxes for sample in samples)}")
    print(f"Data YAML:       {data_yaml}")
    print(f"Hard val YAML:   {hard_val_yaml}")
    print(f"Split manifest:  {manifest}")


if __name__ == "__main__":
    main()
