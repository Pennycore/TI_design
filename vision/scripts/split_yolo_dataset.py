#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def copy_pair(image_path, label_path, image_dst, label_dst):
    image_dst.parent.mkdir(parents=True, exist_ok=True)
    label_dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(image_path, image_dst)
    if label_path.exists():
        shutil.copy2(label_path, label_dst)
    else:
        label_dst.write_text("", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Split raw YOLO images/labels into train/val.")
    parser.add_argument("--raw", required=True, help="Raw folder with images/ and labels/ subfolders.")
    parser.add_argument("--out", default="vision/dataset")
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    raw = Path(args.raw)
    raw_images = raw / "images"
    raw_labels = raw / "labels"
    out = Path(args.out)

    images = [p for p in raw_images.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES]
    if not images:
        raise RuntimeError(f"No images found under {raw_images}")

    random.seed(args.seed)
    random.shuffle(images)
    val_count = max(1, int(len(images) * args.val_ratio))
    val_set = set(images[:val_count])

    for image_path in images:
        split = "val" if image_path in val_set else "train"
        relative = image_path.relative_to(raw_images)
        label_relative = relative.with_suffix(".txt")
        label_path = raw_labels / label_relative
        copy_pair(
            image_path,
            label_path,
            out / "images" / split / relative.name,
            out / "labels" / split / label_relative.name,
        )

    print(f"Images: {len(images)}, train: {len(images) - val_count}, val: {val_count}")


if __name__ == "__main__":
    main()
