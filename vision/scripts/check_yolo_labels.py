#!/usr/bin/env python3
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Check YOLO label files for common mistakes.")
    parser.add_argument("--labels", default="vision/dataset/labels")
    parser.add_argument("--images")
    args = parser.parse_args()

    root = Path(args.labels)
    errors = 0
    for label in root.rglob("*.txt"):
        if label.name == "classes.txt":
            continue
        for line_no, line in enumerate(label.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) != 5:
                print(f"{label}:{line_no}: expected 5 fields, got {len(parts)}")
                errors += 1
                continue
            cls, *coords = parts
            if cls != "0":
                print(f"{label}:{line_no}: class must be 0, got {cls}")
                errors += 1
            for value in coords:
                try:
                    number = float(value)
                except ValueError:
                    print(f"{label}:{line_no}: invalid number {value}")
                    errors += 1
                    continue
                if number < 0.0 or number > 1.0:
                    print(f"{label}:{line_no}: coordinate out of range {number}")
                    errors += 1

    if args.images:
        image_root = Path(args.images)
        image_suffixes = {".jpg", ".jpeg", ".png", ".bmp"}
        image_relatives = {
            image.relative_to(image_root).with_suffix(".txt")
            for image in image_root.rglob("*")
            if image.is_file() and image.suffix.lower() in image_suffixes
        }
        label_relatives = {
            label.relative_to(root)
            for label in root.rglob("*.txt")
            if label.name != "classes.txt"
        }
        for missing in sorted(image_relatives - label_relatives):
            print(f"Missing label: {missing}")
            errors += 1
        for orphan in sorted(label_relatives - image_relatives):
            print(f"Orphan label: {orphan}")
            errors += 1

    if errors:
        raise SystemExit(f"Found {errors} label errors")
    print("Labels OK")


if __name__ == "__main__":
    main()
