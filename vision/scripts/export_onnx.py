#!/usr/bin/env python3
import argparse
import os
from pathlib import Path

os.environ.setdefault("YOLO_CONFIG_DIR", str(Path("vision/.ultralytics").resolve()))

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Export trained YOLO model to ONNX.")
    parser.add_argument("--weights", default="vision/models/steel_ball_yolo11n.pt")
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument(
        "--output",
        help="Output ONNX path. Defaults to vision/exports/<weights-stem>.onnx.",
    )
    args = parser.parse_args()

    weights = Path(args.weights)
    if not weights.exists():
        raise FileNotFoundError(weights)

    model = YOLO(str(weights))
    exported = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=True,
        dynamic=False,
    )

    export_dir = Path("vision/exports")
    export_dir.mkdir(parents=True, exist_ok=True)
    target = Path(args.output) if args.output else export_dir / f"{weights.stem}.onnx"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(Path(exported).read_bytes())
    print(f"Saved {target}")


if __name__ == "__main__":
    main()
