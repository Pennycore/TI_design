#!/usr/bin/env python3
import argparse
import os
from pathlib import Path

os.environ.setdefault("YOLO_CONFIG_DIR", str(Path("vision/.ultralytics").resolve()))
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Train YOLO11n for steel ball detection.")
    parser.add_argument("--data", default="vision/dataset/data.yaml")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--epochs", type=int, default=160)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--project", default="vision/runs")
    parser.add_argument("--name", default="steel_ball_yolo11n")
    parser.add_argument(
        "--finetune",
        action="store_true",
        help="Use a lower learning rate and gentler augmentation for an existing trained model.",
    )
    args = parser.parse_args()

    model = YOLO(args.model)
    train_args = dict(
        data=args.data,
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        project=str(Path(args.project).resolve()),
        name=args.name,
        patience=30,
        cos_lr=True,
        close_mosaic=10,
        degrees=8.0,
        translate=0.08,
        scale=0.45,
        fliplr=0.5,
        hsv_h=0.02,
        hsv_s=0.45,
        hsv_v=0.35,
        workers=0,
    )
    if args.finetune:
        train_args.update(
            optimizer="AdamW",
            lr0=0.0005,
            lrf=0.1,
            warmup_epochs=1.0,
            patience=12,
            mosaic=0.15,
            close_mosaic=5,
            degrees=4.0,
            translate=0.05,
            scale=0.25,
            hsv_h=0.01,
            hsv_s=0.25,
            hsv_v=0.20,
        )
    model.train(**train_args)

    best = Path(model.trainer.best)
    model_name = "steel_ball_yolo11n.pt" if args.name == "steel_ball_yolo11n" else f"{args.name}.pt"
    out = Path("vision/models") / model_name
    out.parent.mkdir(parents=True, exist_ok=True)
    if best.exists():
        out.write_bytes(best.read_bytes())
        print(f"Saved {out}")
    else:
        print(f"Training finished. Best weight expected at {best}")


if __name__ == "__main__":
    main()
