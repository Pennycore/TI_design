#!/usr/bin/env python3
import argparse
from pathlib import Path

import nncase
import numpy as np
from PIL import Image


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def read_model(path):
    return path.read_bytes()


def select_calibration_images(directory, count):
    images = sorted(
        path
        for path in directory.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if len(images) < count:
        raise ValueError(
            "Need at least %d calibration images, found %d in %s"
            % (count, len(images), directory)
        )

    indices = np.linspace(0, len(images) - 1, count, dtype=np.int32)
    return [images[int(index)] for index in indices]


def letterbox_top_left(image, size):
    width, height = image.size
    scale = min(size / width, size / height)
    new_width = max(1, int(width * scale))
    new_height = max(1, int(height * scale))
    resampling = getattr(Image, "Resampling", Image).BILINEAR
    resized = image.resize((new_width, new_height), resampling)

    canvas = Image.new("RGB", (size, size), (128, 128, 128))
    canvas.paste(resized, (0, 0))
    array = np.asarray(canvas, dtype=np.uint8)
    array = np.transpose(array, (2, 0, 1))
    return array[np.newaxis, ...]


def generate_calibration_data(image_paths, input_size):
    data = []
    for index, path in enumerate(image_paths, start=1):
        with Image.open(path) as image:
            tensor = letterbox_top_left(image.convert("RGB"), input_size)
        data.append([tensor])
        if index == 1 or index % 10 == 0 or index == len(image_paths):
            print("Prepared calibration image %d/%d" % (index, len(image_paths)))
    return data


def convert(args):
    model_path = args.model.resolve()
    dataset_path = args.dataset.resolve()
    output_path = args.output.resolve()
    dump_dir = args.dump_dir.resolve()

    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    if not dataset_path.is_dir():
        raise NotADirectoryError(dataset_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    dump_dir.mkdir(parents=True, exist_ok=True)
    input_shape = [1, 3, args.input_size, args.input_size]

    calibration_images = select_calibration_images(dataset_path, args.samples)
    calibration_data = generate_calibration_data(
        calibration_images,
        args.input_size,
    )

    compile_options = nncase.CompileOptions()
    compile_options.target = "k230"
    compile_options.preprocess = True
    compile_options.swapRB = False
    compile_options.input_shape = input_shape
    compile_options.input_type = "uint8"
    compile_options.input_range = [0, 1]
    compile_options.mean = [0, 0, 0]
    compile_options.std = [1, 1, 1]
    compile_options.input_layout = "NCHW"
    compile_options.output_layout = "NCHW"
    compile_options.dump_ir = args.dump_ir
    compile_options.dump_asm = args.dump_asm
    compile_options.dump_dir = str(dump_dir)

    print("Importing ONNX:", model_path)
    compiler = nncase.Compiler(compile_options)
    compiler.import_onnx(read_model(model_path), nncase.ImportOptions())

    ptq_options = nncase.PTQTensorOptions()
    ptq_options.samples_count = args.samples
    ptq_options.calibrate_method = args.calibrate_method
    ptq_options.quant_type = "uint8"
    ptq_options.w_quant_type = "uint8"
    ptq_options.export_weight_range_by_channel = True
    ptq_options.set_tensor_data(calibration_data)
    compiler.use_ptq(ptq_options)

    print(
        "Compiling KModel with %d samples (%s)..."
        % (args.samples, args.calibrate_method)
    )
    compiler.compile()
    output_path.write_bytes(compiler.gencode_tobytes())
    print("Saved:", output_path)
    print("Size: %.2f MiB" % (output_path.stat().st_size / (1024 * 1024)))


def main():
    parser = argparse.ArgumentParser(
        description="Convert the steel-ball YOLO11 ONNX model to a K230 KModel."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("vision/exports/steel_ball_yolo11n.onnx"),
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("vision/dataset/images/train"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("vision/exports/steel_ball_yolo11n.kmodel"),
    )
    parser.add_argument(
        "--dump-dir",
        type=Path,
        default=Path("vision/nncase_dump/steel_ball_yolo11n"),
    )
    parser.add_argument("--input-size", type=int, default=416)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument(
        "--calibrate-method",
        choices=("Kld", "NoClip"),
        default="Kld",
    )
    parser.add_argument("--dump-ir", action="store_true")
    parser.add_argument("--dump-asm", action="store_true")
    convert(parser.parse_args())


if __name__ == "__main__":
    main()
