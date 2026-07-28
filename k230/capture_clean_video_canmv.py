import gc
import sys

from libs.PipeLine import PipeLine


RGB888P_SIZE = [640, 480]
DISPLAY_MODE = "virt"
DISPLAY_SIZE = [800, 480]


def main():
    pipeline = None
    try:
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create()
        pipeline.osd_img.clear()
        print("Clean capture ready: use the IDE Record button")

        while True:
            frame = pipeline.get_frame()
            pipeline.osd_img.clear()
            pipeline.show_image()
            del frame
            gc.collect()
    except KeyboardInterrupt:
        print("Stopped by user")
    except Exception as error:
        sys.print_exception(error)
    finally:
        if pipeline is not None:
            pipeline.destroy()
        gc.collect()


if __name__ == "__main__":
    main()
