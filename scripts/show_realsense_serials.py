#!/usr/bin/env python3
"""Show connected RealSense RGB streams with serial and USB port information."""

import argparse
import re
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display RealSense RGB images with USB port ids for manual camera mapping."
    )
    parser.add_argument("--width", type=int, default=640, help="Color stream width.")
    parser.add_argument("--height", type=int, default=480, help="Color stream height.")
    parser.add_argument("--fps", type=int, default=30, help="Color stream FPS.")
    return parser.parse_args()


def import_runtime_modules():
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ModuleNotFoundError as exc:
        missing = exc.name
        print(
            f"Missing Python module: {missing}\n"
            "Install OpenCV and pyrealsense2 before running this tool.\n"
            "Example:\n"
            "  sudo apt install python3-opencv\n"
            "  python3 -m pip install pyrealsense2",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    return cv2, np, rs


def safe_device_info(device, info):
    if info is None:
        return ""
    try:
        return device.get_info(info)
    except RuntimeError:
        return ""


def usb_port_id(physical_port):
    text = str(physical_port or "")
    matches = re.findall(r"(?<![\w.])\d+-\d+(?:\.\d+)*(?=[:/]|$)", text)
    if matches:
        return matches[-1]
    parts = [part for part in text.split("/") if part]
    return parts[-1].split(":")[0] if parts else ""


def connected_devices(rs):
    context = rs.context()
    devices = []
    for device in context.query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        name = safe_device_info(device, rs.camera_info.name)
        physical_port = safe_device_info(device, getattr(rs.camera_info, "physical_port", None))
        devices.append(
            {
                "serial": serial,
                "name": name,
                "usb_port_id": usb_port_id(physical_port),
                "physical_port": physical_port,
            }
        )
    return devices


def start_pipeline(rs, serial, width, height, fps):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    return pipeline


def draw_label(cv2, image, device):
    output = image.copy()
    label_1 = f"usb_port_id: {device['usb_port_id']}  model: {device['name']}"
    label_2 = f"serial: {device['serial']}"
    cv2.rectangle(output, (8, 8), (8 + 620, 82), (0, 0, 0), thickness=-1)
    cv2.putText(
        output,
        label_1,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        label_2,
        (20, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def main():
    args = parse_args()
    cv2, np, rs = import_runtime_modules()
    devices = connected_devices(rs)
    if not devices:
        print("No RealSense devices found.", file=sys.stderr)
        return 1

    print("Connected RealSense devices:")
    for device in devices:
        print(
            f"  usb_port_id={device['usb_port_id']} "
            f"serial={device['serial']} model={device['name']}"
        )
        if device["physical_port"]:
            print(f"    physical_port={device['physical_port']}")

    pipelines = []
    try:
        for device in devices:
            pipelines.append(
                (
                    device,
                    start_pipeline(rs, device["serial"], args.width, args.height, args.fps),
                )
            )

        print("Press 'q' or ESC in an image window to quit.")
        while True:
            should_quit = False
            for device, pipeline in pipelines:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                image = np.asanyarray(color_frame.get_data())
                image = draw_label(cv2, image, device)
                window_name = f"RealSense {device['usb_port_id'] or device['serial']}"
                cv2.imshow(window_name, image)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                should_quit = True
            if should_quit:
                break
            time.sleep(0.001)
    finally:
        for _, pipeline in pipelines:
            pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
