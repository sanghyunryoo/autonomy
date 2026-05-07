#!/usr/bin/env python3
"""Compatibility wrapper for the renamed USB-port RealSense mapper."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).with_name("realsense_usb_mapper.py")), run_name="__main__")
