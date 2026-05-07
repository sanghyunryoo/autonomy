#!/usr/bin/env python3
"""Compatibility entrypoint with USB-port-oriented naming."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).with_name("show_realsense_serials.py")), run_name="__main__")
