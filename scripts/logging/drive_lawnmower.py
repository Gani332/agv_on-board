#!/usr/bin/env python3
"""Compatibility entrypoint for the straight out-and-back controller."""

import os
import sys


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drive_straight import main  # noqa: E402


if __name__ == "__main__":
    main(sys.argv[1:])
