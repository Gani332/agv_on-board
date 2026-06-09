#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run the calibrated mocap-feedback 1x1 square profile.

This wrapper captures the successful OptiTrack run profile:
  side_length=1.0m, linear=0.20, max_angular=0.22,
  leg_timeout=20s, turn_timeout=12s, max_lateral_error=0.15m.

It intentionally still relies on drive_mocap_square.py's --yes safety gate.
Run:
  python3 drive_square.py --yes
"""

from __future__ import print_function

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import drive_mocap_square


DEFAULT_ARGS = [
    "--side-length", "1.0",
    "--linear", "0.20",
    "--max-angular", "0.22",
    "--leg-timeout", "20",
    "--turn-timeout", "12",
    "--max-lateral-error", "0.15",
    "--verbose",
]


def main(argv):
    return drive_mocap_square.main(DEFAULT_ARGS + argv)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
