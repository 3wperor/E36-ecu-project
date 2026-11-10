#!/usr/bin/env python3
"""Start the tuner:  python3 tuner/run_tuner.py [--demo] [--port PORT] [file.tune]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from e36tuner.app import main  # noqa: E402

sys.exit(main())
