"""Entry point. Works as `python -m mpu6050_viewer` and as `python path/to/mpu6050_viewer`."""
import os
import sys

if __package__ in (None, ""):
    # Started by directory path: there is no parent package, so put tools/ on sys.path
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from mpu6050_viewer.app import main
else:
    from .app import main

sys.exit(main())
