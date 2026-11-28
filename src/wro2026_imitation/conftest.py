"""Pytest setup: make the package importable without installing it."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# corner tests drive scan_memory from the lidar package; in a workspace it is installed,
# in a plain checkout it sits next to this package
try:
    import wro2026_lidar_n10p  # noqa: F401
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'wro2026_lidar_n10p'))
