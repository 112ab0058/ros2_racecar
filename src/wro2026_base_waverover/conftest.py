"""Pytest setup: make the package importable without installing it, plus fixtures."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self, start=100.0):
        self.t = start

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.fixture
def clock():
    return FakeClock()
