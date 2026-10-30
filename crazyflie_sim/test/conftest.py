"""Pytest configuration for crazyflie_sim tests."""

import sys
from unittest.mock import MagicMock

import pytest

try:
    import cffirmware  # noqa: F401
    HAS_CFFIRMWARE = True
except ImportError:
    HAS_CFFIRMWARE = False
    sys.modules['cffirmware'] = MagicMock()


def pytest_configure(config):
    """Register the requires_firmware marker."""
    config.addinivalue_line(
        'markers',
        'requires_firmware: needs Crazyflie firmware Python bindings')


def pytest_collection_modifyitems(config, items):
    """Skip firmware-backed tests when cffirmware is unavailable."""
    if HAS_CFFIRMWARE:
        return
    skip_firmware = pytest.mark.skip(
        reason='cffirmware is not installed')
    for item in items:
        if 'requires_firmware' in item.keywords:
            item.add_marker(skip_firmware)
