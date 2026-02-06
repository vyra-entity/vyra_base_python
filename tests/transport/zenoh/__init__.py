"""
Test suite initialization for Zenoh transport tests.
"""
import pytest


# Test markers
pytest_plugins = []


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers",
        "zenoh: mark test as requiring Zenoh to be available"
    )
