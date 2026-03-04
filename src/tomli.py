"""Compatibility shim for environments without external tomli package.

Provides a minimal API used by tests and runtime fallbacks by delegating to
Python 3.11+ standard library tomllib.
"""

from tomllib import TOMLDecodeError, load, loads

__all__ = ["loads", "load", "TOMLDecodeError"]
