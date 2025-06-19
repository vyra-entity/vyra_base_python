from typing import Any


try:
    from importlib.metadata import version
except ImportError:
    from importlib_metadata import version  # für Python < 3.8

__version__: str | Any = version("vos-base")