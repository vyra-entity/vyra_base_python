from typing import Any
"""
Module to retrieve the version of the 'vyra_base' package.

This module attempts to import the appropriate `version` function from
`importlib.metadata` (Python 3.8+) or `importlib_metadata` (for older versions).
It then sets the `__version__` variable to the installed version of the
'vyra_base' package.

Attributes
----------
__version__ : str or Any
    The version string of the 'vyra_base' package as returned by the
    `version` function.

Raises
------
ImportError
    If neither `importlib.metadata` nor `importlib_metadata` can be imported.
PackageNotFoundError
    If the 'vyra_base' package is not installed.
"""


try:
    from importlib.metadata import version
except ImportError:
    from importlib_metadata import version  # für Python < 3.8

__version__: str | Any = version("vyra_base")