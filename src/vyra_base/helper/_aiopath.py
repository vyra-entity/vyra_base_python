# helper/async_path.py
import sys

if sys.version_info >= (3, 12):
    from anyio import Path as BasePath
else:
    from aiopath import AsyncPath as BasePath

class AsyncPath(BasePath):
    """
    Unified async path interface compatible with Python 3.12+ and older versions.
    
    Provides a consistent async pathlib interface by wrapping:
    - Python 3.12+: anyio.Path
    - Python <3.12: aiopath.AsyncPath
    
    This allows VYRA code to use async path operations regardless of Python version.
    """
    pass  # hier könntest du ggf. Methoden anpassen oder ergänzen