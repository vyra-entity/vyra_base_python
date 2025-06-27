# helper/async_path.py
import sys

if sys.version_info >= (3, 12):
    from anyio import Path as BasePath
else:
    from aiopath import AsyncPath as BasePath

class AsyncPath(BasePath):
    pass  # hier könntest du ggf. Methoden anpassen oder ergänzen