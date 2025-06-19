import asyncio

from aiopath import AsyncPath
from pathlib import Path

from typing import Union

# Dictionary für dateispezifische Locks
file_locks = {}

async def get_lock_for_file(file_path: Union[Path, AsyncPath]) -> asyncio.Lock:
    """Holt (oder erstellt) einen Lock für die spezifische Datei."""
    if file_path not in file_locks:
        file_locks[file_path] = asyncio.Lock()
    return file_locks[file_path]

async def release_lock_for_file(file_path: str):
    """Entfernt den Lock aus dem dict, wenn keine Tasks mehr darauf warten."""
    if file_path in file_locks:
        # Prüfe, ob der Lock gerade verwendet wird
        lock = file_locks[file_path]
        if not lock.locked():
            del file_locks[file_path]