import asyncio
import threading
from pathlib import Path
from typing import Union

from vyra_base.helper._aiopath import AsyncPath

# Dictionary für dateispezifische Locks (async)
file_locks = {}

# Dictionary für dateispezifische Locks (sync)
file_locks_sync = {}

async def get_lock_for_file(file_path: Union[Path, AsyncPath]) -> asyncio.Lock:
    """
    Get (or create) a lock for the specified file.

    :param file_path: The path to the file.
    :type file_path: pathlib.Path or AsyncPath
    :return: An asyncio.Lock object for the file.
    :rtype: asyncio.Lock
    """
    if file_path not in file_locks:
        file_locks[file_path] = asyncio.Lock()
    return file_locks[file_path]

async def release_lock_for_file(file_path: Union[Path, AsyncPath]):
    """
    Remove the lock from the dictionary if no tasks are waiting for it.

    :param file_path: The path to the file.
    :type file_path: pathlib.Path or AsyncPath
    """
    if file_path in file_locks:
        # Prüfe, ob der Lock gerade verwendet wird
        lock = file_locks[file_path]
        if not lock.locked():
            del file_locks[file_path]


# ============================================================================
# SYNC FILE LOCKS
# ============================================================================

def get_lock_for_file_sync(file_path: Union[Path, str]) -> threading.Lock:
    """
    Get (or create) a threading lock for the specified file (synchronous).

    :param file_path: The path to the file.
    :type file_path: pathlib.Path or str
    :return: A threading.Lock object for the file.
    :rtype: threading.Lock
    """
    file_path = Path(file_path) if isinstance(file_path, str) else file_path
    if file_path not in file_locks_sync:
        file_locks_sync[file_path] = threading.Lock()
    return file_locks_sync[file_path]


def release_lock_for_file_sync(file_path: Union[Path, str]):
    """
    Remove the threading lock from the dictionary if not locked (synchronous).

    :param file_path: The path to the file.
    :type file_path: pathlib.Path or str
    """
    file_path = Path(file_path) if isinstance(file_path, str) else file_path
    if file_path in file_locks_sync:
        lock = file_locks_sync[file_path]
        if not lock.locked():
            del file_locks_sync[file_path]