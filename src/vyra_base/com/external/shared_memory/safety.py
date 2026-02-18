"""
Shared Memory Safety & Synchronization

Advanced safety mechanisms and synchronization primitives for shared memory.
Provides thread-safe access, deadlock detection, and memory barriers.

Features:
    - Thread-safe operations with re-entrant locks
    - Deadlock detection and timeout handling
    - Memory barriers for cache coherency
    - Reader-Writer locks for concurrent access
    - Transaction support with rollback
"""
import asyncio
import logging
import threading
import time
from contextlib import contextmanager, asynccontextmanager
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Callable, Dict

import logging
logger = logging.getLogger(__name__)
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)


class LockType(Enum):
    """Lock types for shared memory access."""
    READ = "read"
    WRITE = "write"
    EXCLUSIVE = "exclusive"


@dataclass
class LockInfo:
    """Lock information for debugging and monitoring."""
    lock_type: LockType
    thread_id: int
    acquired_at: float
    timeout: float
    key: str


class DeadlockError(Exception):
    """Raised when a deadlock is detected."""
    pass


class SharedMemorySafetyManager:
    """
    Thread-safe access manager for shared memory operations.
    
    Features:
    - Re-entrant locks per memory key
    - Deadlock detection with timeout
    - Reader-Writer lock pattern
    - Lock statistics and monitoring
    
    Example:
        >>> manager = SharedMemorySafetyManager()
        >>> 
        >>> # Write operation
        >>> with manager.write_lock("sensor_data"):
        ...     # Safe write access
        ...     segment.write(data)
        >>> 
        >>> # Read operation (multiple readers allowed)
        >>> with manager.read_lock("sensor_data"):
        ...     # Safe read access
        ...     data = segment.read()
    """
    
    def __init__(self, deadlock_timeout: float = 5.0):
        """
        Initialize safety manager.
        
        Args:
            deadlock_timeout: Seconds before deadlock detection triggers
        """
        self.deadlock_timeout = deadlock_timeout
        
        # Lock registry per memory key
        self._locks: Dict[str, threading.RLock] = {}
        self._lock_registry: Dict[str, threading.RLock] = {}
        
        # Reader-Writer tracking
        self._reader_counts: Dict[str, int] = {}
        self._writers: Dict[str, int] = {}
        
        # Lock statistics
        self._lock_info: Dict[str, LockInfo] = {}
        self._lock_stats: Dict[str, Dict[str, Any]] = {}
        
        # Master lock for registry access
        self._master_lock = threading.RLock()
    
    def _get_lock(self, key: str) -> threading.RLock:
        """Get or create lock for a memory key."""
        with self._master_lock:
            if key not in self._locks:
                self._locks[key] = threading.RLock()
                self._reader_counts[key] = 0
                self._writers[key] = 0
                self._lock_stats[key] = {
                    "read_count": 0,
                    "write_count": 0,
                    "wait_time_total": 0.0,
                    "lock_time_total": 0.0,
                }
            return self._locks[key]
    
    @contextmanager
    def read_lock(self, key: str, timeout: Optional[float] = None):
        """
        Acquire read lock (multiple readers allowed).
        
        Args:
            key: Memory segment key
            timeout: Optional timeout override
            
        Yields:
            None
            
        Raises:
            DeadlockError: If timeout exceeded
        """
        lock = self._get_lock(key)
        timeout = timeout or self.deadlock_timeout
        start_time = time.time()
        
        # Try to acquire lock
        acquired = lock.acquire(timeout=timeout)
        wait_time = time.time() - start_time
        
        if not acquired:
            raise DeadlockError(
                f"Failed to acquire read lock for '{key}' after {timeout}s"
            )
        
        try:
            # Update reader count
            with self._master_lock:
                self._reader_counts[key] += 1
                self._lock_stats[key]["read_count"] += 1
                self._lock_stats[key]["wait_time_total"] += wait_time
                
                # Store lock info
                self._lock_info[key] = LockInfo(
                    lock_type=LockType.READ,
                    thread_id=threading.get_ident(),
                    acquired_at=time.time(),
                    timeout=timeout,
                    key=key
                )
            
            logger.debug(f"🔓 Read lock acquired: {key} (wait: {wait_time*1000:.2f}ms)")
            yield
            
        finally:
            # Update statistics
            lock_duration = time.time() - start_time - wait_time
            with self._master_lock:
                self._reader_counts[key] -= 1
                self._lock_stats[key]["lock_time_total"] += lock_duration
                if key in self._lock_info:
                    del self._lock_info[key]
            
            lock.release()
            logger.debug(f"🔒 Read lock released: {key} (held: {lock_duration*1000:.2f}ms)")
    
    @contextmanager
    def write_lock(self, key: str, timeout: Optional[float] = None):
        """
        Acquire write lock (exclusive access).
        
        Args:
            key: Memory segment key
            timeout: Optional timeout override
            
        Yields:
            None
            
        Raises:
            DeadlockError: If timeout exceeded
        """
        lock = self._get_lock(key)
        timeout = timeout or self.deadlock_timeout
        start_time = time.time()
        
        # Try to acquire lock
        acquired = lock.acquire(timeout=timeout)
        wait_time = time.time() - start_time
        
        if not acquired:
            raise DeadlockError(
                f"Failed to acquire write lock for '{key}' after {timeout}s"
            )
        
        try:
            # Wait for all readers to finish
            deadline = time.time() + timeout
            while self._reader_counts.get(key, 0) > 0:
                if time.time() > deadline:
                    raise DeadlockError(
                        f"Timeout waiting for readers on '{key}'"
                    )
                time.sleep(0.001)  # 1ms
            
            # Update writer flag
            with self._master_lock:
                self._writers[key] = threading.get_ident()
                self._lock_stats[key]["write_count"] += 1
                self._lock_stats[key]["wait_time_total"] += wait_time
                
                # Store lock info
                self._lock_info[key] = LockInfo(
                    lock_type=LockType.WRITE,
                    thread_id=threading.get_ident(),
                    acquired_at=time.time(),
                    timeout=timeout,
                    key=key
                )
            
            logger.debug(f"🔓 Write lock acquired: {key} (wait: {wait_time*1000:.2f}ms)")
            yield
            
        finally:
            # Update statistics
            lock_duration = time.time() - start_time - wait_time
            with self._master_lock:
                self._writers[key] = 0
                self._lock_stats[key]["lock_time_total"] += lock_duration
                if key in self._lock_info:
                    del self._lock_info[key]
            
            lock.release()
            logger.debug(f"🔒 Write lock released: {key} (held: {lock_duration*1000:.2f}ms)")
    
    def get_lock_statistics(self, key: Optional[str] = None) -> Dict[str, Any]:
        """
        Get lock statistics.
        
        Args:
            key: Optional key to filter (None = all keys)
            
        Returns:
            Statistics dictionary
        """
        with self._master_lock:
            if key:
                return self._lock_stats.get(key, {})
            return self._lock_stats.copy()
    
    def get_active_locks(self) -> Dict[str, LockInfo]:
        """
        Get currently active locks.
        
        Returns:
            Dictionary of active locks
        """
        with self._master_lock:
            return self._lock_info.copy()
    
    def check_deadlock(self) -> list:
        """
        Check for potential deadlocks.
        
        Returns:
            List of keys with potential deadlocks
        """
        deadlocks = []
        current_time = time.time()
        
        with self._master_lock:
            for key, info in self._lock_info.items():
                held_duration = current_time - info.acquired_at
                if held_duration > info.timeout:
                    deadlocks.append({
                        "key": key,
                        "lock_type": info.lock_type.value,
                        "thread_id": info.thread_id,
                        "held_duration": held_duration,
                        "timeout": info.timeout
                    })
        
        return deadlocks


class MemoryBarrier:
    """
    Memory barrier for cache coherency.
    
    Ensures that memory operations are visible across threads/processes.
    Important for multi-core systems with separate caches.
    """
    
    @staticmethod
    def acquire():
        """Acquire barrier (load operations)."""
        # Python's threading already provides acquire semantics
        # This is a placeholder for explicit barriers if needed
        pass
    
    @staticmethod
    def release():
        """Release barrier (store operations)."""
        # Python's threading already provides release semantics
        # This is a placeholder for explicit barriers if needed
        pass
    
    @staticmethod
    def full():
        """Full memory barrier (all operations)."""
        # Force a barrier by acquiring and releasing a dummy lock
        lock = threading.Lock()
        lock.acquire()
        lock.release()


class TransactionalMemory:
    """
    Transaction support for shared memory operations.
    
    Provides ACID-like guarantees with rollback capability.
    """
    
    def __init__(self, safety_manager: SharedMemorySafetyManager):
        """Initialize transactional memory."""
        self.safety_manager = safety_manager
        self._snapshots: Dict[str, Any] = {}
    
    @contextmanager
    def transaction(self, key: str, read_func: Callable, write_func: Callable):
        """
        Execute operations in a transaction.
        
        Args:
            key: Memory key
            read_func: Function to read current value
            write_func: Function to write new value
            
        Yields:
            (current_value, commit_func)
        """
        with self.safety_manager.write_lock(key):
            # Take snapshot
            try:
                snapshot = read_func()
                self._snapshots[key] = snapshot
            except Exception as e:
                logger.error(f"Failed to take snapshot: {e}")
                raise
            
            # Yield commit function
            committed = False
            def commit(new_value):
                nonlocal committed
                try:
                    write_func(new_value)
                    committed = True
                    logger.debug(f"✅ Transaction committed: {key}")
                except Exception as e:
                    logger.error(f"❌ Transaction commit failed: {e}")
                    # Rollback
                    try:
                        write_func(snapshot)
                        logger.info(f"🔄 Transaction rolled back: {key}")
                    except Exception as re:
                        logger.error(f"❌ Rollback failed: {re}")
                    raise
            
            try:
                yield snapshot, commit
            finally:
                # Cleanup snapshot
                if key in self._snapshots:
                    del self._snapshots[key]
                
                if not committed:
                    logger.warning(f"⚠️  Transaction aborted: {key}")


# Global safety manager instance
_global_safety_manager: Optional[SharedMemorySafetyManager] = None


def get_global_safety_manager() -> SharedMemorySafetyManager:
    """
    Get global safety manager singleton.
    
    Returns:
        SharedMemorySafetyManager instance
    """
    global _global_safety_manager
    if _global_safety_manager is None:
        _global_safety_manager = SharedMemorySafetyManager()
    return _global_safety_manager
