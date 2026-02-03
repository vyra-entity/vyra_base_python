"""
Unit tests for Shared Memory Safety Manager

Tests thread-safe access, deadlock detection, and transaction support.
"""
import asyncio
import pytest
import threading
import time
from unittest.mock import Mock, patch

from vyra_base.com.external.shared_memory.safety import (
    SharedMemorySafetyManager,
    LockType,
    DeadlockError,
    MemoryBarrier,
    TransactionalMemory,
    get_global_safety_manager
)


class TestSharedMemorySafetyManager:
    """Test safety manager operations."""
    
    def test_read_lock_acquisition(self):
        """Test read lock can be acquired."""
        manager = SharedMemorySafetyManager(deadlock_timeout=1.0)
        
        with manager.read_lock("test_key"):
            # Lock should be held
            stats = manager.get_lock_statistics("test_key")
            assert stats["read_count"] == 1
        
        # Lock should be released
        active = manager.get_active_locks()
        assert "test_key" not in active
    
    def test_write_lock_acquisition(self):
        """Test write lock can be acquired."""
        manager = SharedMemorySafetyManager(deadlock_timeout=1.0)
        
        with manager.write_lock("test_key"):
            # Lock should be held
            stats = manager.get_lock_statistics("test_key")
            assert stats["write_count"] == 1
        
        # Lock should be released
        active = manager.get_active_locks()
        assert "test_key" not in active
    
    def test_multiple_readers(self):
        """Test multiple readers can access simultaneously."""
        manager = SharedMemorySafetyManager(deadlock_timeout=2.0)
        results = []
        
        def reader(idx):
            with manager.read_lock("test_key", timeout=1.0):
                results.append(f"reader_{idx}_start")
                time.sleep(0.1)
                results.append(f"reader_{idx}_end")
        
        # Start 3 readers
        threads = [threading.Thread(target=reader, args=(i,)) for i in range(3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # All readers should have executed
        assert len(results) == 6
        assert all(f"reader_{i}_start" in results for i in range(3))
        assert all(f"reader_{i}_end" in results for i in range(3))
    
    def test_write_lock_exclusivity(self):
        """Test write lock is exclusive."""
        manager = SharedMemorySafetyManager(deadlock_timeout=2.0)
        results = []
        
        def writer(idx):
            with manager.write_lock("test_key", timeout=1.5):
                results.append(f"writer_{idx}_start")
                time.sleep(0.2)
                results.append(f"writer_{idx}_end")
        
        # Start 2 writers
        threads = [threading.Thread(target=writer, args=(i,)) for i in range(2)]
        start_time = time.time()
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        duration = time.time() - start_time
        
        # Writers should execute sequentially (at least 0.4s total)
        assert duration >= 0.4
        
        # First writer completes before second starts
        writer_0_end_idx = results.index("writer_0_end")
        writer_1_start_idx = results.index("writer_1_start")
        assert writer_0_end_idx < writer_1_start_idx or \
               results.index("writer_1_end") < results.index("writer_0_start")
    
    def test_deadlock_detection_timeout(self):
        """Test deadlock detection triggers on timeout."""
        manager = SharedMemorySafetyManager(deadlock_timeout=0.5)
        
        def blocker():
            with manager.write_lock("test_key", timeout=2.0):
                time.sleep(1.0)  # Hold lock longer than deadlock_timeout
        
        # Start blocker thread
        t = threading.Thread(target=blocker)
        t.start()
        time.sleep(0.1)  # Let blocker acquire lock
        
        # Try to acquire should timeout
        with pytest.raises(DeadlockError) as exc_info:
            with manager.write_lock("test_key", timeout=0.3):
                pass
        
        assert "Failed to acquire write lock" in str(exc_info.value)
        
        t.join()
    
    def test_lock_statistics(self):
        """Test lock statistics tracking."""
        manager = SharedMemorySafetyManager()
        
        # Perform operations
        with manager.read_lock("key1"):
            pass
        
        with manager.write_lock("key1"):
            pass
        
        with manager.read_lock("key2"):
            pass
        
        # Check statistics
        stats1 = manager.get_lock_statistics("key1")
        assert stats1["read_count"] == 1
        assert stats1["write_count"] == 1
        assert stats1["wait_time_total"] >= 0
        assert stats1["lock_time_total"] >= 0
        
        stats2 = manager.get_lock_statistics("key2")
        assert stats2["read_count"] == 1
        assert stats2["write_count"] == 0
        
        # Get all statistics
        all_stats = manager.get_lock_statistics()
        assert "key1" in all_stats
        assert "key2" in all_stats
    
    def test_active_locks_tracking(self):
        """Test active locks are tracked."""
        manager = SharedMemorySafetyManager()
        
        # No active locks initially
        assert len(manager.get_active_locks()) == 0
        
        # Acquire lock in thread
        def holder():
            with manager.write_lock("test_key"):
                time.sleep(0.2)
        
        t = threading.Thread(target=holder)
        t.start()
        time.sleep(0.05)  # Let thread acquire lock
        
        # Check active locks
        active = manager.get_active_locks()
        assert "test_key" in active
        assert active["test_key"].lock_type == LockType.WRITE
        
        t.join()
        
        # Lock should be released
        assert len(manager.get_active_locks()) == 0
    
    def test_check_deadlock(self):
        """Test deadlock checking."""
        manager = SharedMemorySafetyManager(deadlock_timeout=0.5)
        
        # Acquire lock and hold it
        def holder():
            with manager.write_lock("test_key", timeout=2.0):
                time.sleep(1.0)
        
        t = threading.Thread(target=holder)
        t.start()
        time.sleep(0.1)  # Let thread acquire lock
        
        # Check for deadlocks after timeout
        time.sleep(0.5)
        deadlocks = manager.check_deadlock()
        
        # Should detect potential deadlock
        assert len(deadlocks) > 0
        assert deadlocks[0]["key"] == "test_key"
        assert deadlocks[0]["held_duration"] > 0.5
        
        t.join()


class TestMemoryBarrier:
    """Test memory barrier operations."""
    
    def test_acquire_barrier(self):
        """Test acquire barrier."""
        # Should not raise
        MemoryBarrier.acquire()
    
    def test_release_barrier(self):
        """Test release barrier."""
        # Should not raise
        MemoryBarrier.release()
    
    def test_full_barrier(self):
        """Test full memory barrier."""
        # Should not raise
        MemoryBarrier.full()


class TestTransactionalMemory:
    """Test transactional memory operations."""
    
    def test_transaction_commit(self):
        """Test transaction commit."""
        manager = SharedMemorySafetyManager()
        transactional = TransactionalMemory(manager)
        
        # Mock read/write functions
        state = {"value": 10}
        
        def read_func():
            return state["value"]
        
        def write_func(value):
            state["value"] = value
        
        # Execute transaction
        with transactional.transaction("test_key", read_func, write_func) as (current, commit):
            assert current == 10
            new_value = current + 5
            commit(new_value)
        
        # Value should be updated
        assert state["value"] == 15
    
    def test_transaction_rollback_on_exception(self):
        """Test transaction rolls back on exception."""
        manager = SharedMemorySafetyManager()
        transactional = TransactionalMemory(manager)
        
        # Mock read/write functions
        state = {"value": 10}
        
        def read_func():
            return state["value"]
        
        def write_func(value):
            if value > 20:
                raise ValueError("Value too large")
            state["value"] = value
        
        # Execute transaction that fails
        with pytest.raises(ValueError):
            with transactional.transaction("test_key", read_func, write_func) as (current, commit):
                assert current == 10
                commit(25)  # This should fail and rollback
        
        # Value should be rolled back
        assert state["value"] == 10
    
    def test_transaction_without_commit(self):
        """Test transaction without commit aborts."""
        manager = SharedMemorySafetyManager()
        transactional = TransactionalMemory(manager)
        
        # Mock read/write functions
        state = {"value": 10}
        
        def read_func():
            return state["value"]
        
        def write_func(value):
            state["value"] = value
        
        # Execute transaction without commit
        with transactional.transaction("test_key", read_func, write_func) as (current, commit):
            assert current == 10
            # Don't call commit()
        
        # Value should remain unchanged
        assert state["value"] == 10


class TestGlobalSafetyManager:
    """Test global safety manager singleton."""
    
    def test_singleton_instance(self):
        """Test global manager returns same instance."""
        manager1 = get_global_safety_manager()
        manager2 = get_global_safety_manager()
        
        assert manager1 is manager2
    
    def test_global_manager_usage(self):
        """Test global manager can be used."""
        manager = get_global_safety_manager()
        
        # Should work like normal manager
        with manager.read_lock("global_test"):
            stats = manager.get_lock_statistics("global_test")
            assert stats["read_count"] >= 1  # May have been used before


class TestConcurrentAccess:
    """Test concurrent access patterns."""
    
    def test_concurrent_read_write(self):
        """Test concurrent readers and writers."""
        manager = SharedMemorySafetyManager(deadlock_timeout=5.0)
        results = []
        
        def reader(idx):
            for i in range(5):
                with manager.read_lock("shared_data", timeout=2.0):
                    results.append(f"read_{idx}_{i}")
                    time.sleep(0.01)
        
        def writer(idx):
            for i in range(3):
                with manager.write_lock("shared_data", timeout=2.0):
                    results.append(f"write_{idx}_{i}")
                    time.sleep(0.02)
        
        # Start multiple readers and writers
        threads = []
        threads.extend([threading.Thread(target=reader, args=(i,)) for i in range(3)])
        threads.extend([threading.Thread(target=writer, args=(i,)) for i in range(2)])
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # All operations should complete
        assert len(results) == 15 + 6  # 3*5 reads + 2*3 writes
    
    def test_reentrant_locks(self):
        """Test locks are reentrant."""
        manager = SharedMemorySafetyManager()
        
        # Nested read locks (same thread)
        with manager.read_lock("test_key"):
            with manager.read_lock("test_key"):
                stats = manager.get_lock_statistics("test_key")
                # Should allow nested acquisition
                assert stats["read_count"] >= 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
