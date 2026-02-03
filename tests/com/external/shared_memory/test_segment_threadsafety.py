"""
Unit tests for Shared Memory Segment Thread Safety

Tests thread-safe operations and transaction support in segment.py.
"""
import pytest
import threading
import time
from unittest.mock import Mock, patch

from vyra_base.com.external.shared_memory.segment import SharedMemorySegment
from vyra_base.com.core.exceptions import TransportError


@pytest.mark.unit
class TestSegmentThreadSafety:
    """Test thread-safe segment operations."""
    
    @pytest.fixture
    def mock_segment(self):
        """Mock SharedMemorySegment for testing."""
        with patch('vyra_base.com.external.shared_memory.segment.posix_ipc'):
            with patch('vyra_base.com.external.shared_memory.segment.POSIX_IPC_AVAILABLE', True):
                segment = SharedMemorySegment(
                    name="/test_segment",
                    size=1024,
                    create=True
                )
                # Mock internals
                segment._segment = Mock()
                segment._segment.fd = 1
                segment._mutex = Mock()
                segment._mutex.acquire = Mock(return_value=True)
                segment._mutex.release = Mock()
                
                yield segment
                
                # Cleanup
                segment.close()
    
    def test_concurrent_writes(self, mock_segment):
        """Test multiple threads can write safely."""
        results = []
        errors = []
        
        def writer(idx):
            try:
                for i in range(10):
                    success = mock_segment.write({"thread": idx, "count": i})
                    if success:
                        results.append((idx, i))
            except Exception as e:
                errors.append(e)
        
        # Start 5 writer threads
        threads = [threading.Thread(target=writer, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # No errors should occur
        assert len(errors) == 0
        
        # All writes should succeed (50 total: 5 threads * 10 writes)
        assert len(results) == 50
    
    def test_concurrent_reads(self, mock_segment):
        """Test multiple threads can read safely."""
        results = []
        errors = []
        
        # Mock read to return test data
        def mock_read_impl(timeout):
            from vyra_base.com.external.shared_memory.serialization import MessageType
            return (MessageType.REQUEST, {"data": "test"}, int(time.time()))
        
        mock_segment._unsafe_read = Mock(side_effect=mock_read_impl)
        
        def reader(idx):
            try:
                for i in range(10):
                    msg_type, data, timestamp = mock_segment.read()
                    results.append((idx, data))
            except Exception as e:
                errors.append(e)
        
        # Start 5 reader threads
        threads = [threading.Thread(target=reader, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # No errors should occur
        assert len(errors) == 0
        
        # All reads should succeed (50 total: 5 threads * 10 reads)
        assert len(results) == 50
    
    def test_mixed_read_write(self, mock_segment):
        """Test concurrent readers and writers."""
        results = []
        errors = []
        
        # Mock read to return test data
        def mock_read_impl(timeout):
            from vyra_base.com.external.shared_memory.serialization import MessageType
            return (MessageType.REQUEST, {"data": "test"}, int(time.time()))
        
        mock_segment._unsafe_read = Mock(side_effect=mock_read_impl)
        
        def reader(idx):
            try:
                for i in range(5):
                    msg_type, data, timestamp = mock_segment.read()
                    results.append(f"read_{idx}_{i}")
            except Exception as e:
                errors.append(e)
        
        def writer(idx):
            try:
                for i in range(5):
                    mock_segment.write({"thread": idx, "count": i})
                    results.append(f"write_{idx}_{i}")
            except Exception as e:
                errors.append(e)
        
        # Start mixed threads
        threads = []
        threads.extend([threading.Thread(target=reader, args=(i,)) for i in range(3)])
        threads.extend([threading.Thread(target=writer, args=(i,)) for i in range(3)])
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # No errors should occur
        assert len(errors) == 0
        
        # All operations should complete (30 total: 6 threads * 5 ops)
        assert len(results) == 30
    
    def test_thread_lock_prevents_race_conditions(self, mock_segment):
        """Test thread lock prevents race conditions."""
        counter = {"value": 0}
        
        def increment():
            # Read current value
            current = counter["value"]
            time.sleep(0.001)  # Simulate delay
            # Write new value
            counter["value"] = current + 1
        
        # Wrap in thread lock
        def safe_increment():
            with mock_segment._thread_lock:
                increment()
        
        # Run 100 increments concurrently
        threads = [threading.Thread(target=safe_increment) for _ in range(100)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # Should be exactly 100 (no race conditions)
        assert counter["value"] == 100
    
    def test_thread_lock_timeout(self, mock_segment):
        """Test thread lock respects timeout."""
        # Acquire lock in thread and hold it
        def blocker():
            with mock_segment._thread_lock:
                time.sleep(0.5)
        
        t = threading.Thread(target=blocker)
        t.start()
        time.sleep(0.05)  # Let thread acquire lock
        
        # Try to acquire with timeout
        start_time = time.time()
        acquired = mock_segment._thread_lock.acquire(timeout=0.2)
        duration = time.time() - start_time
        
        # Should timeout around 0.2s
        assert not acquired or duration < 0.3
        
        if acquired:
            mock_segment._thread_lock.release()
        
        t.join()


@pytest.mark.unit
class TestSegmentTransactions:
    """Test transactional segment operations."""
    
    @pytest.fixture
    def mock_segment(self):
        """Mock SharedMemorySegment with transaction support."""
        with patch('vyra_base.com.external.shared_memory.segment.posix_ipc'):
            with patch('vyra_base.com.external.shared_memory.segment.POSIX_IPC_AVAILABLE', True):
                segment = SharedMemorySegment(
                    name="/test_segment",
                    size=1024,
                    create=True
                )
                # Mock internals
                segment._segment = Mock()
                segment._segment.fd = 1
                segment._mutex = Mock()
                segment._mutex.acquire = Mock(return_value=True)
                segment._mutex.release = Mock()
                
                # Mock storage
                segment._storage = {"value": 10}
                
                def mock_unsafe_read():
                    from vyra_base.com.external.shared_memory.serialization import MessageType
                    return (
                        MessageType.REQUEST,
                        segment._storage.copy(),
                        int(time.time())
                    )
                
                def mock_unsafe_write(data, msg_type=None):
                    segment._storage.update(data)
                    return True
                
                segment._unsafe_read = Mock(side_effect=mock_unsafe_read)
                segment._unsafe_write = Mock(side_effect=mock_unsafe_write)
                
                yield segment
                
                # Cleanup
                segment.close()
    
    def test_transaction_commit(self, mock_segment):
        """Test transaction commits successfully."""
        # Execute transaction
        with mock_segment.transaction() as (current_value, commit):
            assert current_value["value"] == 10
            
            # Modify and commit
            new_value = {"value": current_value["value"] + 5}
            commit(new_value)
        
        # Value should be updated
        assert mock_segment._storage["value"] == 15
    
    def test_transaction_abort_without_commit(self, mock_segment):
        """Test transaction aborts if commit not called."""
        # Execute transaction without commit
        with mock_segment.transaction() as (current_value, commit):
            assert current_value["value"] == 10
            # Don't call commit()
        
        # Value should remain unchanged
        assert mock_segment._storage["value"] == 10
    
    def test_transaction_rollback_on_error(self, mock_segment):
        """Test transaction rolls back on error."""
        # Modify write to fail on certain values
        original_write = mock_segment._unsafe_write
        
        def failing_write(data, msg_type=None):
            if data.get("value", 0) > 20:
                raise TransportError("Value too large")
            return original_write(data, msg_type)
        
        mock_segment._unsafe_write = Mock(side_effect=failing_write)
        
        # Execute transaction that fails
        try:
            with mock_segment.transaction() as (current_value, commit):
                new_value = {"value": 25}
                commit(new_value)  # This should fail
        except TransportError:
            pass
        
        # Value should be rolled back to original
        assert mock_segment._storage["value"] == 10
    
    def test_transaction_isolation(self, mock_segment):
        """Test transactions are isolated."""
        results = []
        
        def transactional_increment(idx):
            try:
                with mock_segment.transaction() as (current_value, commit):
                    time.sleep(0.01)  # Simulate work
                    new_value = {"value": current_value["value"] + 1}
                    commit(new_value)
                    results.append(idx)
            except Exception as e:
                results.append(f"error_{idx}: {e}")
        
        # Run 5 transactions concurrently
        threads = [threading.Thread(target=transactional_increment, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # All transactions should succeed
        assert len(results) == 5
        
        # Final value should be 15 (10 + 5)
        assert mock_segment._storage["value"] == 15


@pytest.mark.unit
class TestSegmentContextManager:
    """Test segment context manager."""
    
    def test_context_manager_closes_segment(self):
        """Test context manager closes segment on exit."""
        with patch('vyra_base.com.external.shared_memory.segment.posix_ipc'):
            with patch('vyra_base.com.external.shared_memory.segment.POSIX_IPC_AVAILABLE', True):
                # Use context manager
                with SharedMemorySegment("/test", 1024, create=True) as segment:
                    segment._segment = Mock()
                    segment._mutex = Mock()
                    assert segment._segment is not None
                
                # Should be closed after exit
                # Note: close() sets _segment to None
                assert segment._segment is None
    
    def test_context_manager_unlinks_on_create(self):
        """Test context manager unlinks segment when created."""
        with patch('vyra_base.com.external.shared_memory.segment.posix_ipc') as mock_ipc:
            with patch('vyra_base.com.external.shared_memory.segment.POSIX_IPC_AVAILABLE', True):
                mock_ipc.unlink_shared_memory = Mock()
                mock_ipc.unlink_semaphore = Mock()
                
                # Use context manager with create=True
                with SharedMemorySegment("/test", 1024, create=True) as segment:
                    segment._segment = Mock()
                    segment._mutex = Mock()
                
                # Should call unlink
                mock_ipc.unlink_shared_memory.assert_called_once()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
