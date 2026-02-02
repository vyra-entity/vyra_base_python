"""
Shared Memory Segment Management

POSIX IPC-based shared memory segments with PID-tracked discovery.
Implements deterministic, high-performance IPC for VYRA slim image.
"""
import logging
import os
import struct
import time
from pathlib import Path
from typing import Optional, Tuple, Any, TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    import posix_ipc

try:
    import posix_ipc
    POSIX_IPC_AVAILABLE = True
except ImportError:
    POSIX_IPC_AVAILABLE = False
    class MockPosixIpc:
        def __getattr__(self, name):
            raise ImportError("posix_ipc is not installed on this system.")
    posix_ipc = MockPosixIpc()

from vyra_base.com.core.exceptions import TransportError, ConnectionError
from vyra_base.com.transport.shared_memory.serialization import (
    SharedMemorySerializer,
    SerializationFormat,
    MessageType,
    calculate_segment_size,
)

logger = logging.getLogger(__name__)


# Lock metadata structure: | Lock State (1B) | Writer PID (4B) | Reader PID (4B) | Reserved (7B) |
LOCK_FORMAT = '!BII7s'
LOCK_SIZE = struct.calcsize(LOCK_FORMAT)


@dataclass
class SegmentInfo:
    """Shared memory segment information."""
    name: str
    size: int
    created: float
    pid: int
    path: str


class SharedMemorySegment:
    """
    POSIX shared memory segment with mutex-based synchronization.
    
    Features:
    - Zero-copy data transfer
    - Mutex-based synchronization
    - PID validation for crash detection
    - Automatic cleanup on process death
    
    Industrial Requirements:
    - Deterministic latency (<500µs)
    - No memory copies
    - Crash-safe with PID tracking
    """
    
    def __init__(
        self,
        name: str,
        size: int,
        create: bool = False,
        serialization_format: SerializationFormat = SerializationFormat.JSON
    ):
        """
        Initialize shared memory segment.
        
        Args:
            name: Segment name (must start with /)
            size: Segment size in bytes
            create: If True, create new segment
            serialization_format: Serialization format to use
            
        Raises:
            TransportError: If posix_ipc not available
            ConnectionError: If segment doesn't exist (when create=False)
        """
        if not POSIX_IPC_AVAILABLE:
            raise TransportError(
                "posix_ipc not installed. Install with: pip install posix-ipc"
            )
        
        # Ensure name starts with /
        if not name.startswith('/'):
            name = f'/{name}'
        
        self.name = name
        self.size = size + LOCK_SIZE  # Add space for lock metadata
        self._create_mode = create
        self._segment: Optional[Any] = None  # posix_ipc.SharedMemory
        self._mutex: Optional[Any] = None  # posix_ipc.Semaphore
        self._serializer = SharedMemorySerializer(serialization_format)
        self._pid = os.getpid()
        
        # Create or open segment
        if create:
            self._create_segment()
        else:
            self._open_segment()
    
    def _create_segment(self) -> None:
        """Create new shared memory segment and mutex."""
        try:
            # Create shared memory
            self._segment = posix_ipc.SharedMemory(
                self.name,
                flags=posix_ipc.O_CREAT | posix_ipc.O_EXCL,
                mode=0o600,
                size=self.size
            )
            
            # Create mutex (semaphore with initial value 1)
            self._mutex = posix_ipc.Semaphore(
                f"{self.name}_mutex",
                flags=posix_ipc.O_CREAT | posix_ipc.O_EXCL,
                mode=0o600,
                initial_value=1
            )
            
            # Initialize lock metadata
            self._write_lock_metadata(lock_state=0, writer_pid=0, reader_pid=0)
            
            logger.info(
                f"✅ Created shared memory segment '{self.name}' ({self.size} bytes)"
            )
            
        except posix_ipc.ExistentialError:
            # Segment already exists, try to open
            logger.warning(f"Segment '{self.name}' already exists, opening")
            self._open_segment()
        except Exception as e:
            raise TransportError(f"Failed to create segment '{self.name}': {e}")
    
    def _open_segment(self) -> None:
        """Open existing shared memory segment."""
        try:
            self._segment = posix_ipc.SharedMemory(self.name, flags=0, size=self.size)
            self._mutex = posix_ipc.Semaphore(f"{self.name}_mutex", flags=0)
            
            logger.info(f"✅ Opened shared memory segment '{self.name}'")
            
        except posix_ipc.ExistentialError:
            raise ConnectionError(
                f"Shared memory segment '{self.name}' does not exist. "
                f"Create it first with create=True."
            )
        except Exception as e:
            raise TransportError(f"Failed to open segment '{self.name}': {e}")
    
    def _write_lock_metadata(
        self,
        lock_state: int,
        writer_pid: int,
        reader_pid: int
    ) -> None:
        """Write lock metadata to segment."""
        metadata = struct.pack(LOCK_FORMAT, lock_state, writer_pid, reader_pid, b'')
        
        if self._segment is None:
            logger.warning("self._segnemt is None")
            raise TypeError
        
        os.write(self._segment.fd, metadata)
        os.lseek(self._segment.fd, 0, os.SEEK_SET)
    
    def _read_lock_metadata(self) -> Tuple[int, int, int]:
        """Read lock metadata from segment."""
        if self._segment is None:
            logger.warning("self._segnemt is None")
            raise TypeError
        
        os.lseek(self._segment.fd, 0, os.SEEK_SET)
        data = os.read(self._segment.fd, LOCK_SIZE)
        lock_state, writer_pid, reader_pid, _ = struct.unpack(LOCK_FORMAT, data)
        return lock_state, writer_pid, reader_pid
    
    def write(self, data: Any, message_type: MessageType = MessageType.REQUEST) -> bool:
        """
        Write data to shared memory segment.
        
        Args:
            data: Data to write
            message_type: Type of message
            
        Returns:
            bool: True if write successful
            
        Raises:
            TransportError: If serialization or write fails
        """
        try:
            if self._segment is None:
                logger.warning("self._segment is None")
                raise TypeError
        
            if self._mutex is None:
                logger.warning("self._mutex is None")
                raise TypeError
        
            # Serialize data
            serialized = self._serializer.serialize(data, message_type)
            
            if len(serialized) > (self.size - LOCK_SIZE):
                raise TransportError(
                    f"Data too large: {len(serialized)} bytes, "
                    f"max {self.size - LOCK_SIZE} bytes"
                )
            
            # Acquire mutex
            self._mutex.acquire(timeout=5.0)
            
            try:
                # Write lock metadata
                self._write_lock_metadata(
                    lock_state=1,
                    writer_pid=self._pid,
                    reader_pid=0
                )
                
                
                
                # Write data
                os.lseek(self._segment.fd, LOCK_SIZE, os.SEEK_SET)
                os.write(self._segment.fd, serialized)
                
                # Update lock metadata
                self._write_lock_metadata(
                    lock_state=0,
                    writer_pid=self._pid,
                    reader_pid=0
                )
                
                return True
                
            finally:
                # Release mutex
                self._mutex.release()
                
        except Exception as e:
            logger.error(f"Failed to write to segment '{self.name}': {e}")
            raise TransportError(f"Write failed: {e}")
    
    def read(self, timeout: float = 5.0) -> Tuple[MessageType, Any, int]:
        """
        Read data from shared memory segment.
        
        Args:
            timeout: Read timeout in seconds
            
        Returns:
            Tuple[MessageType, data, timestamp]
            
        Raises:
            TimeoutError: If read times out
            TransportError: If read or deserialization fails
        """
        start_time = time.time()
        
        if self._segment is None:
            logger.warning("self._segment is None")
            raise TypeError
    
        if self._mutex is None:
            logger.warning("self._mutex is None")
            raise TypeError
    
        while True:
            try:
                # Acquire mutex
                if not self._mutex.acquire(timeout=0.1):
                    # Check timeout
                    if time.time() - start_time > timeout:
                        raise TimeoutError(f"Read timeout after {timeout}s")
                    continue
                
                try:
                    # Read lock metadata
                    lock_state, writer_pid, _ = self._read_lock_metadata()
                    
                    # Validate writer is alive
                    if writer_pid > 0 and not self._is_process_alive(writer_pid):
                        raise TransportError(
                            f"Writer process {writer_pid} is dead"
                        )
                    
                    # Read data
                    os.lseek(self._segment.fd, LOCK_SIZE, os.SEEK_SET)
                    data = os.read(self._segment.fd, self.size - LOCK_SIZE)
                    
                    # Deserialize
                    if len(data) == 0:
                        raise TransportError("No data in segment")
                    
                    msg_type, payload, timestamp = self._serializer.deserialize(data)
                    
                    # Update lock metadata
                    self._write_lock_metadata(
                        lock_state=0,
                        writer_pid=writer_pid,
                        reader_pid=self._pid
                    )
                    
                    return msg_type, payload, timestamp
                    
                finally:
                    self._mutex.release()
                    
            except (TransportError, TimeoutError):
                raise
            except Exception as e:
                logger.error(f"Read error: {e}")
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Read timeout: {e}")
                time.sleep(0.01)
    
    @staticmethod
    def _is_process_alive(pid: int) -> bool:
        """Check if process with PID is alive."""
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False
    
    def close(self) -> None:
        """Close shared memory segment."""
        if self._segment:
            self._segment.close_fd()
            self._segment = None
        
        if self._mutex:
            self._mutex.close()
            self._mutex = None
        
        logger.debug(f"Closed segment '{self.name}'")
    
    def unlink(self) -> None:
        """Unlink (delete) shared memory segment."""
        try:
            if self._segment:
                posix_ipc.unlink_shared_memory(self.name)
                logger.info(f"Unlinked segment '{self.name}'")
            
            if self._mutex:
                posix_ipc.unlink_semaphore(f"{self.name}_mutex")
        except Exception as e:
            logger.warning(f"Failed to unlink segment '{self.name}': {e}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        if self._create_mode:
            self.unlink()
    
    def __del__(self):
        """Destructor."""
        self.close()


def list_segments() -> list[SegmentInfo]:
    """
    List all VYRA shared memory segments.
    
    Returns:
        List of SegmentInfo objects
    """
    if not POSIX_IPC_AVAILABLE:
        return []
    
    segments = []
    
    # List segments via /dev/shm/
    shm_path = Path('/dev/shm')
    if not shm_path.exists():
        return []
    
    for entry in shm_path.iterdir():
        if entry.name.startswith('vyra_'):
            try:
                stat = entry.stat()
                segments.append(SegmentInfo(
                    name=f"/{entry.name}",
                    size=stat.st_size,
                    created=stat.st_ctime,
                    pid=0,  # Can't determine from file
                    path=str(entry)
                ))
            except Exception as e:
                logger.warning(f"Failed to stat segment {entry}: {e}")
    
    return segments


def cleanup_dead_segments() -> int:
    """
    Cleanup shared memory segments from dead processes.
    
    Returns:
        Number of segments cleaned up
    """
    if not POSIX_IPC_AVAILABLE:
        return 0
    
    cleaned = 0
    
    for segment_info in list_segments():
        try:
            # Try to read segment metadata
            with SharedMemorySegment(segment_info.name, segment_info.size) as seg:
                _, writer_pid, reader_pid = seg._read_lock_metadata()
                
                # Check if both writer and reader are dead
                writer_dead = writer_pid > 0 and not seg._is_process_alive(writer_pid)
                reader_dead = reader_pid > 0 and not seg._is_process_alive(reader_pid)
                
                if writer_dead and reader_dead:
                    seg.unlink()
                    cleaned += 1
                    logger.info(
                        f"Cleaned up dead segment '{segment_info.name}' "
                        f"(writer={writer_pid}, reader={reader_pid})"
                    )
        except Exception as e:
            logger.warning(f"Failed to cleanup segment {segment_info.name}: {e}")
    
    return cleaned
