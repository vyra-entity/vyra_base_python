"""
Shared Memory Speaker Implementation

Publish-Subscribe communication over POSIX shared memory.
Implements fan-out pattern for multiple subscribers.
"""
import asyncio
import logging
import time
from typing import Any, Callable, Dict, List, Optional, Type

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import SpeakerError, TransportError
from vyra_base.com.transport.shared_memory.segment import SharedMemorySegment
from vyra_base.com.transport.shared_memory.serialization import (
    SharedMemorySerializer,
    SerializationFormat,
    MessageType,
)
from vyra_base.com.transport.shared_memory.discovery import get_discovery

logger = logging.getLogger(__name__)


class SharedMemorySpeaker(VyraSpeaker):
    """
    Speaker interface over shared memory (Publish-Subscribe).
    
    Features:
    - Fan-out to multiple subscribers
    - Zero-copy data transfer
    - Deterministic latency
    - Automatic subscriber discovery
    
    Architecture:
        Publisher writes to shared segment → Multiple readers poll for updates
    
    Example:
        >>> # Publisher side
        >>> speaker = SharedMemorySpeaker(
        ...     "sensor_data",
        ...     module_name="robot",
        ...     is_publisher=True
        ... )
        >>> await speaker.initialize()
        >>> await speaker.shout({"temperature": 23.5})
        >>> 
        >>> # Subscriber side
        >>> speaker = SharedMemorySpeaker(
        ...     "sensor_data",
        ...     module_name="robot",
        ...     is_publisher=False
        ... )
        >>> await speaker.initialize()
        >>> 
        >>> # Subscribe with callback
        >>> async def on_message(data):
        ...     print(f"Received: {data}")
        >>> await speaker.listen(on_message)
    """
    
    def __init__(
        self,
        name: str,
        module_name: str = "default",
        is_publisher: bool = True,
        segment_size: int = 4096,
        serialization_format: SerializationFormat = SerializationFormat.JSON,
        **kwargs
    ):
        """
        Initialize shared memory speaker.
        
        Args:
            name: Speaker/Topic name
            module_name: Module name for discovery
            is_publisher: True for publisher, False for subscriber
            segment_size: Segment size in bytes (default: 4KB)
            serialization_format: Serialization format
            **kwargs: Additional metadata
        """
        super().__init__(name, ProtocolType.SHARED_MEMORY, **kwargs)
        self.module_name = module_name
        self.is_publisher = is_publisher
        self.segment_size = segment_size
        self.serialization_format = serialization_format
        
        # Segment name
        self._segment_name = f"/vyra_{module_name}_{name}_pub"
        
        # Shared memory segment
        self._segment: Optional[SharedMemorySegment] = None
        
        # Serializer
        self._serializer = SharedMemorySerializer(serialization_format)
        
        # Discovery
        self._discovery = get_discovery()
        
        # Subscriber state
        self._listener_task: Optional[asyncio.Task] = None
        self._listening = False
        self._callback: Optional[Callable] = None
        self._last_sequence: int = 0
        
        # Publisher statistics
        self._publish_count: int = 0
        self._last_publish_time: float = 0.0
    
    async def initialize(self) -> bool:
        """
        Initialize shared memory speaker.
        
        For publisher: Creates segment and registers
        For subscriber: Discovers and connects to existing segment
        
        Returns:
            bool: True if initialization successful
        """
        try:
            if self.is_publisher:
                return await self._initialize_publisher()
            else:
                return await self._initialize_subscriber()
        except Exception as e:
            logger.error(f"❌ Failed to initialize speaker '{self.name}': {e}")
            return False
    
    async def _initialize_publisher(self) -> bool:
        """Initialize as publisher."""
        logger.info(
            f"🚀 Initializing speaker publisher: "
            f"{self.module_name}.{self.name}"
        )
        
        # Create segment
        self._segment = SharedMemorySegment(
            self._segment_name,
            self.segment_size,
            create=True,
            serialization_format=self.serialization_format
        )
        
        # Register with discovery
        self._discovery.register(
            self.module_name,
            f"{self.name}_pub",
            self._segment_name,
            self.segment_size
        )
        
        self._initialized = True
        logger.info(
            f"✅ Speaker publisher ready: {self.module_name}.{self.name}"
        )
        return True
    
    async def _initialize_subscriber(self) -> bool:
        """Initialize as subscriber."""
        logger.info(
            f"🔍 Discovering speaker: {self.module_name}.{self.name}"
        )
        
        # Discover segment
        segment_info = self._discovery.discover(self.module_name, f"{self.name}_pub")
        if not segment_info:
            raise SpeakerError(
                f"Speaker not found: {self.module_name}.{self.name}"
            )
        
        # Connect to segment
        self._segment = SharedMemorySegment(
            self._segment_name,
            self.segment_size,
            create=False,
            serialization_format=self.serialization_format
        )
        
        self._initialized = True
        logger.info(
            f"✅ Connected to speaker: {self.module_name}.{self.name}"
        )
        return True
    
    async def shout(self, message: Any) -> bool:
        """
        Publish a message.
        
        Args:
            message: Message data (must be JSON-serializable)
            
        Returns:
            bool: True if published successfully
            
        Raises:
            SpeakerError: If not initialized or not a publisher
            TransportError: If communication fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        if not self.is_publisher:
            raise SpeakerError("Cannot publish from subscriber side")
        
        try:
            # Write message with sequence number
            wrapped_message = {
                "data": message,
                "sequence": self._publish_count,
                "timestamp": time.time()
            }
            if self._segment is None:
                raise TypeError("self._segment is None")
            else:
                self._segment.write(wrapped_message, MessageType.PUBLISH)
            
            self._publish_count += 1
            self._last_publish_time = time.time()
            
            logger.debug(
                f"📤 Published message #{self._publish_count} on {self.name}"
            )
            return True
            
        except Exception as e:
            raise TransportError(f"Failed to publish on '{self.name}': {e}")
    
    async def listen(
        self,
        callback: Callable,
        poll_interval: float = 0.01
    ) -> None:
        """
        Start listening for messages (subscriber only).
        
        Args:
            callback: Callback function (async or sync)
            poll_interval: Polling interval in seconds (default: 10ms)
            
        Raises:
            SpeakerError: If not initialized or not a subscriber
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        if self.is_publisher:
            raise SpeakerError("Cannot listen from publisher side")
        
        if self._listening:
            raise SpeakerError(f"Already listening on '{self.name}'")
        
        self._callback = callback
        self._listening = True
        self._listener_task = asyncio.create_task(
            self._listener_loop(poll_interval)
        )
        
        logger.info(f"👂 Started listening on {self.name}")
    
    async def _listener_loop(self, poll_interval: float) -> None:
        """Listener loop: continuously poll for messages."""
        logger.debug(f"🔄 Listener loop started for {self.name}")
        
        while self._listening:
            try:
                # Read message (non-blocking) - returns Tuple[MessageType, data, sequence_id]
                if self._segment is None:
                    raise TypeError("self._segment is None")
                else:
                    result = self._segment.read(timeout=poll_interval)
                
                if result is None:
                    # No message, continue polling
                    await asyncio.sleep(poll_interval)
                    continue
                
                # Unpack tuple
                msg_type, wrapped_data, _seq = result
                
                if msg_type != MessageType.PUBLISH:
                    logger.warning(f"\u26a0\ufe0f Expected PUBLISH, got {msg_type}")
                    continue
                
                # Extract message fields from wrapped data
                sequence = wrapped_data.get("sequence", 0)
                data = wrapped_data.get("data")
                timestamp = wrapped_data.get("timestamp", 0.0)
                
                # Check for missed messages
                if sequence > self._last_sequence + 1:
                    missed = sequence - self._last_sequence - 1
                    logger.warning(
                        f"⚠️ Missed {missed} messages on {self.name}"
                    )
                
                self._last_sequence = sequence
                
                # Call user callback
                logger.debug(
                    f"📥 Received message #{sequence} on {self.name}"
                )
                
                try:
                    if asyncio.iscoroutinefunction(self._callback):
                        await self._callback(data)
                    elif self._callback is None:
                        raise TypeError("self._callback is None")
                    else:
                        self._callback(data)
                except Exception as e:
                    logger.error(f"❌ Callback error: {e}")
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ Listener loop error: {e}")
                await asyncio.sleep(poll_interval)
        
        logger.debug(f"🛑 Listener loop stopped for {self.name}")
    
    async def stop_listening(self) -> None:
        """Stop listening for messages."""
        if not self._listening:
            return
        
        logger.info(f"🛑 Stopping listener on {self.name}")
        
        self._listening = False
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except asyncio.CancelledError:
                pass
        
        logger.info(f"✅ Listener stopped on {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown speaker and cleanup resources."""
        logger.info(f"🛑 Shutting down speaker: {self.name}")
        
        # Stop listener if running
        if self._listening:
            await self.stop_listening()
        
        # Cleanup segment
        if self._segment:
            if self.is_publisher:
                self._segment.unlink()
            self._segment.close()
        
        # Unregister from discovery (publisher only)
        if self.is_publisher:
            self._discovery.unregister(self.module_name, f"{self.name}_pub")
        
        self._initialized = False
        logger.info(f"✅ Speaker shutdown complete: {self.name}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get speaker statistics.
        
        Returns:
            Dict with statistics
        """
        stats: Dict[str, Any] = {
            "name": self.name,
            "module": self.module_name,
            "role": "publisher" if self.is_publisher else "subscriber",
            "initialized": self._initialized
        }
        
        if self.is_publisher:
            stats["publish_count"] = self._publish_count
            stats["last_publish_time"] = self._last_publish_time
            stats["publish_rate_hz"] = self._calculate_publish_rate()
        else:
            stats["listening"] = self._listening
            stats["last_sequence"] = self._last_sequence
        
        return stats
    
    def _calculate_publish_rate(self) -> float:
        """Calculate approximate publish rate in Hz."""
        if self._publish_count < 2:
            return 0.0
        
        elapsed = time.time() - self._last_publish_time
        if elapsed < 0.001:  # Avoid division by very small numbers
            return 0.0
        
        # Simple rate calculation (messages / time)
        # More accurate would track last N messages
        return 1.0 / elapsed if elapsed > 0 else 0.0
