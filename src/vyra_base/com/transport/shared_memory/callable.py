"""
Shared Memory Callable Implementation

Request-Response communication over POSIX shared memory.
Implements zero-copy IPC with deterministic latency.
"""
import asyncio
import logging
import time
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import CallableError, TimeoutError, TransportError
from vyra_base.com.transport.shared_memory.segment import SharedMemorySegment
from vyra_base.com.transport.shared_memory.serialization import (
    SharedMemorySerializer,
    SerializationFormat,
    MessageType,
    calculate_segment_size,
)
from vyra_base.com.transport.shared_memory.discovery import get_discovery

logger = logging.getLogger(__name__)


class SharedMemoryCallable(VyraCallable):
    """
    Callable interface over shared memory.
    
    Features:
    - Zero-copy data transfer
    - Deterministic latency (<500µs)
    - Request-response pattern
    - Automatic timeout handling
    
    Architecture:
        Client writes request → Server reads and processes → Server writes response
    
    Example:
        >>> # Server side
        >>> async def handle_request(request):
        ...     return {"result": request["value"] * 2}
        >>> 
        >>> callable = SharedMemoryCallable(
        ...     "calculate",
        ...     callback=handle_request,
        ...     module_name="math_service"
        ... )
        >>> await callable.initialize()
        >>> 
        >>> # Client side
        >>> callable = SharedMemoryCallable(
        ...     "calculate",
        ...     module_name="math_service"
        ... )
        >>> await callable.initialize()
        >>> result = await callable.call({"value": 21}, timeout=5.0)
        >>> print(result)  # {"result": 42}
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        module_name: str = "default",
        segment_size: int = 4096,
        serialization_format: SerializationFormat = SerializationFormat.JSON,
        **kwargs
    ):
        """
        Initialize shared memory callable.
        
        Args:
            name: Callable name
            callback: Server-side callback function
            module_name: Module name for discovery
            segment_size: Segment size in bytes (default: 4KB)
            serialization_format: Serialization format
            **kwargs: Additional metadata
        """
        super().__init__(name, callback, ProtocolType.SHARED_MEMORY, **kwargs)
        self.module_name = module_name
        self.segment_size = segment_size
        self.serialization_format = serialization_format
        
        # Segment names
        self._request_segment_name = f"/vyra_{module_name}_{name}_req"
        self._response_segment_name = f"/vyra_{module_name}_{name}_resp"
        
        # Shared memory segments
        self._request_segment: Optional[SharedMemorySegment] = None
        self._response_segment: Optional[SharedMemorySegment] = None
        
        # Serializer
        self._serializer = SharedMemorySerializer(serialization_format)
        
        # Discovery
        self._discovery = get_discovery()
        
        # Server mode flag
        self._is_server = callback is not None
        
        # Background task for server
        self._server_task: Optional[asyncio.Task] = None
        self._running = False
    
    async def initialize(self) -> bool:
        """
        Initialize shared memory callable.
        
        For server: Creates segments and starts listening
        For client: Discovers and connects to existing segments
        
        Returns:
            bool: True if initialization successful
        """
        try:
            if self._is_server:
                return await self._initialize_server()
            else:
                return await self._initialize_client()
        except Exception as e:
            logger.error(f"❌ Failed to initialize callable '{self.name}': {e}")
            return False
    
    async def _initialize_server(self) -> bool:
        """Initialize as server."""
        logger.info(
            f"🚀 Initializing callable server: "
            f"{self.module_name}.{self.name}"
        )
        
        # Create request segment (server reads from here)
        self._request_segment = SharedMemorySegment(
            self._request_segment_name,
            self.segment_size,
            create=True,
            serialization_format=self.serialization_format
        )
        
        # Create response segment (server writes to here)
        self._response_segment = SharedMemorySegment(
            self._response_segment_name,
            self.segment_size,
            create=True,
            serialization_format=self.serialization_format
        )
        
        # Register with discovery
        self._discovery.register(
            self.module_name,
            self.name,
            self._request_segment_name,
            self.segment_size
        )
        
        # Start server loop
        self._running = True
        self._server_task = asyncio.create_task(self._server_loop())
        
        self._initialized = True
        logger.info(
            f"✅ Callable server ready: {self.module_name}.{self.name}"
        )
        return True
    
    async def _initialize_client(self) -> bool:
        """Initialize as client."""
        logger.info(
            f"🔍 Discovering callable: {self.module_name}.{self.name}"
        )
        
        # Discover segment
        segment_info = self._discovery.discover(self.module_name, self.name)
        if not segment_info:
            raise CallableError(
                f"Callable not found: {self.module_name}.{self.name}"
            )
        
        # Connect to request segment (client writes to here)
        self._request_segment = SharedMemorySegment(
            self._request_segment_name,
            self.segment_size,
            create=False,
            serialization_format=self.serialization_format
        )
        
        # Connect to response segment
        self._response_segment = SharedMemorySegment(
            self._response_segment_name,
            self.segment_size,
            create=False,
            serialization_format=self.serialization_format
        )
        
        self._initialized = True
        logger.info(
            f"✅ Connected to callable: {self.module_name}.{self.name}"
        )
        return True
    
    async def _server_loop(self) -> None:
        """Server loop: continuously process requests."""
        logger.debug(f"🔄 Server loop started for {self.name}")
        
        while self._running:
            try:
                # Check for request (non-blocking poll)
                if not self._request_segment:
                    raise CallableError("Request segment not initialized")
                request_data = self._request_segment.read(timeout=0.1)
                
                if request_data is None:
                    # No request, continue polling
                    await asyncio.sleep(0.001)  # 1ms sleep
                    continue
                
                # Process request
                logger.debug(f"📥 Received request on {self.name}")
                
                try:
                    # Call user callback
                    if not self.callback:
                        raise CallableError("Callback not set")
                    if asyncio.iscoroutinefunction(self.callback):
                        response = await self.callback(request_data)
                    else:
                        response = self.callback(request_data)
                    
                    # Write response
                    if not self._response_segment:
                        raise CallableError("Response segment not initialized")
                    self._response_segment.write(response, MessageType.RESPONSE)
                    logger.debug(f"📤 Sent response on {self.name}")
                    
                except Exception as e:
                    # Send error response
                    error_response = {
                        "error": str(e),
                        "type": type(e).__name__
                    }

                    if self._response_segment:
                        self._response_segment.write(
                            error_response,
                            MessageType.ERROR
                        )
                    logger.error(f"❌ Error processing request: {e}")
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ Server loop error: {e}")
                await asyncio.sleep(0.1)
        
        logger.debug(f"🛑 Server loop stopped for {self.name}")
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call the remote callable.
        
        Args:
            request: Request data (must be JSON-serializable)
            timeout: Timeout in seconds (default: 5.0)
            
        Returns:
            Response data
            
        Raises:
            CallableError: If not initialized or not a client
            TimeoutError: If timeout exceeded
            TransportError: If communication fails
        """
        if not self._initialized:
            raise CallableError(f"Callable '{self.name}' not initialized")
        
        if self._is_server:
            raise CallableError("Cannot call from server side")
        
        start_time = time.time()
        
        if not self._request_segment:
            raise TypeError("Could not get self._request_segment. Is None")
        
        if not self._response_segment:
            raise TypeError("Could not get self._response_segment. Is None")
        
        try:
            # Write request
            logger.debug(f"📤 Sending request to {self.name}")
            self._request_segment.write(request, MessageType.REQUEST)
            
            # Wait for response with timeout
            response = None
            deadline = start_time + timeout
            
            while time.time() < deadline:
                response = self._response_segment.read(timeout=0.1)
                
                if response is not None:
                    break
                
                # Small sleep to avoid busy-wait
                await asyncio.sleep(0.001)
            
            if response is None:
                elapsed = time.time() - start_time
                raise TimeoutError(
                    f"Callable '{self.name}' timeout after {elapsed:.2f}s"
                )
            
            # Check for error response
            if isinstance(response, dict) and "error" in response:
                raise CallableError(
                    f"Remote error: {response.get('error', 'Empty-Error')} ({response.get('type', 'Unknown')})"
                )
            
            elapsed = time.time() - start_time
            logger.debug(f"✅ Call completed in {elapsed*1000:.2f}ms")
            
            return response
            
        except TimeoutError:
            raise
        except Exception as e:
            raise TransportError(f"Failed to call '{self.name}': {e}")
    
    async def shutdown(self) -> None:
        """Shutdown callable and cleanup resources."""
        logger.info(f"🛑 Shutting down callable: {self.name}")
        
        # Stop server loop
        if self._server_task:
            self._running = False
            self._server_task.cancel()
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass
        
        # Cleanup segments
        if self._request_segment:
            if self._is_server:
                self._request_segment.unlink()
            self._request_segment.close()
        
        if self._response_segment:
            if self._is_server:
                self._response_segment.unlink()
            self._response_segment.close()
        
        # Unregister from discovery
        if self._is_server:
            self._discovery.unregister(self.module_name, self.name)
        
        self._initialized = False
        logger.info(f"✅ Callable shutdown complete: {self.name}")
