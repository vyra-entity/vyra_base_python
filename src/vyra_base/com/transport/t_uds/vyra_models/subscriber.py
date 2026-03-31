"""
UDS Subscriber Implementation

Uses Unix Datagram Sockets for message reception.
"""

import logging
import socket
import json
import asyncio
import hashlib
import uuid
from typing import Optional, Any, Callable, Awaitable
from pathlib import Path

from vyra_base.com.core.types import InterfaceType, VyraSubscriber, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR

logger = logging.getLogger(__name__)


class VyraSubscriberImpl(VyraSubscriber):
    """
    Unix Domain Socket subscriber implementation using datagram sockets.
    
    Pattern: Listens on own socket for messages from publishers.
    Socket path: /tmp/vyra_sockets/sub_{module}_{topic}_{instance_id}.sock
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        subscriber_callback: Callable[[Any], Awaitable[None]],
        message_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(name, topic_builder, subscriber_callback, ProtocolType.UDS, **kwargs)
        self.message_type = message_type
        self._module_name = module_name
        self._instance_id = str(uuid.uuid4())[:8]
        self._socket: Optional[socket.socket] = None
        self._socket_dir = UDS_SOCKET_DIR
        self._socket_path = None  # Set in initialize()
        self._listen_task: Optional[asyncio.Task] = None
        
    @staticmethod
    def _short_socket_name(prefix: str, module_name: str, topic_name: str, instance_id: str) -> str:
        """Build a short socket filename that stays within AF_UNIX 108-char limit.

        Uses a SHA-256 hash (12 hex chars) of the full identifier so the
        resulting path ``<socket_dir>/<result>`` never exceeds ~60 chars.
        """
        raw = f"{module_name}_{topic_name}_{instance_id}"
        digest = hashlib.sha256(raw.encode()).hexdigest()[:12]
        # Keep a human-readable hint (first 20 chars of module name)
        hint = module_name[:20]
        return f"{prefix}_{hint}_{digest}.sock"

    async def initialize(self) -> bool:
        """Initialize UDS subscriber."""
        try:
            topic_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            sock_name = self._short_socket_name("sub", self._module_name, topic_name, self._instance_id)
            self._socket_path = self._socket_dir / sock_name
            
            # Create socket directory
            self._socket_dir.mkdir(parents=True, exist_ok=True)
            
            # Create datagram socket
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._socket.setblocking(False)  # Non-blocking for async
            
            # Remove old socket file if exists
            if self._socket_path.exists():
                self._socket_path.unlink()
                
            # Bind to socket path
            self._socket.bind(str(self._socket_path))
            
            # Write .topic meta file so publishers can discover us
            self._topic_meta_path = Path(f"{self._socket_path}.topic")
            self._topic_meta_path.write_text(topic_name)
            
            self._initialized = True
            logger.info(f"✅ VyraSubscriber '{self.name}' initialized: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize VyraSubscriber '{self.name}': {e}")
            raise InterfaceError(f"VyraSubscriber initialization failed: {e}")
    
    async def subscribe(self) -> bool:
        """
        Start listening for messages.
        
        Returns:
            True on success
        """
        if not self._socket:
            logger.error("❌ VyraSubscriber not initialized")
            return False
            
        try:
            # Start listen task
            self._listen_task = asyncio.create_task(self._listen_loop())
            
            logger.info(f"📡 VyraSubscriber listening on: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ VyraSubscriber subscribe failed: {e}")
            return False
    
    async def _listen_loop(self):
        """Background task to receive messages."""
        try:
            loop = asyncio.get_event_loop()
            
            if not self._socket:
                logger.error("❌ VyraSubscriber socket not available in listen loop")
                return

            if not self.subscriber_callback:
                logger.error("❌ VyraSubscriber callback not set")
                return

            while True:
                try:
                    # Async wait for data (non-blocking socket)
                    data = await loop.sock_recv(self._socket, 65536)
                    
                    if data:
                        # Deserialize message
                        message_dict = json.loads(data.decode('utf-8'))
                        
                        # Convert dict to msg_type instance
                        if self.message_type:
                            try:
                                # Try to instantiate message type from dict
                                if hasattr(self.message_type, '__call__'):
                                    # Callable class constructor
                                    message_obj = self.message_type(**message_dict)
                                else:
                                    message_obj = message_dict
                            except Exception as e:
                                logger.warning(f"Could not convert dict to {self.message_type}: {e}")
                                message_obj = message_dict
                        else:
                            message_obj = message_dict
                        
                        # Call user callback
                        await self.subscriber_callback(message_obj)
                        
                except BlockingIOError:
                    await asyncio.sleep(0.01)
                except Exception as e:
                    logger.error(f"❌ VyraSubscriber message handling failed: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            logger.debug("VyraSubscriber listen loop canceled")
        except Exception as e:
            logger.error(f"❌ VyraSubscriber listen loop failed: {e}")
    
    async def cleanup(self):
        """Cleanup UDS resources."""
        if self._listen_task:
            self._listen_task.cancel()
            try:
                await self._listen_task
            except asyncio.CancelledError:
                pass
            self._listen_task = None
            
        if self._socket:
            self._socket.close()
            self._socket = None
            
        # Remove socket file
        if self._socket_path and self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except Exception as e:
                logger.warning(f"❌ Failed to remove socket file: {e}")
        
        # Remove .topic meta file
        if hasattr(self, '_topic_meta_path') and self._topic_meta_path and self._topic_meta_path.exists():
            try:
                self._topic_meta_path.unlink()
            except Exception as e:
                logger.warning(f"❌ Failed to remove topic meta file: {e}")
                
        logger.info(f"🔄 VyraSubscriber cleaned up: {self._socket_path}")
    
    @property
    def interface_type(self) -> InterfaceType:
        return InterfaceType.SUBSCRIBER
