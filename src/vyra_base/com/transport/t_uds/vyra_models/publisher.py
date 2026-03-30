"""
UDS Publisher Implementation

Uses Unix Datagram Sockets for message broadcasting.
"""

import logging
import socket
import json
import hashlib
import os
from typing import Optional, Any
from pathlib import Path

from vyra_base.com.core.types import VyraPublisher, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR

logger = logging.getLogger(__name__)


class VyraPublisherImpl(VyraPublisher):
    """
    Unix Domain Socket publisher implementation using datagram sockets.
    
    Pattern: Sends messages to subscribers via datagram socket.
    Each subscriber listens on their own socket path.
    Publisher maintains list of subscriber paths (broadcast pattern).
    
    Socket path: /tmp/vyra_sockets/pub_{module}_{topic}.sock
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        message_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(name, topic_builder, ProtocolType.UDS, **kwargs)
        self.message_type = message_type
        self._module_name = module_name
        self._socket: Optional[socket.socket] = None
        self._socket_dir = UDS_SOCKET_DIR
        self._socket_path = None  # Set in initialize()
        self._subscribers = []  # List of subscriber socket paths
        
    @staticmethod
    def _short_socket_name(prefix: str, module_name: str, topic_name: str) -> str:
        """Build a short socket filename that stays within AF_UNIX 108-char limit.

        Uses a SHA-256 hash (12 hex chars) of the full identifier so the
        resulting path ``<socket_dir>/<result>`` never exceeds ~60 chars.
        """
        raw = f"{module_name}_{topic_name}"
        digest = hashlib.sha256(raw.encode()).hexdigest()[:12]
        hint = module_name[:20]
        return f"{prefix}_{hint}_{digest}.sock"

    async def initialize(self) -> bool:
        """Initialize UDS publisher."""
        try:
            topic_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            sock_name = self._short_socket_name("pub", self._module_name, topic_name)
            self._socket_path = self._socket_dir / sock_name
            
            # TODO: Initialize UDS publisher socket
            logger.warning(f"{self.name} - UDS publisher initialization not fully implemented")
            
            self._initialized = True
            logger.info(f"✅ UDS Publisher '{self.name}' initialized: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize UDS Publisher '{self.name}': {e}")
            raise InterfaceError(f"Publisher initialization failed: {e}")
    
    async def cleanup(self) -> None:
        """Cleanup UDS publisher resources."""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._socket_path and self._socket_path.exists():
            self._socket_path.unlink()
        logger.debug(f"✅ VyraPublisher '{self.name}' cleaned up")
    
    async def shutdown(self) -> None:
        """Shutdown UDS publisher."""
        await self.cleanup()
        self._initialized = False
        
    async def publish(self, message: Any) -> bool:
        """Publish message (async)."""
        if not self._initialized:
            raise InterfaceError(f"VyraPublisher '{self.name}' not initialized")
        
        try:
            # TODO: Implement UDS publish via datagram broadcast
            logger.warning(f"⚠️ VyraPublisher '{self.name}' - UDS publish not fully implemented")
            return True
        except Exception as e:
            logger.error(f"❌ VyraPublisher '{self.name}' failed to publish: {e}")
            return False
