"""
UDS Communication Layer

Core Unix Domain Socket functionality.
This layer contains the functional implementation of UDS socket operations.

Components:
    - UnixSocket: Unix domain socket management
    - Socket constants and utilities

Usage:
    >>> from vyra_base.com.transport.t_uds.communication import UnixSocket
    >>> 
    >>> # Server
    >>> socket = UnixSocket("/tmp/my_socket.sock")
    >>> await socket.listen(handler)
    >>> 
    >>> # Client
    >>> socket = UnixSocket("/tmp/my_socket.sock")
    >>> await socket.connect()
    >>> response = await socket.send_request(data)
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.transport.t_uds.communication.socket import (
        UnixSocket,
        UDS_SOCKET_DIR,
        MESSAGE_HEADER_FORMAT,
        MESSAGE_HEADER_SIZE,
    )
    UDS_COMMUNICATION_AVAILABLE = True
    logger.debug("✅ UDS communication layer available")
    
except ImportError as e:
    UnixSocket = None
    UDS_SOCKET_DIR = None
    MESSAGE_HEADER_FORMAT = None
    MESSAGE_HEADER_SIZE = None
    UDS_COMMUNICATION_AVAILABLE = False
    logger.debug(f"⚠️  UDS communication layer unavailable: {e}")

__all__ = [
    "UnixSocket",
    "UDS_SOCKET_DIR",
    "MESSAGE_HEADER_FORMAT",
    "MESSAGE_HEADER_SIZE",
    "UDS_COMMUNICATION_AVAILABLE",
]
