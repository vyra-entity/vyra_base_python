"""
Unix Domain Socket (UDS) Transport

Stream-based local IPC via Unix domain sockets.
Provides low-latency request-response communication.
"""
from vyra_base.com.transport.uds.socket import (
    UnixSocket,
    UDS_SOCKET_DIR,
    MESSAGE_HEADER_FORMAT,
    MESSAGE_HEADER_SIZE,
)
from vyra_base.com.transport.uds.callable import UDSCallable
from vyra_base.com.transport.uds.provider import UDSProvider

__all__ = [
    # Socket management
    "UnixSocket",
    "UDS_SOCKET_DIR",
    "MESSAGE_HEADER_FORMAT",
    "MESSAGE_HEADER_SIZE",
    
    # Communication interfaces
    "UDSCallable",
    
    # Provider
    "UDSProvider",
]
