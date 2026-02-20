"""
TCP transport layer for VYRA external communication.

Provides standalone async TCP client and server classes based on
:mod:`asyncio` (no extra dependencies required).
"""
from vyra_base.com.external.tcp.tcp_client import AsyncTcpClient
from vyra_base.com.external.tcp.tcp_server import AsyncTcpServer

__all__ = ["AsyncTcpClient", "AsyncTcpServer"]
