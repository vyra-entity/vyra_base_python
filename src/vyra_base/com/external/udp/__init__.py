"""
UDP transport layer for VYRA external communication.

Provides standalone async UDP client and server classes based on
:mod:`asyncio` (no extra dependencies required).  Both classes support
optional IPv4 multicast group join.
"""
from vyra_base.com.external.udp.udp_client import AsyncUdpClient
from vyra_base.com.external.udp.udp_server import AsyncUdpServer

__all__ = ["AsyncUdpClient", "AsyncUdpServer"]
