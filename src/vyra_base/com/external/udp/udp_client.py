"""
Async UDP client for VYRA external communication.

Example::

    client = AsyncUdpClient()
    await client.open()
    await client.send(b"hello", host="10.0.0.1", port=5000)
    data, addr = await client.receive()
    await client.close()

Multicast example::

    client = AsyncUdpClient(multicast_group="239.0.0.1")
    await client.open(bind_port=5000)
    await client.send_json({"event": "broadcast"}, host="239.0.0.1", port=5000)
"""
from __future__ import annotations

import asyncio
import json
import logging
import socket
import struct
from typing import Any, Optional, Tuple

logger = logging.getLogger(__name__)

_DEFAULT_RECV_SIZE = 65535


class _UdpProtocol(asyncio.DatagramProtocol):
    """Internal transport shim — feeds received datagrams into a queue."""

    def __init__(self, queue: asyncio.Queue) -> None:
        self._queue = queue
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        self._queue.put_nowait((data, addr))

    def error_received(self, exc: Exception) -> None:
        logger.warning("⚠️ UDP error_received: %s", exc)

    def connection_lost(self, exc: Optional[Exception]) -> None:
        logger.debug("UDP connection_lost: %s", exc)


class AsyncUdpClient:
    """Async UDP client with optional multicast support.

    :param multicast_group: IPv4 multicast group address.  When set,
        :meth:`open` joins the group automatically on bind.
    :type multicast_group: str, optional
    :param multicast_ttl: TTL for multicast datagrams.
    :type multicast_ttl: int
    :param recv_size: Maximum datagram size for receive operations.
    :type recv_size: int
    """

    def __init__(
        self,
        multicast_group: Optional[str] = None,
        multicast_ttl: int = 1,
        recv_size: int = _DEFAULT_RECV_SIZE,
    ):
        self._multicast_group = multicast_group
        self._multicast_ttl = multicast_ttl
        self._recv_size = recv_size
        self._transport: Optional[asyncio.DatagramTransport] = None
        self._protocol: Optional[_UdpProtocol] = None
        self._queue: asyncio.Queue = asyncio.Queue()
        self._open: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        """``True`` when the socket is ready to send/receive."""
        return self._open

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def open(
        self,
        bind_host: str = "",
        bind_port: int = 0,
    ) -> None:
        """Create the UDP socket.

        :param bind_host: Local address to bind to.  Leave empty for OS default.
        :param bind_port: Local port.  ``0`` lets the OS pick a free port.
        """
        loop = asyncio.get_event_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if self._multicast_group:
            sock.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_MULTICAST_TTL,
                self._multicast_ttl,
            )
            sock.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_MULTICAST_LOOP,
                1,
            )
            if bind_port:
                sock.bind((bind_host, bind_port))
                mreq = struct.pack(
                    "4sL",
                    socket.inet_aton(self._multicast_group),
                    socket.INADDR_ANY,
                )
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        elif bind_host or bind_port:
            sock.bind((bind_host, bind_port))

        sock.setblocking(False)
        self._protocol = _UdpProtocol(self._queue)
        self._transport, _ = await loop.create_datagram_endpoint(  # type: ignore[assignment]
            lambda: self._protocol,
            sock=sock,
        )
        self._open = True
        logger.info("✅ UDP socket opened (multicast=%s)", self._multicast_group or "no")

    async def close(self) -> None:
        """Close the UDP socket."""
        self._open = False
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        logger.info("🔌 UDP socket closed.")

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    async def send(
        self,
        data: bytes,
        host: str,
        port: int,
    ) -> None:
        """Send raw *data* to *host:port*.

        :param data: Raw bytes to send.
        :param host: Destination hostname or IP.
        :param port: Destination port.
        :raises RuntimeError: When socket is not open.
        """
        if not self._open or self._transport is None:
            raise RuntimeError("UDP socket is not open. Call await open() first.")
        self._transport.sendto(data, (host, port))

    async def send_json(
        self,
        obj: Any,
        host: str,
        port: int,
        *,
        encoding: str = "utf-8",
    ) -> None:
        """Serialize *obj* as JSON and send it.

        :param obj: JSON-serializable object.
        :param host: Destination hostname or IP.
        :param port: Destination port.
        """
        await self.send(json.dumps(obj).encode(encoding), host, port)

    # ------------------------------------------------------------------
    # Receive
    # ------------------------------------------------------------------

    async def receive(
        self, *, timeout: Optional[float] = None
    ) -> Tuple[bytes, Tuple[str, int]]:
        """Wait for the next incoming datagram.

        :param timeout: Seconds to wait.  ``None`` waits forever.
        :return: ``(data, (host, port))`` tuple.
        :raises asyncio.TimeoutError: When *timeout* expires.
        """
        if timeout is not None:
            return await asyncio.wait_for(self._queue.get(), timeout=timeout)
        return await self._queue.get()

    async def receive_json(
        self,
        *,
        timeout: Optional[float] = None,
        encoding: str = "utf-8",
    ) -> Tuple[Any, Tuple[str, int]]:
        """Wait for then next datagram and decode it as JSON.

        :return: ``(object, (host, port))`` tuple.
        """
        data, addr = await self.receive(timeout=timeout)
        return json.loads(data.decode(encoding)), addr

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    async def __aenter__(self) -> "AsyncUdpClient":
        await self.open()
        return self

    async def __aexit__(self, *_: Any) -> None:
        await self.close()
