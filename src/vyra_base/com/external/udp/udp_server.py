"""
Async UDP server for VYRA external communication.

Example::

    server = AsyncUdpServer(host="0.0.0.0", port=5000)

    @server.on_datagram
    async def handle(data: bytes, addr: tuple):
        print(f"{addr}: {data}")

    await server.start()

Multicast example::

    server = AsyncUdpServer(host="0.0.0.0", port=5000,
                             multicast_group="239.0.0.1")
    await server.start()
"""
from __future__ import annotations

import asyncio
import json
import logging
import socket
import struct
from typing import Any, Callable, Coroutine, Optional, Tuple

logger = logging.getLogger(__name__)

_DatagramCallback = Callable[[bytes, Tuple[str, int]], Coroutine[Any, Any, None]]


class _UdpServerProtocol(asyncio.DatagramProtocol):
    """Internal asyncio protocol that dispatches datagrams to a callback queue."""

    def __init__(self, queue: asyncio.Queue) -> None:
        self._queue = queue
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        self._queue.put_nowait((data, addr))

    def error_received(self, exc: Exception) -> None:
        logger.warning("⚠️ UDP server error_received: %s", exc)

    def connection_lost(self, exc: Optional[Exception]) -> None:
        logger.debug("UDP server connection_lost: %s", exc)


class AsyncUdpServer:
    """Async UDP server with optional multicast group join.

    :param host: Interface to listen on (``"0.0.0.0"`` for all).
    :type host: str
    :param port: UDP port to bind.
    :type port: int
    :param multicast_group: IPv4 multicast group to join after bind.
    :type multicast_group: str, optional
    :param recv_size: Internal queue item limit (0 = unlimited).
    :type recv_size: int
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5000,
        *,
        multicast_group: Optional[str] = None,
        queue_maxsize: int = 0,
    ):
        self._host = host
        self._port = port
        self._multicast_group = multicast_group
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=queue_maxsize)
        self._transport: Optional[asyncio.DatagramTransport] = None
        self._protocol: Optional[_UdpServerProtocol] = None
        self._dispatch_task: Optional[asyncio.Task] = None
        self._datagram_cb: Optional[_DatagramCallback] = None
        self._running: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        """``True`` while the server is bound and dispatching datagrams."""
        return self._running

    @property
    def listen_address(self) -> str:
        """Human-readable ``host:port`` string."""
        return f"{self._host}:{self._port}"

    # ------------------------------------------------------------------
    # Callback registration
    # ------------------------------------------------------------------

    def on_datagram(self, cb: _DatagramCallback) -> _DatagramCallback:
        """Decorator / direct setter for the datagram callback.

        Callback signature: ``async def handler(data: bytes, addr: tuple)``
        """
        self._datagram_cb = cb
        return cb

    set_datagram_callback = on_datagram  # alias

    # ------------------------------------------------------------------
    # Server lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Bind to configured address and start dispatching.

        :raises RuntimeError: When the server is already running.
        """
        if self._running:
            raise RuntimeError("UDP server is already running.")

        loop = asyncio.get_event_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._host, self._port))

        if self._multicast_group:
            mreq = struct.pack(
                "4sL",
                socket.inet_aton(self._multicast_group),
                socket.INADDR_ANY,
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            logger.info("📡 UDP server joined multicast group %s", self._multicast_group)

        sock.setblocking(False)
        self._protocol = _UdpServerProtocol(self._queue)
        self._transport, _ = await loop.create_datagram_endpoint(  # type: ignore[assignment]
            lambda: self._protocol,
            sock=sock,
        )
        self._running = True
        self._dispatch_task = asyncio.ensure_future(self._dispatch_loop())
        logger.info("✅ UDP server listening on %s", self.listen_address)

    async def stop(self) -> None:
        """Stop the server and release the socket."""
        self._running = False
        if self._dispatch_task is not None:
            self._dispatch_task.cancel()
            try:
                await self._dispatch_task
            except asyncio.CancelledError:
                pass
            self._dispatch_task = None
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        logger.info("🛑 UDP server on %s stopped.", self.listen_address)

    # ------------------------------------------------------------------
    # Send helpers
    # ------------------------------------------------------------------

    async def send(self, data: bytes, host: str, port: int) -> None:
        """Send a unicast reply datagram.

        :param data: Raw bytes to send.
        :param host: Destination host.
        :param port: Destination port.
        """
        if self._transport is None:
            raise RuntimeError("UDP server is not running.")
        self._transport.sendto(data, (host, port))

    async def send_json(
        self,
        obj: Any,
        host: str,
        port: int,
        *,
        encoding: str = "utf-8",
    ) -> None:
        """Serialize *obj* as JSON and send to *host:port*."""
        await self.send(json.dumps(obj).encode(encoding), host, port)

    async def broadcast(self, data: bytes, port: int) -> None:
        """Send *data* to the subnet broadcast address.

        :param data: Raw bytes to broadcast.
        :param port: Destination port.
        """
        if self._transport is None:
            raise RuntimeError("UDP server is not running.")
        # Enable broadcast on underlying socket
        sock: socket.socket = self._transport.get_extra_info("socket")
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._transport.sendto(data, ("<broadcast>", port))

    # ------------------------------------------------------------------
    # Internal dispatch loop
    # ------------------------------------------------------------------

    async def _dispatch_loop(self) -> None:
        while self._running:
            try:
                data, addr = await asyncio.wait_for(self._queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break

            if self._datagram_cb is not None:
                try:
                    await self._datagram_cb(data, addr)
                except Exception as exc:
                    logger.error("❌ on_datagram callback error: %s", exc)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    async def __aenter__(self) -> "AsyncUdpServer":
        await self.start()
        return self

    async def __aexit__(self, *_: Any) -> None:
        await self.stop()
