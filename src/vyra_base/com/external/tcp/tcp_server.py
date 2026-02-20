"""
Async TCP server for VYRA external communication.

Example::

    server = AsyncTcpServer(host="0.0.0.0", port=9000)

    @server.on_message
    async def handle(client_id: str, data: bytes):
        print(f"{client_id}: {data}")

    await server.start()
    # … later
    await server.stop()
"""
from __future__ import annotations

import asyncio
import json
import logging
import uuid
from typing import Any, Callable, Coroutine, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

_ClientId = str
_MessageCallback = Callable[[_ClientId, bytes], Coroutine[Any, Any, None]]
_ConnectCallback = Callable[[_ClientId, str, int], Coroutine[Any, Any, None]]
_DisconnectCallback = Callable[[_ClientId], Coroutine[Any, Any, None]]


class AsyncTcpServer:
    """Multi-client async TCP server.

    Callbacks are registered via the decorator helpers
    :meth:`on_message`, :meth:`on_connected`, :meth:`on_disconnected`
    or by calling :meth:`set_message_callback` etc. directly.

    :param host: Interface to listen on (``"0.0.0.0"`` for all).
    :type host: str
    :param port: TCP port to bind.
    :type port: int
    :param recv_size: Per-recv read size.
    :type recv_size: int
    :param max_clients: Maximum simultaneous connections (0 = unlimited).
    :type max_clients: int
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9000,
        *,
        recv_size: int = 4096,
        max_clients: int = 0,
    ):
        self._host = host
        self._port = port
        self._recv_size = recv_size
        self._max_clients = max_clients
        self._server: Optional[asyncio.Server] = None
        self._clients: Dict[_ClientId, Tuple[asyncio.StreamReader, asyncio.StreamWriter]] = {}
        self._message_cb: Optional[_MessageCallback] = None
        self._connect_cb: Optional[_ConnectCallback] = None
        self._disconnect_cb: Optional[_DisconnectCallback] = None
        self._running: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        """``True`` while the server is accepting connections."""
        return self._running

    @property
    def client_count(self) -> int:
        """Number of currently connected clients."""
        return len(self._clients)

    @property
    def listen_address(self) -> str:
        """Human-readable ``host:port`` string."""
        return f"{self._host}:{self._port}"

    # ------------------------------------------------------------------
    # Callback registration
    # ------------------------------------------------------------------

    def on_message(self, cb: _MessageCallback) -> _MessageCallback:
        """Decorator / direct setter for the message callback.

        Callback signature: ``async def handler(client_id: str, data: bytes)``
        """
        self._message_cb = cb
        return cb

    def on_connected(self, cb: _ConnectCallback) -> _ConnectCallback:
        """Decorator / direct setter for the client-connected callback.

        Callback signature: ``async def handler(client_id: str, host: str, port: int)``
        """
        self._connect_cb = cb
        return cb

    def on_disconnected(self, cb: _DisconnectCallback) -> _DisconnectCallback:
        """Decorator / direct setter for the client-disconnected callback.

        Callback signature: ``async def handler(client_id: str)``
        """
        self._disconnect_cb = cb
        return cb

    # Alias helpers for direct assignment
    set_message_callback = on_message
    set_connect_callback = on_connected
    set_disconnect_callback = on_disconnected

    # ------------------------------------------------------------------
    # Server lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Start listening for incoming connections.

        :raises RuntimeError: When the server is already running.
        """
        if self._running:
            raise RuntimeError("Server is already running.")
        self._server = await asyncio.start_server(
            self._handle_client,
            self._host,
            self._port,
        )
        self._running = True
        addr = self._server.sockets[0].getsockname()  # type: ignore[index]
        logger.info("✅ TCP server listening on %s:%s", addr[0], addr[1])

    async def stop(self) -> None:
        """Stop accepting new connections and close all active ones."""
        self._running = False
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        # Gracefully close all connected clients
        for cid, (_, writer) in list(self._clients.items()):
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
        self._clients.clear()
        logger.info("🛑 TCP server on %s stopped.", self.listen_address)

    # ------------------------------------------------------------------
    # Client communication
    # ------------------------------------------------------------------

    async def send(self, client_id: _ClientId, data: bytes) -> None:
        """Send raw *data* to a specific client.

        :param client_id: Opaque client identifier returned by callbacks.
        :param data: Raw bytes to transmit.
        :raises KeyError: When *client_id* is unknown.
        """
        if client_id not in self._clients:
            raise KeyError(f"Unknown client: {client_id}")
        _, writer = self._clients[client_id]
        writer.write(data)
        await writer.drain()

    async def send_json(self, client_id: _ClientId, obj: Any, *, encoding: str = "utf-8") -> None:
        """Serialize *obj* as JSON and send to *client_id*."""
        payload = json.dumps(obj).encode(encoding) + b"\n"
        await self.send(client_id, payload)

    async def broadcast(self, data: bytes) -> None:
        """Send *data* to **all** connected clients.

        Failed sends to individual clients are logged but do not abort the
        broadcast to the remaining clients.

        :param data: Raw bytes to send to every client.
        """
        for cid in list(self._clients):
            try:
                await self.send(cid, data)
            except Exception as exc:
                logger.warning("⚠️ TCP broadcast to %s failed: %s", cid, exc)

    async def broadcast_json(self, obj: Any, *, encoding: str = "utf-8") -> None:
        """Serialize *obj* as JSON and broadcast to all clients."""
        await self.broadcast(json.dumps(obj).encode(encoding) + b"\n")

    def disconnect_client(self, client_id: _ClientId) -> None:
        """Request disconnection of a specific client (non-blocking)."""
        asyncio.ensure_future(self._close_client(client_id))

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peername = writer.get_extra_info("peername", ("unknown", 0))
        client_id = str(uuid.uuid4())[:8]

        if self._max_clients and len(self._clients) >= self._max_clients:
            logger.warning(
                "⚠️ TCP server max clients reached (%d), rejecting %s",
                self._max_clients, peername
            )
            writer.close()
            return

        self._clients[client_id] = (reader, writer)
        logger.info("🔗 TCP client connected: %s from %s", client_id, peername)

        if self._connect_cb is not None:
            try:
                await self._connect_cb(client_id, peername[0], peername[1])
            except Exception as exc:
                logger.error("❌ on_connected callback error: %s", exc)

        try:
            while True:
                data = await reader.read(self._recv_size)
                if not data:
                    break
                if self._message_cb is not None:
                    try:
                        await self._message_cb(client_id, data)
                    except Exception as exc:
                        logger.error("❌ on_message callback error: %s", exc)
        except asyncio.CancelledError:
            pass
        except (ConnectionResetError, OSError) as exc:
            logger.warning("⚠️ TCP client %s disconnected abnormally: %s", client_id, exc)
        finally:
            await self._close_client(client_id)

    async def _close_client(self, client_id: _ClientId) -> None:
        if client_id not in self._clients:
            return
        _, writer = self._clients.pop(client_id)
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass
        logger.info("🔌 TCP client %s disconnected.", client_id)
        if self._disconnect_cb is not None:
            try:
                await self._disconnect_cb(client_id)
            except Exception as exc:
                logger.error("❌ on_disconnected callback error: %s", exc)

    async def __aenter__(self) -> "AsyncTcpServer":
        await self.start()
        return self

    async def __aexit__(self, *_: Any) -> None:
        await self.stop()
