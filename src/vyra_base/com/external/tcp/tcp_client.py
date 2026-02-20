"""
Async TCP client for VYRA external communication.

Example::

    client = AsyncTcpClient(host="10.0.0.1", port=9000)
    await client.connect()
    await client.send_json({"cmd": "START"})
    response = await client.receive_json()
    await client.close()
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

_DEFAULT_RECONNECT_DELAY = 3.0
_DEFAULT_CONNECT_TIMEOUT = 10.0
_DEFAULT_RECV_SIZE = 4096


class AsyncTcpClient:
    """Fully async TCP client with automatic reconnection.

    :param host: Remote host name or IP address.
    :type host: str
    :param port: Remote TCP port.
    :type port: int
    :param connect_timeout: Seconds before a connection attempt times out.
    :type connect_timeout: float
    :param reconnect: Enable automatic reconnection on connection loss.
    :type reconnect: bool
    :param reconnect_delay: Seconds to wait between reconnection attempts.
    :type reconnect_delay: float
    :param recv_size: Default buffer size for :meth:`receive` calls.
    :type recv_size: int
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout: float = _DEFAULT_CONNECT_TIMEOUT,
        reconnect: bool = True,
        reconnect_delay: float = _DEFAULT_RECONNECT_DELAY,
        recv_size: int = _DEFAULT_RECV_SIZE,
    ):
        self._host = host
        self._port = port
        self._connect_timeout = connect_timeout
        self._reconnect = reconnect
        self._reconnect_delay = reconnect_delay
        self._recv_size = recv_size
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._connected: bool = False
        self._closed: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """``True`` when an active connection is established."""
        return self._connected and self._writer is not None

    @property
    def remote_address(self) -> str:
        """Human-readable ``host:port`` string."""
        return f"{self._host}:{self._port}"

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        """Open a TCP connection to *host:port*.

        :raises ConnectionError: When the connection attempt fails.
        """
        if self._closed:
            raise ConnectionError("Client has been permanently closed.")
        try:
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection(self._host, self._port),
                timeout=self._connect_timeout,
            )
            self._connected = True
            logger.info("✅ TCP connected to %s", self.remote_address)
        except (TimeoutError, OSError) as exc:
            self._connected = False
            raise ConnectionError(
                f"TCP connect to {self.remote_address} failed: {exc}"
            ) from exc

    async def close(self) -> None:
        """Close the connection gracefully."""
        self._closed = True
        self._connected = False
        if self._writer is not None:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
            finally:
                self._writer = None
                self._reader = None
        logger.info("🔌 TCP connection to %s closed.", self.remote_address)

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    async def send(self, data: bytes) -> None:
        """Send raw *data* bytes.

        Attempts one automatic reconnection if the connection is lost.

        :param data: Raw bytes to send.
        :type data: bytes
        :raises ConnectionError: When not connected and reconnection fails.
        """
        await self._ensure_connected()
        try:
            self._writer.write(data)  # type: ignore[union-attr]
            await self._writer.drain()  # type: ignore[union-attr]
        except (OSError, AttributeError) as exc:
            logger.warning("⚠️ TCP send failed (%s), reconnecting…", exc)
            self._connected = False
            if self._reconnect and not self._closed:
                await asyncio.sleep(self._reconnect_delay)
                await self.connect()
                self._writer.write(data)  # type: ignore[union-attr]
                await self._writer.drain()  # type: ignore[union-attr]
            else:
                raise

    async def send_json(self, obj: Any, *, encoding: str = "utf-8") -> None:
        """Serialize *obj* as JSON and send it with a trailing newline.

        :param obj: JSON-serializable object.
        :param encoding: Character encoding, default ``utf-8``.
        """
        payload = json.dumps(obj).encode(encoding) + b"\n"
        await self.send(payload)

    # ------------------------------------------------------------------
    # Receive
    # ------------------------------------------------------------------

    async def receive(self, size: Optional[int] = None) -> bytes:
        """Read up to *size* bytes from the server.

        :param size: Maximum bytes to read. Defaults to :attr:`recv_size`.
        :type size: int, optional
        :return: Received bytes (empty bytes on EOF).
        :rtype: bytes
        :raises ConnectionError: When not connected.
        """
        await self._ensure_connected()
        read_size = size or self._recv_size
        data = await self._reader.read(read_size)  # type: ignore[union-attr]
        if not data:
            self._connected = False
        return data

    async def receive_line(self, *, encoding: str = "utf-8") -> str:
        """Read a newline-terminated string from the server.

        :return: Decoded string without the trailing newline.
        :rtype: str
        """
        await self._ensure_connected()
        raw = await self._reader.readline()  # type: ignore[union-attr]
        if not raw:
            self._connected = False
            return ""
        return raw.decode(encoding).rstrip("\n")

    async def receive_json(self, *, encoding: str = "utf-8") -> Any:
        """Read a newline-terminated JSON line from the server.

        :return: Deserialized Python object.
        :rtype: Any
        """
        line = await self.receive_line(encoding=encoding)
        return json.loads(line) if line else None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    async def _ensure_connected(self) -> None:
        if not self.is_connected:
            if self._reconnect and not self._closed:
                await self.connect()
            else:
                raise ConnectionError(
                    f"Not connected to {self.remote_address}."
                )

    async def __aenter__(self) -> "AsyncTcpClient":
        await self.connect()
        return self

    async def __aexit__(self, *_: Any) -> None:
        await self.close()
