"""
Unit tests for async TCP and UDP external communication classes.

These tests use loopback (127.0.0.1) connections and are marked as
``integration`` because they require an available TCP/UDP stack.
"""
import asyncio
import json
import pytest

from vyra_base.com.external.tcp.tcp_client import AsyncTcpClient
from vyra_base.com.external.tcp.tcp_server import AsyncTcpServer
from vyra_base.com.external.udp.udp_client import AsyncUdpClient
from vyra_base.com.external.udp.udp_server import AsyncUdpServer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _free_port() -> int:
    """Return a free TCP/UDP port on 127.0.0.1."""
    import socket
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ---------------------------------------------------------------------------
# AsyncTcpClient / AsyncTcpServer
# ---------------------------------------------------------------------------

@pytest.mark.integration
class TestAsyncTcpEcho:
    """End-to-end TCP echo tests with a real loopback server."""

    @pytest.mark.asyncio
    async def test_send_and_echo(self):
        port = _free_port()
        received: list[bytes] = []

        server = AsyncTcpServer(host="127.0.0.1", port=port)

        @server.on_message
        async def handle(client_id: str, data: bytes):
            received.append(data)
            await server.send(client_id, data)  # echo back

        await server.start()
        try:
            client = AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False)
            await client.connect()
            await client.send(b"ping")
            reply = await asyncio.wait_for(client.receive(), timeout=2.0)
            assert reply == b"ping"
            await client.close()
        finally:
            await server.stop()

    @pytest.mark.asyncio
    async def test_send_receive_json(self):
        port = _free_port()
        server = AsyncTcpServer(host="127.0.0.1", port=port)

        @server.on_message
        async def handle(client_id: str, data: bytes):
            obj = json.loads(data.decode().strip())
            obj["echo"] = True
            await server.send_json(client_id, obj)

        await server.start()
        try:
            client = AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False)
            await client.connect()
            await client.send_json({"cmd": "test"})
            reply = await asyncio.wait_for(client.receive_json(), timeout=2.0)
            assert reply is not None
            assert reply.get("cmd") == "test"
            assert reply.get("echo") is True
            await client.close()
        finally:
            await server.stop()

    @pytest.mark.asyncio
    async def test_broadcast(self):
        port = _free_port()
        server = AsyncTcpServer(host="127.0.0.1", port=port)
        await server.start()

        client1 = AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False)
        client2 = AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False)
        await client1.connect()
        await client2.connect()
        await asyncio.sleep(0.05)  # allow server to register both clients

        await server.broadcast(b"broadcast_msg")
        await asyncio.sleep(0.05)

        r1 = await asyncio.wait_for(client1.receive(), timeout=2.0)
        r2 = await asyncio.wait_for(client2.receive(), timeout=2.0)
        assert r1 == b"broadcast_msg"
        assert r2 == b"broadcast_msg"

        await client1.close()
        await client2.close()
        await server.stop()

    @pytest.mark.asyncio
    async def test_connect_callbacks(self):
        port = _free_port()
        connected: list[str] = []
        disconnected: list[str] = []

        server = AsyncTcpServer(host="127.0.0.1", port=port)

        @server.on_connected
        async def on_conn(client_id, host, p):
            connected.append(client_id)

        @server.on_disconnected
        async def on_disc(client_id):
            disconnected.append(client_id)

        await server.start()
        client = AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False)
        await client.connect()
        await asyncio.sleep(0.05)
        assert len(connected) == 1
        await client.close()
        await asyncio.sleep(0.05)
        assert len(disconnected) == 1
        await server.stop()

    @pytest.mark.asyncio
    async def test_server_already_running_raises(self):
        port = _free_port()
        server = AsyncTcpServer(host="127.0.0.1", port=port)
        await server.start()
        with pytest.raises(RuntimeError):
            await server.start()
        await server.stop()

    @pytest.mark.asyncio
    async def test_client_context_manager(self):
        port = _free_port()
        server = AsyncTcpServer(host="127.0.0.1", port=port)
        await server.start()
        async with AsyncTcpClient(host="127.0.0.1", port=port, reconnect=False) as client:
            assert client.is_connected
        await server.stop()


# ---------------------------------------------------------------------------
# AsyncTcpClient — unit tests (no real server)
# ---------------------------------------------------------------------------

class TestAsyncTcpClientUnit:
    def test_remote_address(self):
        client = AsyncTcpClient(host="10.0.0.1", port=9000)
        assert client.remote_address == "10.0.0.1:9000"

    def test_not_connected_initially(self):
        client = AsyncTcpClient(host="10.0.0.1", port=9000)
        assert client.is_connected is False

    @pytest.mark.asyncio
    async def test_connect_failure_raises_connection_error(self):
        client = AsyncTcpClient(
            host="127.0.0.1", port=1,  # port 1 should be refused
            connect_timeout=1.0,
            reconnect=False,
        )
        with pytest.raises(ConnectionError):
            await client.connect()


# ---------------------------------------------------------------------------
# AsyncTcpServer — unit tests
# ---------------------------------------------------------------------------

class TestAsyncTcpServerUnit:
    def test_not_running_initially(self):
        server = AsyncTcpServer()
        assert server.is_running is False

    def test_client_count_zero(self):
        server = AsyncTcpServer()
        assert server.client_count == 0

    def test_listen_address(self):
        server = AsyncTcpServer(host="0.0.0.0", port=8765)
        assert server.listen_address == "0.0.0.0:8765"


# ---------------------------------------------------------------------------
# AsyncUdpClient / AsyncUdpServer
# ---------------------------------------------------------------------------

@pytest.mark.integration
class TestAsyncUdpEcho:
    """End-to-end UDP datagram tests on loopback."""

    @pytest.mark.asyncio
    async def test_send_receive(self):
        port = _free_port()
        server = AsyncUdpServer(host="127.0.0.1", port=port)

        replies: list = []

        @server.on_datagram
        async def handle(data: bytes, addr):
            replies.append(data)
            await server.send(data, addr[0], addr[1])  # echo

        await server.start()
        client = AsyncUdpClient()
        await client.open()

        await client.send(b"hello_udp", "127.0.0.1", port)
        data, _ = await asyncio.wait_for(client.receive(), timeout=2.0)
        assert data == b"hello_udp"

        await client.close()
        await server.stop()

    @pytest.mark.asyncio
    async def test_send_receive_json(self):
        port = _free_port()
        server = AsyncUdpServer(host="127.0.0.1", port=port)

        @server.on_datagram
        async def handle(data: bytes, addr):
            obj = json.loads(data)
            obj["pong"] = True
            await server.send_json(obj, addr[0], addr[1])

        await server.start()
        async with AsyncUdpClient() as client:
            await client.send_json({"ping": True}, "127.0.0.1", port)
            obj, _ = await asyncio.wait_for(client.receive_json(), timeout=2.0)
            assert obj.get("pong") is True

        await server.stop()

    @pytest.mark.asyncio
    async def test_server_already_running_raises(self):
        port = _free_port()
        server = AsyncUdpServer(host="127.0.0.1", port=port)
        await server.start()
        with pytest.raises(RuntimeError):
            await server.start()
        await server.stop()


# ---------------------------------------------------------------------------
# AsyncUdpClient — unit tests
# ---------------------------------------------------------------------------

class TestAsyncUdpClientUnit:
    def test_not_open_initially(self):
        client = AsyncUdpClient()
        assert client.is_open is False

    @pytest.mark.asyncio
    async def test_send_without_open_raises(self):
        client = AsyncUdpClient()
        with pytest.raises(RuntimeError):
            await client.send(b"data", "127.0.0.1", 9999)


# ---------------------------------------------------------------------------
# AsyncUdpServer — unit tests
# ---------------------------------------------------------------------------

class TestAsyncUdpServerUnit:
    def test_not_running_initially(self):
        server = AsyncUdpServer()
        assert server.is_running is False

    def test_listen_address(self):
        server = AsyncUdpServer(host="0.0.0.0", port=1234)
        assert server.listen_address == "0.0.0.0:1234"
