"""
Example 07-01: Async TCP Echo Server + Client

Demonstrates AsyncTcpServer and AsyncTcpClient:
- Callback-based message handling
- JSON send/receive over TCP
- Automatic server broadcast to all clients
- Context manager usage
"""
import asyncio
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)

HOST = "127.0.0.1"
PORT = 19001


async def run_server():
    """Start a TCP server that echoes messages back in JSON."""
    from vyra_base.com.external.tcp.tcp_server import AsyncTcpServer

    server = AsyncTcpServer(host=HOST, port=PORT)

    @server.on_message
    async def handle(client_id: str, data: bytes):
        import json
        try:
            obj = json.loads(data.decode().strip())
            obj["echo"] = True
            await server.send_json(client_id, obj)
            logger.info("Server echoed to %s: %s", client_id, obj)
        except Exception as e:
            logger.warning("Server parse error: %s", e)

    @server.on_connected
    async def on_connect(client_id, host, port):
        logger.info("Client connected: %s from %s:%d", client_id, host, port)

    @server.on_disconnected
    async def on_disconnect(client_id):
        logger.info("Client disconnected: %s", client_id)

    return server


async def run_client():
    """Connect to server, send JSON, receive echo."""
    from vyra_base.com.external.tcp.tcp_client import AsyncTcpClient

    async with AsyncTcpClient(host=HOST, port=PORT, reconnect=False) as client:
        logger.info("Client connected to %s", client.remote_address)

        for i in range(3):
            msg = {"index": i, "value": i * 10}
            await client.send_json(msg)
            reply = await asyncio.wait_for(client.receive_json(), timeout=3.0)
            logger.info("Client received echo #%d: %s", i, reply)
            assert reply["echo"] is True
            await asyncio.sleep(0.1)

    logger.info("✅ TCP client finished.")


async def main():
    server = await run_server()
    await server.start()

    # Give server a moment to bind
    await asyncio.sleep(0.1)

    await run_client()
    await server.stop()
    logger.info("✅ TCP echo demo complete.")


if __name__ == "__main__":
    asyncio.run(main())
