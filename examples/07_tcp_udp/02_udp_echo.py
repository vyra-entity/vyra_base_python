"""
Example 07-02: Async UDP Client + Server

Demonstrates AsyncUdpClient and AsyncUdpServer:
- Sending datagrams
- Receiving and echoing
- Context manager usage
"""
import asyncio
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)

import socket

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


async def main():
    from vyra_base.com.external.udp.udp_server import AsyncUdpServer
    from vyra_base.com.external.udp.udp_client import AsyncUdpClient

    port = _free_port()
    logger.info("Using UDP port %d", port)

    server = AsyncUdpServer(host="127.0.0.1", port=port)

    @server.on_datagram
    async def handle(data: bytes, addr):
        import json
        obj = json.loads(data)
        logger.info("Server received from %s: %s", addr, obj)
        obj["reply"] = True
        await server.send_json(obj, addr[0], addr[1])

    await server.start()

    async with AsyncUdpClient() as client:
        for i in range(3):
            msg = {"seq": i, "hello": "udp"}
            await client.send_json(msg, "127.0.0.1", port)
            reply, src = await asyncio.wait_for(client.receive_json(), timeout=2.0)
            logger.info("Client got reply #%d from %s: %s", i, src, reply)
            assert reply.get("reply") is True
            await asyncio.sleep(0.05)

    await server.stop()
    logger.info("✅ UDP demo complete.")


if __name__ == "__main__":
    asyncio.run(main())
