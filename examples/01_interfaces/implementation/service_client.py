"""
Example 01b — Service Client
=============================

Shows how to call a remote service using ``InterfaceFactory.create_client()``.

Assumption: a server for "calculator/add" is running on the same Zenoh session
(e.g. started with service_server.py or the module is deployed).

Run:
    python examples/01_service/service_client.py
"""

import asyncio
import logging

from vyra_base.com import InterfaceFactory, ProtocolType

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


async def main():
    # Create a client for the "add" service in the "calculator" namespace
    client = await InterfaceFactory.create_client(
        name="add",
        namespace="calculator",
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
    )

    if client is None:
        logger.error("No protocol available — ensure Zenoh or Redis is running.")
        return

    # Call the service
    request = {"x": 10, "y": 7}
    logger.info("Calling calculator/add with %s", request)
    response = await client.call(request, timeout=5.0)
    logger.info("Response: %s", response)

    # Multiple calls
    for x, y in [(3, 4), (100, 200), (0, 0)]:
        resp = await client.call({"x": x, "y": y}, timeout=5.0)
        logger.info("%s + %s = %s", x, y, resp.get("result"))

    logger.info("✅ service_client.py completed")


if __name__ == "__main__":
    asyncio.run(main())
