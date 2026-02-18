"""
Example 02a — Publisher
========================

Shows how to create a publisher using the ``@remote_publisher`` decorator
and how to trigger publishing programmatically.

Run:
    python examples/02_publisher_subscriber/publisher.py
"""

import asyncio
import logging

from vyra_base.com import (
    remote_publisher,
    bind_decorated_callbacks,
    InterfaceFactory,
    ProtocolType,
)

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


class SensorComponent:
    """
    Publishes temperature readings on the "sensors/temperature" topic.
    """

    def __init__(self):
        self._publisher = None   # set after InterfaceFactory creates the interface

    @remote_publisher(
        name="temperature",
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
        namespace="sensors",
    )
    async def publish_temperature(self, value: float, unit: str = "°C"):
        """Return the message dict that will be published."""
        return {"value": value, "unit": unit, "sensor_id": "sensor_01"}

    async def run(self):
        """Simulate periodic sensor readings."""
        import random

        # Phase 2: bind callbacks
        bind_decorated_callbacks(self, namespace="sensors")

        # In a full module the interface is created automatically.
        # Here we show how to manually create it and use it:
        self._publisher = await InterfaceFactory.create_publisher(
            name="temperature",
            namespace="sensors",
            protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
        )

        if self._publisher is None:
            logger.warning("No publisher created — no protocol available")
            return

        for i in range(5):
            temp = round(20.0 + random.uniform(-2, 2), 2)
            logger.info("Publishing temperature: %.2f °C", temp)
            await self._publisher.publish({"value": temp, "unit": "°C", "sensor_id": "sensor_01"})
            await asyncio.sleep(0.5)

        logger.info("✅ publisher.py completed")


async def main():
    sensor = SensorComponent()
    await sensor.run()


if __name__ == "__main__":
    asyncio.run(main())
