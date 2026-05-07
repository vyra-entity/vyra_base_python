"""
Example 02b — Subscriber
=========================

Shows how to subscribe to a topic using the ``@remote_subscriber`` decorator.

This subscriber listens on the "sensors/temperature" topic published
by publisher.py (or any other module publishing to that topic).

Run:
    python examples/02_publisher_subscriber/subscriber.py
"""

import asyncio
import logging

from vyra_base.com import (
    remote_subscriber,
    bind_decorated_callbacks,
    InterfaceFactory,
    ProtocolType,
)

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


class DashboardComponent:
    """
    Subscribes to temperature readings and logs them.
    """

    @remote_subscriber(
        name="temperature",
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
        namespace="sensors",
    )
    async def on_temperature(self, message: dict):
        """Called every time a new temperature message arrives."""
        logger.info(
            "🌡  Temperature from %s: %.2f %s",
            message.get("sensor_id", "unknown"),
            message.get("value", 0.0),
            message.get("unit", "?"),
        )


async def main():
    dashboard = DashboardComponent()

    # Phase 2: bind callbacks
    bind_decorated_callbacks(dashboard, namespace="sensors")

    # Create the subscriber interface
    subscriber = await InterfaceFactory.create_subscriber(
        name="temperature",
        namespace="sensors",
        subscriber_callback=dashboard.on_temperature,
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
    )

    if subscriber is None:
        logger.warning("No subscriber created — no protocol available")
        return

    logger.info("Listening on sensors/temperature … (Ctrl+C to stop)")
    try:
        # Keep the subscriber alive for 30 seconds
        await asyncio.sleep(30)
    except asyncio.CancelledError:
        pass

    logger.info("✅ subscriber.py completed")


if __name__ == "__main__":
    asyncio.run(main())
