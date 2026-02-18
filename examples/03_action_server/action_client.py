"""
Example 03b — Action Client
============================

Shows how to send a goal to a running action server and receive feedback.

Assumption: action_server.py is running (or the module is deployed).

Run:
    python examples/03_action_server/action_client.py
"""

import asyncio
import logging

from vyra_base.com import InterfaceFactory, ProtocolType

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


def on_feedback(feedback: dict):
    """Called for every feedback message from the server."""
    logger.info("  ↳ Feedback: %d%% (step %s)", feedback.get("progress", 0), feedback.get("step"))


async def main():
    # Create an action client for "processor/process_data"
    client = await InterfaceFactory.create_action_client(
        name="process_data",
        namespace="processor",
        protocols=[ProtocolType.ZENOH],
        feedback_callback=on_feedback,
    )

    if client is None:
        logger.error("No action client created — is Zenoh available?")
        return

    # Send a goal
    goal = {"dataset": "sensor_logs_2026.csv", "steps": 20}
    logger.info("Sending goal: %s", goal)

    result = await client.send_goal(goal, timeout=30.0)
    logger.info("Result: %s", result)

    logger.info("✅ action_client.py completed")


if __name__ == "__main__":
    asyncio.run(main())
