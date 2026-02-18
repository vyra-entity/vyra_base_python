"""
Example 03a — Action Server (multi-callback pattern)
=====================================================

Shows how to implement a long-running action using ``@remote_actionServer``.

An action has three phases:
  1. Goal received     → on_goal()   — accept or reject
  2. Execution         → execute()   — do the work, publish feedback
  3. Cancellation      → on_cancel() — clean up if the client cancels

Your class MUST inherit from ``IActionHandler``.

Run:
    python examples/03_action_server/action_server.py
"""

import asyncio
import logging

from vyra_base.com import (
    remote_actionServer,
    IActionHandler,
    IGoalHandle,
    bind_decorated_callbacks,
    InterfaceFactory,
    ProtocolType,
)
from vyra_base.com.transport.t_zenoh.vyra_models import VyraActionServerImpl

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


class DataProcessingComponent(IActionHandler):
    """
    Long-running action that processes a dataset and reports progress.

    Goal request format:  {"dataset": "<name>", "steps": <int>}
    Feedback format:      {"progress": <int>, "step": <int>}
    Result format:        {"status": "done", "processed_steps": <int>}
    """

    @remote_actionServer.on_goal(
        name="process_data",
        protocols=[ProtocolType.ZENOH],
        namespace="processor",
    )
    async def on_goal(self, goal_request: dict) -> bool:
        """
        Called when a client sends a new goal.
        Return True to accept, False to reject.
        """
        if not goal_request.get("dataset"):
            logger.warning("Rejecting goal — no dataset specified")
            return False
        logger.info("Accepting goal: dataset=%s, steps=%s",
                    goal_request.get("dataset"), goal_request.get("steps", 10))
        return True

    @remote_actionServer.execute(name="process_data")
    async def execute(self, goal_handle: IGoalHandle):
        """
        Main execution loop.
        Use goal_handle.publish_feedback() to report progress.
        Call goal_handle.succeed() / .abort() when done.
        """
        goal = goal_handle.goal
        steps = goal.get("steps", 10)

        for step in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                logger.info("Cancellation requested at step %d/%d", step, steps)
                goal_handle.canceled()
                return {"status": "canceled", "processed_steps": step - 1}

            await asyncio.sleep(0.1)  # simulate work
            progress = int(step / steps * 100)
            await goal_handle.publish_feedback({"progress": progress, "step": step})
            logger.info("Step %d/%d  (%d%%)", step, steps, progress)

        goal_handle.succeed()
        logger.info("✅ Action completed: %d steps processed", steps)
        return {"status": "done", "processed_steps": steps}

    @remote_actionServer.on_cancel(name="process_data")
    async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
        """
        Called when the client requests cancellation.
        Return True to accept cancellation.
        """
        logger.info("Cancel request received — accepting")
        return True


async def main():
    processor = DataProcessingComponent()

    # Phase 2: bind callbacks
    bind_decorated_callbacks(processor, namespace="processor")

    logger.info("Action server 'processor/process_data' is registered.")
    logger.info("In a VYRA module, InterfaceFactory creates live action servers automatically.")
    logger.info("✅ action_server.py completed")


if __name__ == "__main__":
    asyncio.run(main())
