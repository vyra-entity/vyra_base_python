"""
Example 01a — Service Server
============================

Shows how to expose a method as a request/response service using
the ``@remote_service`` decorator (two-phase initialization pattern).

A service follows a strict request/response model:
  client sends a request dict → server processes → server returns response dict

Run this file standalone to see blueprint introspection:
    python examples/01_service/service_server.py
"""

import asyncio
import logging

from vyra_base.com import (
    remote_service,
    bind_decorated_callbacks,
    get_decorated_methods,
    InterfaceFactory,
    ProtocolType,
    AccessLevel,
)

logging.basicConfig(level=logging.INFO, format="%(name)s  %(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 1. Define your component with decorated methods
# ---------------------------------------------------------------------------

class CalculatorComponent:
    """
    Example component that exposes two services:
      - calculator/add    — adds two numbers
      - calculator/divide — divides two numbers (with error handling)
    """

    def __init__(self):
        self.call_count = 0

    @remote_service(
        name="add",
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
        namespace="calculator",
        access_level=AccessLevel.PUBLIC,
    )
    async def add(self, request: dict, response=None) -> dict:
        """Add request['x'] and request['y']."""
        self.call_count += 1
        result = request.get("x", 0) + request.get("y", 0)
        logger.info("add(%s, %s) = %s  [call #%d]", request.get("x"), request.get("y"), result, self.call_count)
        return {"result": result, "call_count": self.call_count}

    @remote_service(
        name="divide",
        protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
        namespace="calculator",
    )
    async def divide(self, request: dict, response=None) -> dict:
        """Divide request['x'] by request['y']. Returns error on division by zero."""
        self.call_count += 1
        x = request.get("x", 0)
        y = request.get("y", 1)
        if y == 0:
            logger.warning("Division by zero requested")
            return {"error": "division by zero", "call_count": self.call_count}
        result = x / y
        logger.info("divide(%s, %s) = %s  [call #%d]", x, y, result, self.call_count)
        return {"result": result, "call_count": self.call_count}


# ---------------------------------------------------------------------------
# 2. Phase 2 — bind callbacks and inspect blueprints
# ---------------------------------------------------------------------------

async def main():
    component = CalculatorComponent()

    # Introspect decorated methods (before binding)
    decorated = get_decorated_methods(component)
    logger.info("Decorated methods found: %s", [m.__name__ for m in decorated])

    # Bind callbacks to blueprints (Phase 2)
    bind_decorated_callbacks(component, namespace="calculator")
    logger.info("Callbacks bound — ready to create interfaces")

    # In a full VYRA module, InterfaceFactory.create_from_blueprint()
    # is called by the entity infrastructure automatically.
    # Here we show the manual pattern for understanding:
    #
    # blueprints = CallbackRegistry.get_all_blueprints()
    # for bp in blueprints:
    #     interface = await InterfaceFactory.create_from_blueprint(bp)
    #
    # The component's callbacks are now reachable over the configured protocols.

    logger.info("✅ service_server.py completed — component ready")


if __name__ == "__main__":
    asyncio.run(main())
