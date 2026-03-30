"""
VYRA Security Framework - Secure Server Example
================================================

Demonstrates how to create a secure VYRA entity that:
1. Accepts access requests via the built-in ``request_access`` Callable
2. Validates incoming messages with HMAC signatures
3. Enforces access levels on services

This entity acts as "Module B" that other components authenticate with.

Run:
    python examples/security/secure_server_node.py
"""
import asyncio
import logging
from pathlib import Path

from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


class SecureServer(SecurityManager):
    """
    Example secure VYRA component using the SecurityManager mixin.

    SecurityManager exposes a ``request_access`` Callable automatically once
    ``setup_security_service()`` is called.  The HMAC level (4) is used here,
    which means all callers must provide a valid HMAC signature.
    """

    def __init__(self) -> None:
        """Initialize SecureServer with HMAC security level."""
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC,
            session_duration_seconds=3600,
            module_passwords={
                # Map module_id → shared password for Level 3+ auth.
                # "client-module-uuid": "s3cr3t-passw0rd",
            },
        )
        logger.info("SecureServer initialised (max level: HMAC)")

    async def handle_data(self, payload: dict, session_token: str) -> dict:
        """
        Example business-logic handler that requires a valid session.

        :param payload: Incoming request payload.
        :param session_token: Session token obtained from ``request_access``.
        :returns: Response dict.
        :raises ValueError: If the session token is invalid.
        """
        session = self.sessions.get(session_token)
        if session is None or session.is_expired():
            raise ValueError("Invalid or expired session token")

        return {"status": "ok", "echo": payload.get("data")}


async def main() -> None:
    """Run the demo: create server, simulate an access request, call handle_data."""
    server = SecureServer()

    # Simulate an access request (normally sent by a remote component):
    dummy_request = {
        "module_name": "demo_client",
        "module_id": "demo-client-0001",
        "requested_level": SecurityLevel.HMAC,
        "password_hash": "",
    }

    result = await server.request_access_impl(dummy_request)
    logger.info("Access result: %s", result)

    if result.get("success"):
        token = result["session_token"]
        response = await server.handle_data({"data": "hello"}, token)
        logger.info("handle_data response: %s", response)


if __name__ == "__main__":
    asyncio.run(main())
