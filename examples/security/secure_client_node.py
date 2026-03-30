"""
VYRA Security Framework - Secure Client Example
================================================

Demonstrates how a client component:
1. Authenticates with a SecurityManager-based server via ``request_access``
2. Stores the session token for subsequent calls
3. Signs outgoing payloads at HMAC level

This component acts as "Module A" that authenticates with Module B.

Run:
    python examples/security/secure_client_node.py
"""
import asyncio
import logging
import uuid
import hmac as _hmac
import hashlib

from vyra_base.security.security_levels import SecurityLevel

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


class SecureClient:
    """
    Example client component that authenticates with a SecurityManager server.

    After calling :meth:`authenticate`, ``session_token`` and ``hmac_key``
    are available for signing subsequent requests.
    """

    def __init__(self, module_name: str = "demo_client") -> None:
        """
        Initialise the client.

        :param module_name: Human-readable module name sent during authentication.
        """
        self.module_name = module_name
        self.module_id = str(uuid.uuid4())
        self.session_token: str | None = None
        self.hmac_key: str | None = None
        self.granted_level: int = SecurityLevel.NONE

    async def authenticate(self, server) -> bool:
        """
        Run the request_access handshake against *server*.

        :param server: An object that exposes ``request_access_impl``.
        :returns: ``True`` if access was granted.
        """
        logger.info("Requesting access (module_id=%s)…", self.module_id)
        result = await server.request_access_impl(
            {
                "module_name": self.module_name,
                "module_id": self.module_id,
                "requested_level": SecurityLevel.HMAC,
                "password_hash": "",
            }
        )

        if result.get("success"):
            self.session_token = result["session_token"]
            self.hmac_key = result.get("hmac_key", "")
            self.granted_level = result.get("granted_level", SecurityLevel.NONE)
            logger.info(
                "Access granted — level=%s, token=%s…",
                self.granted_level,
                self.session_token[:12],
            )
            return True

        logger.error("Access denied: %s", result.get("message"))
        return False

    def sign_payload(self, payload: str) -> str:
        """
        Create an HMAC-SHA256 signature for *payload*.

        :param payload: The serialised request payload (e.g. JSON string).
        :returns: Hex-encoded HMAC signature.
        :raises RuntimeError: If no HMAC key is available (not yet authenticated).
        """
        if not self.hmac_key:
            raise RuntimeError("Not authenticated — call authenticate() first")
        return _hmac.new(
            self.hmac_key.encode(),
            payload.encode(),
            hashlib.sha256,
        ).hexdigest()


async def main() -> None:
    """End-to-end demo: spin up a server, authenticate a client, make a call."""
    from vyra_base.security.security_manager import SecurityManager

    server = SecurityManager(
        max_security_level=SecurityLevel.HMAC,
        module_passwords={},
    )
    client = SecureClient(module_name="demo_client")

    authenticated = await client.authenticate(server)
    if not authenticated:
        return

    payload_str = '{"action": "get_status"}'
    sig = client.sign_payload(payload_str)
    logger.info("Signed payload signature: %s…", sig[:16])
    logger.info("Client ready — token=%s…", client.session_token[:12] if client.session_token else "N/A")


if __name__ == "__main__":
