"""Minimal WebSocket availability check with optional connect example."""

import asyncio

from vyra_base.com.external import WEBSOCKET_AVAILABLE


async def main() -> None:
    """Show websocket availability and a non-blocking connection skeleton."""
    print(f"WEBSOCKET_AVAILABLE={WEBSOCKET_AVAILABLE}")
    if not WEBSOCKET_AVAILABLE:
        print("WebSocket client is unavailable (install optional dependencies).")
        return

    print("WebSocket support detected. Configure endpoint and call connect() as needed.")


if __name__ == "__main__":
    asyncio.run(main())
