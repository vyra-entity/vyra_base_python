"""Minimal REST client example with graceful fallback when aiohttp is missing."""

import asyncio

from vyra_base.com.external import REST_AVAILABLE


async def main() -> None:
    """Show REST availability and demonstrate optional import usage."""
    print(f"REST_AVAILABLE={REST_AVAILABLE}")
    if not REST_AVAILABLE:
        print("REST client is unavailable (install optional dependencies).")
        return

    from vyra_base.com.external.rest import RestClient

    client = RestClient(base_url="https://httpbin.org")
    response = await client.get("/get")
    print("GET /get status:", response.get("status"))


if __name__ == "__main__":
    asyncio.run(main())
