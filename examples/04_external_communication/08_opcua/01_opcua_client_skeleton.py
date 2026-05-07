"""OPC UA client skeleton example using the external communication package."""

import asyncio

from vyra_base.com.external.opcua import OPCUA_AVAILABLE


async def main() -> None:
    """Try to connect to a local OPC UA endpoint and fail gracefully if absent."""
    print(f"OPCUA_AVAILABLE={OPCUA_AVAILABLE}")
    if not OPCUA_AVAILABLE:
        print("OPC UA is unavailable (install optional dependencies).")
        return

    from vyra_base.com.external.opcua import OpcuaClient

    client = OpcuaClient(endpoint="opc.tcp://127.0.0.1:4840")
    try:
        await client.connect()
        print("Connected to OPC UA server.")
    except Exception as exc:
        print(f"OPC UA connect failed (expected without server): {exc}")
    finally:
        try:
            await client.close()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
