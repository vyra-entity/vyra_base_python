"""Modbus TCP skeleton example using the external communication package."""

import asyncio

from vyra_base.com.external.modbus import MODBUS_AVAILABLE


async def main() -> None:
    """Try to open a Modbus TCP connection and handle missing server gracefully."""
    print(f"MODBUS_AVAILABLE={MODBUS_AVAILABLE}")
    if not MODBUS_AVAILABLE:
        print("Modbus is unavailable (install optional dependencies).")
        return

    from vyra_base.com.external.modbus.tcp import ModbusTCPClient

    client = ModbusTCPClient(host="127.0.0.1", port=502)
    try:
        await client.connect()
        print("Connected to Modbus server.")
    except Exception as exc:
        print(f"Modbus connect failed (expected without server): {exc}")
    finally:
        try:
            await client.close()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
