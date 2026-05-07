"""gRPC client skeleton with optional runtime demo and availability guard."""

from __future__ import annotations

import asyncio
import os

from vyra_base.com.external import GRPC_CLIENT_AVAILABLE, GRPC_SERVER_AVAILABLE


def print_availability() -> None:
    """Print gRPC availability flags from vyra_base.com.external."""
    print(f"GRPC_CLIENT_AVAILABLE={GRPC_CLIENT_AVAILABLE}")
    print(f"GRPC_SERVER_AVAILABLE={GRPC_SERVER_AVAILABLE}")


def should_run_demo() -> bool:
    """Return True when the runtime demo should run against a real endpoint."""
    return os.getenv("RUN_GRPC_DEMO", "0") == "1"


async def run_client_skeleton() -> None:
    """Show a safe gRPC client flow that can be enabled with an env toggle."""
    print_availability()

    if not GRPC_CLIENT_AVAILABLE:
        print("gRPC client is unavailable (install optional dependency: grpcio).")
        return

    from vyra_base.com.external.grpc import GrpcClient

    target = os.getenv("GRPC_TARGET", "localhost:50051")
    method = os.getenv("GRPC_METHOD", "/example.ExampleService/Ping")
    payload = os.getenv("GRPC_PAYLOAD", '{"ping": true}').encode("utf-8")

    print("Configured gRPC skeleton:")
    print(f"- target: {target}")
    print(f"- method: {method}")
    print("- payload bytes: configured")

    if not should_run_demo():
        print("Set RUN_GRPC_DEMO=1 to execute connect/call/close against your server.")
        return

    client = GrpcClient(target=target, timeout=5.0)
    try:
        await client.connect()
        response = await client.call_method(method=method, request=payload)
        print(f"gRPC call succeeded: {len(response)} response bytes")
    except Exception as exc:
        print(f"gRPC demo failed: {exc}")
    finally:
        await client.close()


if __name__ == "__main__":
    asyncio.run(run_client_skeleton())
