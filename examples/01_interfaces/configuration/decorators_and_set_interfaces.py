"""Complete callback decoration and set_interfaces loading example."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from vyra_base.com import ProtocolType, remote_actionServer, remote_publisher, remote_service
from vyra_base.defaults.entries import (
    FunctionConfigBaseTypes,
    FunctionConfigDisplaystyle,
    FunctionConfigEntry,
    FunctionConfigTags,
)


class InterfaceCallbacks:
    """Example callbacks for message, service, and action interfaces."""

    @remote_service(name="read_status", protocols=[ProtocolType.ZENOH])
    async def read_status(self, request: dict, response: Any = None) -> dict:
        return {"ok": True, "source": request.get("source", "unknown")}

    @remote_publisher(name="status_stream", protocols=[ProtocolType.ZENOH])
    async def status_stream(self, message: dict) -> None:
        # In real modules this method is replaced by publisher binding.
        return None

    @remote_actionServer.on_goal(name="move_axis", protocols=[ProtocolType.ZENOH])
    async def move_axis_on_goal(self, goal_request: dict) -> bool:
        return goal_request.get("distance", 0) >= 0

    @remote_actionServer.on_cancel(name="move_axis", protocols=[ProtocolType.ZENOH])
    async def move_axis_on_cancel(self, goal_handle: Any) -> bool:
        return True

    @remote_actionServer.execute(name="move_axis", protocols=[ProtocolType.ZENOH])
    async def move_axis_execute(self, goal_handle: Any) -> dict:
        return {"result": "done", "success": True}


@dataclass
class FakeEntity:
    """Minimal stand-in showing how set_interfaces() is called."""

    async def set_interfaces(self, settings: list[FunctionConfigEntry]) -> None:
        print(f"Loaded {len(settings)} interfaces")


def build_interface_settings(callbacks: InterfaceCallbacks) -> list[FunctionConfigEntry]:
    """Build full FunctionConfigEntry list for message/service/action."""
    return [
        FunctionConfigEntry(
            tags=[FunctionConfigTags.ZENOH],
            type=FunctionConfigBaseTypes.service.value,
            interfacetypes=dict,
            functionname="read_status",
            displayname="Read Status",
            description="Returns module status.",
            displaystyle=FunctionConfigDisplaystyle(visible=True, published=True),
            callbacks={"response": callbacks.read_status},
        ),
        FunctionConfigEntry(
            tags=[FunctionConfigTags.ZENOH],
            type=FunctionConfigBaseTypes.message.value,
            interfacetypes=dict,
            functionname="status_stream",
            displayname="Status Stream",
            description="Publishes status updates.",
            displaystyle=FunctionConfigDisplaystyle(visible=True, published=True),
            callbacks={"response": callbacks.status_stream},
        ),
        FunctionConfigEntry(
            tags=[FunctionConfigTags.ZENOH],
            type=FunctionConfigBaseTypes.action.value,
            interfacetypes=dict,
            functionname="move_axis",
            displayname="Move Axis",
            description="Starts a move action.",
            displaystyle=FunctionConfigDisplaystyle(visible=True, published=True),
            callbacks={
                "on_goal": callbacks.move_axis_on_goal,
                "on_cancel": callbacks.move_axis_on_cancel,
                "execute": callbacks.move_axis_execute,
            },
        ),
    ]


async def main() -> None:
    """Demonstrate decorator usage and settings loading via set_interfaces."""
    callbacks = InterfaceCallbacks()
    settings = build_interface_settings(callbacks)

    entity = FakeEntity()
    await entity.set_interfaces(settings)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
