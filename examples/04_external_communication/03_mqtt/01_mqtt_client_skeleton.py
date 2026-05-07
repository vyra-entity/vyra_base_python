"""MQTT client skeleton with optional runtime demo and availability guard."""

from __future__ import annotations

import asyncio
import os
from typing import Any

from vyra_base.com.external import MQTT_AVAILABLE


def print_availability() -> None:
    """Print MQTT availability flag from vyra_base.com.external."""
    print(f"MQTT_AVAILABLE={MQTT_AVAILABLE}")


def should_run_demo() -> bool:
    """Return True when publish/subscribe demo should run against a real broker."""
    return os.getenv("RUN_MQTT_DEMO", "0") == "1"


async def on_message(topic: str, payload: Any) -> None:
    """Handle received MQTT messages for the demo subscription."""
    print(f"Received MQTT message: {topic} -> {payload}")


async def run_client_skeleton() -> None:
    """Show a safe MQTT client flow that can be enabled with an env toggle."""
    print_availability()

    if not MQTT_AVAILABLE:
        print("MQTT client is unavailable (install optional dependency: paho-mqtt).")
        return

    from vyra_base.com.external.mqtt import MqttClient

    broker = os.getenv("MQTT_BROKER", "localhost")
    port = int(os.getenv("MQTT_PORT", "1883"))
    topic = os.getenv("MQTT_TOPIC", "vyra/example/status")

    print("Configured MQTT skeleton:")
    print(f"- broker: {broker}:{port}")
    print(f"- topic: {topic}")

    if not should_run_demo():
        print("Set RUN_MQTT_DEMO=1 to execute connect/subscribe/publish/close.")
        return

    client = MqttClient(broker=broker, port=port)
    try:
        await client.connect()
        await client.subscribe(topic=topic, callback=on_message, qos=0)
        await client.publish(topic=topic, payload={"status": "online", "source": "skeleton"})

        # Give the local callback a short window for loopback/demo brokers.
        await asyncio.sleep(0.5)
    except Exception as exc:
        print(f"MQTT demo failed: {exc}")
    finally:
        await client.close()


if __name__ == "__main__":
    asyncio.run(run_client_skeleton())
