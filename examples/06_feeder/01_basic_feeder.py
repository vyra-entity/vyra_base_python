"""
Example 06-01: Basic VYRA Feeder

Demonstrates how to use StateFeeder, NewsFeeder, and ErrorFeeder without
starting a real ROS2/Zenoh stack — useful for understanding the API.

Key concepts:
- Protocol resolution from interface config JSON
- Feed buffering when publisher not yet ready
- Metrics tracking (feed_count, error_count)
"""
import asyncio
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Demonstrate FeederConfigResolver in isolation
# ---------------------------------------------------------------------------

def demo_config_resolver():
    """Show how FeederConfigResolver reads interface config files."""
    import json
    import tempfile
    import os

    from vyra_base.com.feeder.config_resolver import FeederConfigResolver

    # Create a minimal interface config JSON (mirrors module config format)
    interface_config = [
        {
            "type": "publisher",
            "functionname": "TemperatureFeed",
            "name": "temperature_feed",
            "tags": ["zenoh"],
            "filetype": ["VBASETemperatureFeed.proto"],
            "description": "Temperature readings from PLCs",
        },
        {
            "type": "publisher",
            "functionname": "PressureFeed",
            "name": "pressure_feed",
            "tags": ["redis"],
            "filetype": ["VBASEPressureFeed.proto"],
            "description": "Pressure sensor readings",
        },
    ]

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False
    ) as f:
        json.dump(interface_config, f)
        config_path = f.name

    try:
        resolver = FeederConfigResolver()

        # ✅ Exact name match
        result = resolver.resolve("TemperatureFeed", [config_path])
        if result:
            logger.info(
                "✅ Resolved: %s → protocol=%s  tags=%s",
                result.feeder_name, result.protocol, result.tags,
            )

        # ❌ Unknown name — fuzzy suggestions will be logged
        result_miss = resolver.resolve("TemperturFeed", [config_path])
        logger.info("Result for typo 'TemperturFeed': %s", result_miss)

    finally:
        os.unlink(config_path)


# ---------------------------------------------------------------------------
# Demonstrate CustomBaseFeeder without transport
# ---------------------------------------------------------------------------

async def demo_custom_feeder():
    """Show CustomBaseFeeder with validation and message building."""
    from unittest.mock import MagicMock
    from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
    from vyra_base.com.feeder.registry import register_feeder

    @register_feeder("TemperatureFeed")
    class TemperatureFeeder(CustomBaseFeeder):
        """Publishes validated temperature readings."""

        def _validate(self, raw: float) -> bool:
            return isinstance(raw, (int, float)) and -50.0 <= raw <= 300.0

        def _build_message(self, raw: float) -> dict:
            return {
                "sensor_id": "PT-01",
                "value": raw,
                "unit": "°C",
            }

    # Create feeder (no module entity needed for this demo)
    entity = MagicMock()
    entity.name = "demo_module"
    entity.uuid = "0000-demo"

    feeder = TemperatureFeeder(
        feeder_name="TemperatureFeed",
        module_entity=entity,
    )

    # Before start() the feeder buffers messages
    logger.info("Feeder ready: %s", feeder.is_alive())

    await feeder.feed(87.3)     # valid — gets buffered
    await feeder.feed(999.9)    # invalid — dropped with warning
    await feeder.feed(22.0)     # valid — gets buffered

    logger.info(
        "Buffered %d message(s) before start()  error_count=%d",
        len(feeder.get_buffer()),
        feeder.error_count,
    )

    # Demonstrate metrics
    assert feeder.feed_count == 0  # not published yet (no publisher)
    assert len(feeder.get_buffer()) == 2  # two valid messages buffered

    logger.info("✅ CustomBaseFeeder demo complete.")


# ---------------------------------------------------------------------------
# Demonstrate FeederRegistry
# ---------------------------------------------------------------------------

def demo_registry():
    """Show FeederRegistry listing and lookup."""
    from vyra_base.com.feeder.registry import FeederRegistry, register_feeder
    from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder

    @register_feeder("PressureFeed")
    class PressureFeeder(CustomBaseFeeder):
        def _build_message(self, raw):
            return {"bar": raw}

    @register_feeder("VibrationFeed")
    class VibrationFeeder(CustomBaseFeeder):
        def _build_message(self, raw):
            return {"rms": raw}

    logger.info("Registered feeders: %s", FeederRegistry.list_feeders())

    klass = FeederRegistry.get("PressureFeed")
    logger.info("Got class from registry: %s", klass)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logger.info("=== VYRA Feeder Examples ===\n")

    logger.info("--- 1. Config Resolver ---")
    demo_config_resolver()

    logger.info("\n--- 2. Custom Feeder ---")
    asyncio.run(demo_custom_feeder())

    logger.info("\n--- 3. Feeder Registry ---")
    demo_registry()
