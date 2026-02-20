"""
Example 06-02: Custom Feeder with @register_feeder & full lifecycle

Shows how to build a production-ready custom feeder:
- Validation
- Message transformation
- Registry registration
- Logging integration
"""
import asyncio
import logging
from typing import Any

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Custom feeder definition
# ---------------------------------------------------------------------------

from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
from vyra_base.com.feeder.registry import FeederRegistry, register_feeder


@register_feeder("PressureFeed")
class PressureFeeder(CustomBaseFeeder):
    """Feeder for hydraulic pressure readings (0–400 bar).

    Interface config entry (in your module's interfaces/config/*.json)::

        {
            "type": "publisher",
            "functionname": "PressureFeed",
            "name": "pressure_feed",
            "tags": ["zenoh"],
            "filetype": ["VBASEPressureFeed.proto"]
        }
    """

    MAX_PRESSURE = 400.0
    MIN_PRESSURE = 0.0

    def _validate(self, raw: Any) -> bool:
        if not isinstance(raw, (int, float)):
            logger.warning("PressureFeeder: expected numeric raw value, got %s", type(raw))
            return False
        if not (self.MIN_PRESSURE <= raw <= self.MAX_PRESSURE):
            logger.warning(
                "PressureFeeder: value %.1f bar out of range [%.0f, %.0f]",
                raw, self.MIN_PRESSURE, self.MAX_PRESSURE,
            )
            return False
        return True

    def _build_message(self, raw: float) -> dict:
        return {
            "sensor": "HYD-P01",
            "value": round(raw, 2),
            "unit": "bar",
            "alarm": raw > 350.0,
        }


# ---------------------------------------------------------------------------
# Simulate usage
# ---------------------------------------------------------------------------

async def main():
    from unittest.mock import MagicMock

    logger.info("=== Custom Feeder Example ===\n")

    # Show registry
    logger.info("Registered feeders: %s", FeederRegistry.list_feeders())

    # Create feeder instance
    entity = MagicMock()
    entity.name = "hydraulics_module"
    entity.uuid = "hydraulics-0001"

    feeder = PressureFeeder(
        feeder_name="PressureFeed",
        module_entity=entity,
    )

    # Feed values before start() — they get buffered
    test_values = [120.5, -10.0, 380.0, "invalid", 200.0, 450.0]
    for v in test_values:
        feeder.feed(v)

    logger.info(
        "After feeding %d raw values: buffer_size=%d  error_count=%d",
        len(test_values),
        len(feeder.get_buffer()),
        feeder.error_count,
    )

    # Expected buffer: 120.5, 380.0, 200.0 → 3 valid messages
    assert len(feeder.get_buffer()) == 3, f"Expected 3 buffered, got {len(feeder.get_buffer())}"
    logger.info("✅ Validation correctly filtered %d invalid values.", feeder.error_count)

    # Show transformed messages
    for i, msg in enumerate(feeder.get_buffer()):
        logger.info("  Buffered message %d: %s", i + 1, msg)


if __name__ == "__main__":
    asyncio.run(main())
