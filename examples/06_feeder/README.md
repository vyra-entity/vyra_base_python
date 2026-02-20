# Example 06 — VYRA Feeders

Demonstrates the feeder subsystem: protocol resolution, custom feeders, and the feeder registry.

## Files

| File | Description |
|------|-------------|
| `01_basic_feeder.py` | `FeederConfigResolver`, metrics, and `FeederRegistry` walkthrough |
| `02_custom_feeder.py` | Full `CustomBaseFeeder` subclass with `@register_feeder` |

## Quick Start

```bash
cd vyra_base_python
python examples/06_feeder/01_basic_feeder.py
python examples/06_feeder/02_custom_feeder.py
```

## Key Concepts

### Protocol Resolution

Each feeder reads its own transport protocol from the module's interface config JSON by matching `functionname`:

```json
{
  "type": "publisher",
  "functionname": "TemperatureFeed",
  "tags": ["zenoh"],
  "filetype": ["VBASETemperatureFeed.proto"]
}
```

If the name is not found, `FeederConfigResolver` logs an error with fuzzy-matched suggestions
(powered by `difflib.get_close_matches`).

### Custom Feeder

```python
from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
from vyra_base.com.feeder.registry import register_feeder

@register_feeder("TemperatureFeed")
class TemperatureFeeder(CustomBaseFeeder):
    def _validate(self, raw: float) -> bool:
        return -50 <= raw <= 300

    def _build_message(self, raw: float) -> dict:
        return {"value": raw, "unit": "°C"}
```

### Feeder Registry

```python
from vyra_base.com.feeder.registry import FeederRegistry

klass = FeederRegistry.get("TemperatureFeed")
print(FeederRegistry.list_feeders())
```

## Protocol Support

| Tag in config | Protocol used |
|---------------|---------------|
| `zenoh`       | Zenoh CAL     |
| `ros2`        | ROS2 topics   |
| `redis`       | Redis pub/sub |
| `uds`         | Unix Domain Socket |
