# Example 15 — Skills

This example covers the **Skill** feature of VYRA modules.  A Skill is a named
configuration set that maps logical skill-type requirements (defined in a
blueprint) to concrete module resources (parameters, volatiles, interfaces).

## What is a Skill?

A skill instance has:

| Field | Description |
|-------|-------------|
| `id` | Unique name within the module (e.g. `motion_slow`) |
| `skill_type` | Logical type from the blueprint (e.g. `motion_control`) |
| `is_enabled` | Whether the skill is active |
| `parameter_mapping` | `{skill_param_name: module_param_key}` |
| `volatile_mapping` | `{skill_volatile_name: module_volatile_key}` |
| `interface_mapping` | `{skill_interface_name: module_functionname}` |
| `local_defaults` | Instance-specific default value overrides |
| `displayname` | Optional human-readable label |
| `description` | Optional description |
| `tags` | Optional string array |

## Multiple Instances per Skill Type

Multiple skill instances can share the same `skill_type` with different
`local_defaults` to describe operational profiles without repeating mappings:

```
motion_slow   → motion_control  (max_speed=0.3 m/s, acceleration=0.5 m/s²)
motion_normal → motion_control  (max_speed=1.2 m/s, acceleration=1.5 m/s²)
motion_rough  → motion_control  (max_speed=2.5 m/s, acceleration=3.0 m/s²)
```

## Skill Zenoh Services

All modules automatically expose these services (defined in
`vyra_base/interfaces/config/vyra_skills.meta.json`):

| Service | Description |
|---------|-------------|
| `read_all_skills` | Returns all skill instances as JSON array |
| `get_skill` | Returns one skill by ID |
| `add_skill` | Creates a new skill instance |
| `update_skill` | Partially updates an existing skill |
| `delete_skill` | Deletes a skill by ID |

## Example: Adding a Skill via Zenoh

```python
import json
import asyncio
from zenoh import Config, Session

MODULE_KEY = "v2_mymodule_abc123"

async def main() -> None:
    session = await Session.open(Config())

    # Add a motion_control skill instance
    payload = {
        "id": "motion_slow",
        "skill_type": "motion_control",
        "parameter_mapping": {
            "max_speed": "drive.max_speed_mps",
            "acceleration": "drive.acceleration_ms2"
        },
        "volatile_mapping": {
            "current_position": "robot.current_position"
        },
        "interface_mapping": {
            "move_to": "drive_move_to_position",
            "stop": "drive_emergency_stop"
        },
        "local_defaults": {
            "drive.max_speed_mps": 0.3,
            "drive.acceleration_ms2": 0.5
        },
        "displayname": "Motion — Slow",
        "description": "Safe low-speed motion for confined spaces.",
        "tags": json.dumps(["motion", "safe"])
    }

    reply = await session.get(
        f"{MODULE_KEY}/add_skill",
        payload=json.dumps(payload).encode()
    )
    result = json.loads(reply.ok.payload.deserialize(str))
    print("add_skill:", result)  # {"success": true, "message": "..."}

    # Read back all skills
    reply = await session.get(f"{MODULE_KEY}/read_all_skills", payload=b"{}")
    skills = json.loads(json.loads(reply.ok.payload.deserialize(str))["all_skills_json"])
    print("skills:", json.dumps(skills, indent=2))

    await session.close()

asyncio.run(main())
```

## Example: Using `SkillManager` inside a Module

```python
from vyra_base.core.skill_manager import SkillManager
from vyra_base.storage.db_access import DbAccess

# In your module's _base_.py or setup_storage():
skill_manager = SkillManager(db_access=entity.database_access)

# Create a skill instance
await skill_manager.add_skill(
    id="motion_slow",
    skill_type="motion_control",
    parameter_mapping={"max_speed": "drive.max_speed_mps"},
    volatile_mapping={},
    interface_mapping={"move_to": "drive_move_to_position"},
    local_defaults={"drive.max_speed_mps": 0.3},
    displayname="Motion — Slow",
)

# List all skills
skills = await skill_manager.list_skills()
for s in skills:
    print(s.id, s.skill_type, s.is_enabled)

# Get skills by type (used by blueprint verifier)
motion_skills = await skill_manager.get_skills_by_type("motion_control")
```

## Example: skill_instance.meta.json (optional module-side config)

Modules can optionally ship a JSON file that pre-seeds skill instances when
the module is first started. Format: `src/{module}_interfaces/config/skill_instances.json`

```json
[
    {
        "id": "motion_slow",
        "skill_type": "motion_control",
        "parameter_mapping": {
            "max_speed": "drive.max_speed_mps"
        },
        "volatile_mapping": {},
        "interface_mapping": {
            "move_to": "drive_move_to_position"
        },
        "local_defaults": {
            "drive.max_speed_mps": 0.3
        },
        "displayname": "Motion — Slow",
        "tags": ["motion", "safe"]
    }
]
```

## Blueprint Verification Flow

When the Module Manager starts, it:

1. Reads `module_data.yaml` for each registered module (e.g. `blueprints: [robot]`)
2. Fetches the blueprint definition from `vyra_storage_pool`
   (`GET /api/v1/blueprints/robot`)
3. For each `required_skills` entry (e.g. `motion_control`):
   - Calls `read_all_skills` on the module
   - Checks that at least one skill with `skill_type == "motion_control"` exists
     and is enabled
   - Validates that the skill's `interface_mapping` covers all
     `required_interfaces` from the skill type definition
   - Validates that `parameter_mapping` covers all `required_parameters`
   - Validates that `volatile_mapping` covers all `required_volatiles`
4. Writes `blueprint_verified: true/false` to the `registered_module` DB row
5. The browser module list shows a **Blueprint Verified ✓** badge

## See Also

- [`vyra_base/interfaces/config/vyra_skills.meta.json`](../../src/vyra_base/interfaces/config/vyra_skills.meta.json)
  — Zenoh service definitions
- [`vyra_base/storage/tb_skill.py`](../../src/vyra_base/storage/tb_skill.py)
  — SQLAlchemy model (to be created in Phase 1)
- [`vyra_base/core/skill_manager.py`](../../src/vyra_base/core/skill_manager.py)
  — Business logic layer (to be created in Phase 1)
- [`14_interfaces_config/README.md`](../14_interfaces_config/README.md)
  — How to write `*.meta.json` files
