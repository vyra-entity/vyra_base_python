# Deprecated Documentation Files

This directory contains documentation files that have been deprecated, 
consolidated, or relocated as part of the documentation restructuring.

## State Machine (Consolidated into components/state-machine/)

- **STATE_MACHINE.md** — Outdated operational states, superseded by state-machine_FULL.rst
- **3_Layer_Statemachine.md** — Mixed German/English, concepts partially outdated
- **operational_state_machine.md** — Overlaps with FULL version, content merged
- **States.md** — Duplicates states.rst
- **Events.md** — Duplicates events.rst  
- **Transitions.md** — Duplicates transitions.rst

## Communication (German README cleanup)

Deleted minimal German README.md stubs (3 lines each):
- docs/com/core/README.md
- docs/com/transport/README.md
- docs/com/external/README.md
- docs/com/industrial/README.md
- docs/com/industrial/modbus/README.md
- docs/com/industrial/opcua/README.md
- docs/com/transport/redis/README.md
- docs/com/transport/ros2/README.md
- docs/com/transport/uds/README.md

## Storage (Moved to components/storage/)

- **redis_connection.md** — Moved to vyra_base.com.transport.t_redis, imports broken
- **api_reference.md** — Incomplete, duplicates database_operations.md

## Root-Level .md Files (Migrated to guides/)

- **LOGGING_README.md** — Moved to guides/logging-setup.md
- **QUICK_START_INTERFACE_DESCRIPTORS.md** — Moved to guides/interfaces-how-to.md
- **README_i18n.md** — Deprecated, i18n not maintained

## Migration Timeline

- **Date Started**: 2026-05-05
- **Status**: In Progress
- **Phase 1 (Analysis)**: ✅ Complete
- **Phase 2 (Consolidation)**: 🔄 In Progress
- **Phase 3 (Validation)**: ⏳ Pending
- **Phase 4 (Archive)**: ⏳ Pending

## Recovery

If you need to restore a deprecated file, it may be in git history:

```bash
git log --follow -- docs/state/STATE_MACHINE.md
git show <commit>:docs/state/STATE_MACHINE.md
```

---

**Note**: Deprecated files are preserved in version control. New documentation
follows the structure in `docs/components/`, `docs/guides/`, and `docs/api-reference/`.
