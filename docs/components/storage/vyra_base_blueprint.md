# VYRA Base Blueprint

## Purpose

The VYRA Base Blueprint defines the minimum contract for foundational framework modules.
It standardizes lifecycle, health, and event interfaces so orchestration and monitoring
can rely on a consistent baseline.

## Blueprint ID

- blueprint_type: VYRA_BASE
- source file: vyra_storage_pool/data/blueprints/vyra_base.yaml

## Required Skills

- lifecycle_management
- health_reporting
- event_publishing

## Optional Skills

- rest_api
- frontend_ui

## Required Interfaces

- /module/initialize (service)
- /module/stop (service)
- /module/state (topic)
- /module/health (topic)

## How It Is Used

1. A module declares VYRA_BASE in module_data blueprints.
2. ModuleManager BlueprintVerifier fetches the blueprint from repository storage.
3. SkillVerifier checks required skill presence and element mappings.
4. Verification result is persisted in the registered module table.

## Migration Guidance

- Existing modules using BASIC can keep that blueprint.
- Core orchestration modules should migrate to VYRA_BASE.
- If both are needed, declare both blueprints and satisfy the union of required skills.
