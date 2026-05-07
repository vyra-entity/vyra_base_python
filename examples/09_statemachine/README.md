# Example 10 - statemachine

This folder is the extended state-machine example set.

## Files

- `unified_state_callbacks.py`
	- callback registration (`on_any_change`, `on_lifecycle_change`)
	- transition diagnostics and history
- `unified_state_machine_example.py`
	- full three-layer lifecycle walk-through
- `operational_state_machine_example.py`
	- operational metaclass and lifecycle method patterns

## Run

```bash
python examples/10_statemachine/unified_state_callbacks.py
python examples/10_statemachine/unified_state_machine_example.py
python examples/10_statemachine/operational_state_machine_example.py
```
