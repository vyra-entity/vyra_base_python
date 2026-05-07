# Example 09 - parameter_volatile

This folder documents practical usage patterns for:

- `vyra_base.core.parameter.Parameter`
- `vyra_base.core.volatile.Volatile`

## Files

- `parameter_volatile_patterns.py`: quick pattern walkthrough for startup and runtime calls.

## Run

```bash
python examples/09_parameter_volatile/parameter_volatile_patterns.py
```

## Parameter usage in modules

Typical flow:

1. Create `Parameter` in entity initialization.
2. Load default values with `await parameter.load_defaults(...)`.
3. Read values using `await parameter.get_parameter_impl(key)` for internal logic.
4. Expose runtime updates through the `set_parameter` service.

## Volatile usage in modules

Typical flow:

1. Create `Volatile` with transient Redis access.
2. Write transient values via `await volatile.set_volatile_value(key, value)`.
3. Read values via `await volatile.get_volatile_value(key)`.
4. Optionally expose updates with `await volatile.publish_volatile_to_ros2(key)`.

## Rule of thumb

- Use **Parameter** for persisted, user-configurable values.
- Use **Volatile** for transient runtime state that is not persisted.
