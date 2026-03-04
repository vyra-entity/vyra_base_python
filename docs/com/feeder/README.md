# vyra_base.com.feeder

Feeder liefern Status-, News- und Error-Daten über das konfigurierte
Transportprotokoll (z. B. Zenoh, ROS2, Redis) und unterstützen Monitoring,
Condition Rules, Execution Points und Debouncing.

## Canonical Documentation

- Technische Hauptdoku (maintained at source):
	`src/vyra_base/com/feeder/README.md`
- Sphinx API Pages:
	- `docs/com/feeder/feeder.rst`
	- `docs/com/feeder/state_feeder.rst`
	- `docs/com/feeder/news_feeder.rst`
	- `docs/com/feeder/error_feeder.rst`

## Runnable Examples

- `examples/06_feeder/`
- `vyra_module_template/examples/feeders/` (template-oriented feeder snippets)

## Notes

- Prefer the source-level feeder README for implementation details.
- Prefer `examples/` for executable usage over inline snippets in docs.
