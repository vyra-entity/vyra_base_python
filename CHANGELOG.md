# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added (2026-03-05)
- **Docs: VyraLogHandler**: Neue Dokumentation `docs/com/handler/LOGGER_HANDLER.md` beschreibt Klasse, `get_recent()`-API, Zenoh-Service-Integration und SSE-Polling-Pattern.
- **README: VyraLogHandler**: Abschnitt `#### 4. Log History` und Link in `Core Documentation` ergänzt.
- `vyra_base/com/handler/logger.py`: Neues Modul mit öffentlicher `VyraLogHandler`-Klasse (in-memory Ring-Buffer für Log-Records, `deque(maxlen=1000)`).
- `VyraLogHandler` wird von `com/handler/__init__.py` exportiert.
- `VyraEntity.get_log_history`: Neuer `@remote_service` der bis zu 1000 Log-Einträge als `logs_json` (JSON-Array) zurückgibt. Für den Dashboard-Logs-Tab.

### Changed (2026-03-05)
- `_VyraLogHandler` aus `core/entity.py` in `com/handler/logger.py` verschoben und als `VyraLogHandler` öffentlich gemacht.
- `entity.py` importiert nun `VyraLogHandler` aus `vyra_base.com.handler.logger`.


## [0.1.8+build.108] - 2026-03-04

### Build

update typechecker in parameter.py


## [0.1.8+build.107] - 2026-03-04

### Build

update interface config naming structure everywhere to vyra_<domain>.meta.json


## [0.1.8+build.106] - 2026-03-04

### Build

add new build containing the underlaying changelog

### Added (2026-03-04)
- `core/parameter.py`: neuer Remote-Service `create_new_parameter` mit strikt-neuer Semantik (Fehler bei existierendem Key).
- `core/volatile.py`: impl-basierte Basisfunktionen ergänzt (`get_volatile_impl`, `set_volatile_impl`, `read_all_volatiles_impl`, `create_new_volatile_impl`) sowie Remote-Service `create_new_volatile`.
- `tests/com/core/test_parameter_volatile_extensions.py`: neue Unit-Tests für striktes Parameter/Create-Verhalten und Volatile-Adressschema.

### Changed (2026-03-04)
- Volatile-Adressierung vereinheitlicht auf `<module_name>_<module_id>/volatile/<volatile_name>` inkl. Prefix-Normalisierung und Rückgabe der vollen `address` in `read_all_volatiles`.
- `core/entity.py`: Übergibt `module_name` an `Volatile` für deterministische Key-Namensbildung.
- `interfaces/config/vyra_core.meta.json`: um `create_new_parameter` und `create_new_volatile` erweitert.
- `interfaces/tools/generate_interfaces.py`: Generierungswarnung, wenn bei `access_level > 1` der Parameter `auth_token` in `params` fehlt.


## [0.1.8+build.104] - 2026-03-04

### Build

Unify interface config file naming to vyra_<domain>.meta.json

### Fixed (2026-03-04)
- `FunctionConfigBaseTypes` um den kompatiblen Enum-Wert `publisher` ergänzt, damit bestehende Aufrufer nicht mit `AttributeError` brechen.
- Zielgerichtete Validierung der betroffenen Defaults-Tests erfolgreich (`pytest -q --no-cov tests/test_defaults.py`).

### Changed (2026-03-04)
- Beispiele neu strukturiert und erweitert:
  - neue Root-Beispiele unter `examples/11_defaults_entries/` und `examples/12_security_levels/`
  - bestehende Beispielgruppen (`08/09/10`) als primäre Referenz konsolidiert.
- Doku-Policy geschärft: ausführbare Beispiele liegen unter `examples/`, `docs/` referenziert nur noch darauf.

### Documentation (2026-03-04)
- `docs/security/examples.rst` auf indexbasiertes Format mit Verweisen auf lauffähige Beispiele umgestellt.
- `docs/com/feeder/README.md` und `docs/STRUCTURE.md` inhaltlich auf aktuelle Struktur und Beispiel-Workflow angehoben.


## [0.1.8+build.102] - 2026-03-03

### Build

bugfixing

### Documentation

- Feeder-Dokumentation in `src/vyra_base/com/feeder/README.md` erweitert um:
  - `monitor(...)` Wrapper für Error/News/Custom,
  - Condition-Rules inkl. `execution_point` (`BEFORE|DURING|AFTER|ALWAYS`),
  - Rule-/Tag-Filter (`rule_names`, `tags`), Runtime-Context-Felder und Dispatch-Verhalten,
  - Debouncing-Hinweise.
- `examples/06_feeder/README.md` um Monitoring-/Condition-Workflow ergänzt.


## [0.1.8+build.101] - 2026-03-03

### Build

bugfixing feeder


## [0.1.8+build.100] - 2026-03-03

### Build

Adding feeder decorator that could be used to mark functions if they fullfill a specific condfition function they will be fired too. So for example a Newsfeed could be released after a function call if a custom condition within this function was true. ErrorFeed decorator always fire if a exception in this function occures

### Added
- Feeder-Tracking-Grundlagen in `com/feeder`:
  - Debouncing für identische Feed-Nachrichten (5s Fenster, Duplicate-Counter)
  - Condition-Registry (sync bool-only) mit optionalen Success/Failure-Messages und Tag-Unterstützung
  - Optionale Filterung bei Condition-Auswertung nach Rule-Namen und Tags
  - `execution_point` pro Condition: `BEFORE`, `DURING`, `AFTER`, `ALWAYS`
  - Lazy-Entity-Lookup-Helfer für Decorator-basierte Feed-Anbindung
- `ErrorFeeder` Monitor-Decorator (`feed_tracker.monitor(...)`) für Exception-Überwachung mit Traceback-Analyse und automatischem ErrorFeed-Push.
- `NewsFeeder` Condition-Tracker API (`register_news_condition`, `evaluate_tracked_conditions`).
- Generalisierte Monitor-Unterstützung für `NewsFeeder` und `CustomBaseFeeder` (condition-getriebene Dispatches ohne Exception-Pflicht).

### Changed
- `StateType` um `ANY` erweitert und Callback-Subscription in `state_machine.py` auf robuste Layer-Normalisierung (`StateType` oder `str`) umgestellt.
- `UnifiedStateMachine.on_any_change()` nutzt nun `StateType.ANY`.
- `FeedTracker` aus `error_feeder.py` in gemeinsames `com/feeder/tracking.py` verschoben; Export über `vyra_base.com.feeder` vereinheitlicht.

### Tests
- Neue Tests für Callback-Enum-Subscription, Debounce, Condition-Validierung und `feed_tracker` Lazy-Entity-Lookup.


## [0.1.8+build.98] - 2026-03-02

### Build

update db manipulater loghandling


## [0.1.8+build.97] - 2026-03-02

### Build

bugfixing statemachine

## [2026-03-02]
### Added: Lifecycle Suspend/Resume API & fehlende UnifiedStateMachine-Delegierungen
- **`lifecycle_layer.py`**:
  - `is_suspended()` — State-Query für `SUSPENDED`
  - `enter_suspend(reason)` — Übergang `Active → Suspended` via `SET_SUSPENDED` Event
  - `resume_from_suspend(info)` — Übergang `Suspended → Active` via `RESUME_SUSPENDED` Event
- **`unified.py`**:
  - `suspend(reason)` / `resume_from_suspend(info)` — delegieren an LifecycleLayer
  - `enter_recovery()`, `complete_recovery()`, `fail_recovery()` — vorher fehlende Delegierungen
  - `is_suspended()`, `is_initializing()`, `is_active()`, `is_recovering()`, `is_shutting_down()`, `is_offline()` — Lifecycle-State-Shortcuts
### Fixed
- **`lifecycle_layer.py`**: `fail_recovery()` Docstring korrigiert: „Recovering → Deactivated" → „Recovering → ShuttingDown"
### Docs
- **`docs/state/Transitions.md`**: SUSPENDED-Transitions ergänzt (`Active↔Suspended`, `Suspended→Offline`); fehlender `Offline → Initializing` Eintrag hinzugefügt
- **`docs/state/Events.md`**: `SET_SUSPENDED` und `RESUME_SUSPENDED` in Lifecycle-Events-Tabelle ergänzt; UnifiedStateMachine-Beispiel um Suspend/Resume und Recovery erweitert


## [0.1.8+build.96] - 2026-03-02

### Build

Update interfaces and add combined proto generation. This will allow to have function definitions from different fields in one proto file. For example you can habe set_parameter and get_parameter and sevela other functions definitions in VBASEParameter.proto


## [0.1.8+build.95] - 2026-03-02

### Build

Update Feeder converting error bug


## [0.1.8+build.94] - 2026-03-02

### Build


### Fix: AttributeError in NewsFeeder.feed_sync (vyra_base_python)
- `NewsFeeder.feed_sync()` delegierte raw strings direkt an `BaseFeeder` → `_prepare_entry_for_publish` versuchte `str.level` → `AttributeError`
- Fix: `feed_sync` normalisiert `str`/`list` via `build_newsfeed()` zu `NewsEntry` bevor es an `super().feed_sync()` weitergegeben wird
- Datei: `vyra_base_python/src/vyra_base/com/feeder/news_feeder.py`



## [0.1.8+build.93] - 2026-02-27

### Build

Change log output order (levelname) before (name)


## [0.1.8+build.91] - 2026-02-26

### Build

Bugfix: Error in generate_interfacess.py regarding interface folders that throw an error if they already exists


## [0.1.8+build.90] - 2026-02-26

### Build

Update transport structure and interface structure


## [0.1.8+build.89] - 2026-02-25

### Build

Bugfixing feeder


## [0.1.8+build.84] - 2026-02-25

### Build

bugfixing com/converter


## [0.1.8+build.83] - 2026-02-25

### Build

Update extract_interfaces to copy base interface files to be used in othnterface files for complex structures


## [0.1.8+build.82] - 2026-02-25

### Build

update feeder structure to use interface files


## [0.1.8+build.80] - 2026-02-24

### Build

bugfixing


## [0.1.8+build.79] - 2026-02-24

### Build

bugfixing and cleaning


## [0.1.8+build.78] - 2026-02-24

### Build

bugfix entity (FunctionConfigBaseTypes publisher->message)


## [0.1.8+build.77] - 2026-02-24

### Build

Bugfix interfacestypes


## [0.1.8+build.76] - 2026-02-24

### Build

Update bugs containing interface_type (publisher,server,actionServer) -> (message,service, action)


## [0.1.8+build.75] - 2026-02-23

### Build

bugfix interface: "server" -> "service"


## [0.1.8+build.74] - 2026-02-23

### Build

Interfaces are now only copying the config files not the interfacesfiles directly. They will later be generated by the configuration


## [0.1.8+build.66] - 2026-02-20

### Build

Refactored feeder module to improve extensibility and align with the new abstract feeder structure (no functional behavior changes).


## [0.1.8+build.65] - 2026-02-20

### Build

Update feeder structure (make abstract, more usable, examples, tests, docs) and add tcp and udp external communication interfaces 


## [0.1.8+build.62] - 2026-02-18

### Build

update generate_interfaces


## [0.1.8+build.61] - 2026-02-18

### Build

bugfix generate_interfaces.py


## [0.1.8+build.60] - 2026-02-18

### Build

Bugfix extract_interfaces


## [0.1.8+build.59] - 2026-02-18

### Build

update .proto interfaces available in msg srv and action folder


## [0.1.8+build.58] - 2026-02-18

### Build

massive refactoring, cleaning, test and docs update


## [0.1.8+build.54] - 2026-02-18

### Build

Adding blueprint and update decorators
Change interface options from speaker, callable, job to publisher, server and actionServer


## [0.1.8+build.51] - 2026-02-16

### Build

Update logger ,docs, and transport structure to use publisher, service and action in the future instead ob speaker, callable and job


## [0.1.8+build.50] - 2026-02-16

### Build

Remove static dead Logger and adding python base logger


## [0.1.8+build.48] - 2026-02-12

### Build

Update vyra transport layer to work with dynamic loading interface types


## [0.1.8+build.45] - 2026-02-11

### Build

make logger autark and update testings


## [0.1.8+build.42] - 2026-02-09

### Build

Update FunctionConfigEntry ros2type->interfacetype to be not ros2 dependent but type agnostic


## [0.1.8+build.41] - 2026-02-09

### Build

Bugfixing: Wrong types in feeder.py


## [0.1.8+build.40] - 2026-02-09

### Build

Update types metadata structure:[bugfix]


## [0.1.8+build.39] - 2026-02-09

### Build

Update transport structure in com->transport and bugfixing in development process


## [0.1.8+build.38] - 2026-02-09

### Build

update speaker, callable, job structure to all transport provider


## [0.1.8+build.37] - 2026-02-09

### Build

Bugfixing topic_builder


## [0.1.8+build.36] - 2026-02-09

### Build

adding topic_builder to generate a generic naming convention for all transport protocols


## [0.1.8+build.35] - 2026-02-06

### Build

Simple rebuild


## [0.1.8+build.34] - 2026-02-06

### Build

Update auto generated .proto file script to add the header and update namings (add VBASE...)


## [0.1.8+build.33] - 2026-02-06

### Build

 - Add zenoh as transport option with protobuf support 
 - Refactor transport module structure to support multiple implementations (e.g., ROS2, Redis, UDS, Zenoh) under a unified interface 
 - Update documentation and examples to reflect new transport options and structure 
 - Add tests for new Zenoh transport implementation and converters - Ensure backward compatibility for existing transports while introducing new ones 
 - Update README files and documentation to guide users on how to use the new transport options and converters


## [0.1.8+build.32] - 2026-02-05

### Build

Updating interfaces to not depend on ros2 but on vyra interfaces. Therefor we can depict not only ros2 as interface but all transport protocols defined in com/transport


## [0.1.8+build.31] - 2026-02-05

### Build

Bugfixing: redis->callable->self.redis_client



## [0.1.0] - 2025-06-11

First draft

<!-- ### Added
Added \_\_init__ to all folders -->

