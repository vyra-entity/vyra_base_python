# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]


## [0.1.8+build.143] - 2026-04-02

### Build

bugfix


## [0.1.8+build.140] - 2026-04-01

### Build

adding bcrypt

### Added — JWT and bcrypt support in crypto_helper

- **`crypto_helper.py`**: Added `create_jwt_token()`, `verify_jwt_token()`, `decode_jwt_token_unverified()` for RS256-signed JWT creation and verification using PyJWT
- **`crypto_helper.py`**: Added `hash_password_bcrypt()` and `verify_password_bcrypt()` for bcrypt password hashing
- **`crypto_helper.py`**: Added `_ensure_jwt_crypto()` internal helper to work around redis-py importing jwt before cryptography is fully available
- **`pyproject.toml`**: Added `PyJWT[crypto]` and `bcrypt` dependencies
- **`security/__init__.py`**: Refactored to lazy `__getattr__` imports to break circular import chain (`crypto_helper` → `security_levels` → `security/__init__` → `security_manager`)
- **`tests/security/conftest.py`**: Added fixture that restores real `cryptography` and `jwt` modules (mocked globally for ROS2 tests)
- **`tests/security/test_crypto_jwt.py`**: Added 9 unit tests for JWT token lifecycle and bcrypt password hashing


## [0.1.8+build.138] - 2026-03-31

### Build

see fixes below

### Fixed

- **ROS2 provider**: Skip non-ROS2 types (e.g. `dict`) in `create_publisher` / `create_subscriber` — fall through to `topic_builder.load_interface_type()` instead of failing with `'dict' has no attribute '_TYPE_SUPPORT'`
- **ROS2 provider**: Pop `topic_builder` from `**kwargs` in `create_publisher`, `create_subscriber`, `create_server`, `create_action_server` to prevent duplicate keyword argument error when factory injects `topic_builder` via kwargs
- **Redis client**: Iterate over copy of `_active_channels` in `close()` to prevent `RuntimeError: Set changed size during iteration`
- **ROS2 provider**: Added `_resolve_ros2_type()` to `create_publisher`, `create_subscriber`, `create_action_server`, `create_action_client` — extracts ROS2 class from mixed `interfacetypes` lists (e.g. `["MessageTest.proto", <class MessageTest>]`) preventing `'list' object has no attribute '_TYPE_SUPPORT'`
- **UDS publisher**: Implemented `initialize()` and `publish()` — datagram broadcast using SOCK_DGRAM with `.topic` meta file based subscriber discovery
- **UDS subscriber**: Added `.topic` meta file writing in `initialize()` and cleanup in `cleanup()` to enable publisher discovery
- **UDS action_client**: Fixed socket path mismatch — removed extra `module_name_` prefix to match action_server path (`{safe_name}.sock`)
- **Redis client**: Fixed SSL `APPLICATION_DATA_AFTER_CLOSE_NOTIFY` in `close()` — cancel pubsub listener task before closing connection


## [0.1.8+build.137] - 2026-03-31

### Build

bugfix <com/transport>/service. see below

### Fixed

- **Redis client**: Added missing `context` parameter to `_handle_response()` — pubsub loop calls `handler(message, context)` but method only accepted one arg, causing response future to never resolve
- **Redis server**: Replaced `_ResponseHolder` with `_AttrDict` to support attribute access on request dicts (`request.t1`); capture and use callback return value instead of empty response holder
- **UDS server/client**: Sanitize `/` in topic names with `_` for filesystem-safe socket paths (matching existing action_server/action_client pattern) — fixes `[Errno 2] No such file or directory`
- **UDS server**: Added `(request, response)` ROS2-style callback support with `_AttrDict` wrapper and `inspect.signature` detection, matching Redis server pattern


## [0.1.8+build.136] - 2026-03-31

### Build

bugfix interface schemas


## [0.1.8+build.135] - 2026-03-31

### Build

bugfix transport see below


## [0.1.8+build.134] - 2026-03-30

### Build

bugfixing

### Fixed

- Redis subscriber callback signature mismatch: `_on_message()` was called with 2 args (message, context) but only accepted 1. RedisSubscriberImpl now wraps the user callback to extract and parse the data payload.
- UDS transport `AF_UNIX path too long`: socket filenames now use a SHA-256 hash to stay within the 108-char limit for both publisher and subscriber.


## [0.1.8+build.133] - 2026-03-30

### Build

bugfixing transport provider


## [0.1.8+build.132] - 2026-03-30

### Build

bugfixing see below

### Fixed — InterfaceGenerator missing `map_type` method (2026-03-30)

- **`src/vyra_base/interfaces/tools/generate_interfaces.py`**: Restored the missing `def map_type(self, datatype: str) -> Tuple[str, bool]:` method definition. The method body was present in the file but the `def` line had been accidentally deleted, leaving the implementation as an orphaned docstring/code block. This caused `'InterfaceGenerator' object has no attribute 'map_type'` errors during `colcon` builds in Docker (e.g. `get_state`, `get_lifecycle_state`, etc.).


## [0.1.8+build.131] - 2026-03-30

### Build

extending protocols see below

### Fixed
- `com/transport/t_redis/provider.py`: Added lazy initialization via `_ensure_initialized()` — Redis provider now reconnects on first use if initial startup connection failed
- `com/transport/t_redis/provider.py`: Added `_require_client()` helper for Pyright type narrowing; replaced `disconnect()` (non-existent) with `close()`; fixed `ActionBlueprint` cast errors in `create_action_server`
- `com/transport/t_uds/provider.py`: Added lazy initialization via `_ensure_initialized()` — UDS provider now initializes on first use
- `com/transport/t_uds/provider.py`: `service_type` and `action_type` guards removed — dict-based communication works without schema type
- `com/transport/t_uds/vyra_models/client.py`: `service_type` parameter made `Optional[type] = None`
- `com/transport/t_uds/vyra_models/action_client.py`: `action_type` parameter made `Optional[type] = None`
- `core/entity.py`: UDS provider now correctly initialized with `check_availability()` + `initialize()` during `_register_transport_provider()`


## [0.1.8+build.130] - 2026-03-30

### Build

bugfixing see below

### Added
- `tests/plugin/test_plugin_runtime.py` — 24 unit tests for `StubRuntime`, `create_plugin_runtime()` factory, and `PluginCallError` (no wasmtime required)
- `tests/security/test_security_levels.py` — 29 unit tests for `SecurityLevel` enum values, `is_valid()`, `get_name()`, ordering, `requires_hmac()`, and `requires_certificate()`
- `tests/plugin/__init__.py` and `tests/security/__init__.py` — package markers for new test directories
- `cryptography` submodule mocks in `tests/conftest.py` to prevent pyo3 Rust-extension panic under Python 3.11
- `pytest.ini` now includes `vyra_base.plugin` and `vyra_base.security` in coverage collection

### Fixed
- `security/security_client.py`: replaced fragile `importlib.util.find_spec("rclpy")` check with `try/except ImportError` to avoid `ValueError` when rclpy is mocked in sys.modules

### Changed
- `tests/README.md` rewritten to reflect actual test directory structure


## [0.1.8+build.129] - 2026-03-27

### Build

rebuild see below


## [0.1.8+build.128] - 2026-03-27

### Build

see below

### Changed — decouple ROS2 from core transport layer (2026-03-27)

- **`defaults/entries.py`**: Removed `_type` field from `ErrorEntry`, `NewsEntry`, `StateEntry`, `PullRequestEntry`. Message types are now resolved internally by the interface layer.
- **`core/entity.py`**: Removed all references to `_type` / `_error_entry_type`; feeder constructors no longer receive a `type` argument.
- **`com/feeder/state_feeder.py`**, **`news_feeder.py`**, **`error_feeder.py`**: Removed `type: Any` parameter from `__init__`; `_type` attribute dropped.
- **`com/feeder/feeder.py`**: `message_type` kwarg in `create_publisher` calls is now conditional — only passed when `self._type is not None`.
- **`core/parameter.py`**: `_init_publisher` now uses `ProtocolType.ZENOH` instead of `ProtocolType.ROS2`; `message_type` kwarg removed (Zenoh resolves type from the interface proto file).
- **`core/volatile.py`**: `REDIS_TYPE_MAP` uses `.get()` instead of `[]` so an empty `transient_base_types` dict is safe; `publish_volatile_to_ros2` now raises `RuntimeError` when `communication_node is None`; `VyraNode` import wrapped in `try/except ImportError`.

### Added — log history date filtering (2026-03-27)

- **`com/handler/logger.py`** `VyraLogHandler.get_recent()`: New optional parameter
  `since_ts: Optional[float]` (Unix timestamp in seconds). When provided, only entries
  whose `seq` value (millisecond epoch) corresponds to a time ≥ `since_ts` are returned.
- **`core/entity.py`** `get_log_history`:
  - New request field `since_date` (ISO date `YYYY-MM-DD` or datetime `YYYY-MM-DDTHH:MM:SS`).
    Date-only values cover the whole day from 00:00:00 local time. Malformed values are
    silently ignored (falls back to returning all entries).
  - `limit` max increased from 1 000 to 10 000.
  - Ring-buffer capacity (`VyraLogHandler`) increased from 1 000 to 10 000 entries.


## [0.1.8+build.127] - 2026-03-26

### Build

see below


## [0.1.8+build.126] - 2026-03-26

### Build

see fixes below

### Fixed — Action server namespace propagation from FunctionConfigEntry (2026-07-14)

- **`src/vyra_base/core/interface_builder.py`** — `create_action()`: all four protocol branches (ROS2, Zenoh, Redis, UDS) now pass `namespace=setting.namespace` to `InterfaceFactory.create_action_server()`. The `namespace` value flows through `**kwargs` → `provider.create_action_server(**kwargs)` → `VyraActionServerImpl(**kwargs)` → `VyraTransport.__init__()` → `self.namespace`. As a result `initialize()` calls `topic_builder.build(name, namespace=self.namespace)` which produces the correct key including the namespace segment (e.g. `v2_modulemanager_id/test/action_test`). Previously the server always omitted the namespace, causing a mismatch with clients that correctly forwarded namespace from the HTTP request.

### Fixed — Zenoh action server/client Zenoh 1.x API (2026-06-10)

- **`action_server.py`**: `_handle_goal_request_sync` / `_handle_cancel_request_sync` replaced `loop.run_until_complete()` (deadlock when called from Zenoh thread) with `asyncio.run_coroutine_threadsafe(..., self._main_loop).result(timeout=30)`. Loop captured in `initialize()` as `self._main_loop`. Fixed `query.value.payload` → `query.payload`.
- **`action_client.py`**: Fixed `value=` → `payload=` kwarg and added `bytes()` conversion around `reply.ok.payload`.


## [0.1.8+build.125] - 2026-03-25

### Build

see fixes below

### Fixed — Double JSON serialization causing backslash flooding in logs (2026-03-25)

- **`src/vyra_base/core/entity.py`**
  - `get_log_history()` now assigns `response.logs_json = recent` (list) instead of `json.dumps(recent)` (JSON string).
  - **Root cause**: The Zenoh transport serializer (`ZenohSerializer.serialize()`) was applying `json.dumps()` to the entire response object. When `logs_json` was already a JSON string, this created nested escaping (e.g., `"` → `\"` → `\\\"` → `\\\\\\\"` etc.), producing millions of backslashes after repeated calls.
  - **Impact**: Each time the modulemanager polled `get_log_history`, the backslash count doubled, eventually writing multi-megabyte lines to `core_stdout.log` and freezing the browser.
  - **Solution**: Return the raw list so the Zenoh serializer handles JSON conversion exactly once.
  - Updated docstring to clarify that `logs_json` is now a list, not a JSON string.

### Fixed — Log message truncation to prevent browser freezing (2026-03-25)

- **`src/vyra_base/com/handler/logger.py`**
  - Added `max_message_length` parameter (default 10000) to `VyraLogHandler.__init__()` to prevent extremely long log messages from causing memory overflow and browser freezing.
  - Modified `emit()` method to truncate log messages exceeding `max_message_length` with a `[TRUNCATED: X chars]` suffix.
  - Fixes issue where 2.7 MB single-line log entries with backslashes caused the modulemanager log tab to freeze.


## [0.1.8+build.124] - 2026-03-25

### Build

bugfixing see CHANGELOGS below

### Fixed — `_handle_query_sync` timeout and empty error message (2026-03-25)

- **`src/vyra_base/com/transport/t_zenoh/vyra_models/server.py`**
  - `future.result(timeout=10)` → `future.result(timeout=30)`: 10 s was too short for slow event loops under load, causing spurious TimeoutError every ~13 s.
  - `logger.error(f"... failed: {e}")` → `logger.error(f"... failed: {type(e).__name__}: {e!r}")`: `concurrent.futures.TimeoutError()` has an empty `str()` representation, making the error log unreadable. Now logs the exception class name and repr.


## [0.1.8+build.123] - 2026-03-24

### Build

update params


## [0.1.8+build.122] - 2026-03-24

### Build

update default


## [0.1.8+build.121] - 2026-03-24

### Build

update subscriber.py if no async loop is available


## [0.1.8+build.120] - 2026-03-24

### Fixed

- `interface_builder.py` – `create_action()` completely rewritten: added Zenoh/Redis/UDS fallback (matching `create_service` pattern); no longer raises `InterfaceError` when ROS2 fails.
- `interface_builder.py` – Added `has_ros2_type()` guard to `create_action`, `create_service`, and `create_publisher`: ROS2 is skipped silently when `interfacetypes` contains only strings (no actual ROS2 class), preventing spurious ERROR log entries for proto-only interfaces.
- `entity.py` – `create_action` now receives the `ProviderRegistry` instance to avoid unnecessary re-instantiation.
- `t_redis/provider.py` – Fixed `UnboundLocalError` in `create_action_server()`: the `isinstance(_bp, ActionBlueprint)` check was outside the block where `_bp` is assigned.
- `t_zenoh/vyra_models/subscriber.py` – `_callback_wrapper` now checks `loop.is_closed()` before calling `call_soon_threadsafe`/`run_coroutine_threadsafe`, preventing `RuntimeError: Event loop is closed` during module shutdown/restart.

## [0.1.8+build.120] - 2026-03-20

### Build

add logger to every subscription, call and actionCall of any transport option (ros2, zenoh, redis, uds)

## [Unreleased] - 2026-03-20

### Added

- `InterfaceFactory._debug_wrap()`: transport introspection wrapper that emits a `DEBUG` log entry to `core_stdout.log` whenever any inbound transport event fires.  Covered operations and log tags:
  - `subscribe` – subscriber callback invoked (any protocol: ros2, zenoh, redis, uds)
  - `call` – server `response_callback` invoked; client `call()` logs both outbound request and inbound response
  - `actionCall.goal` / `actionCall.cancel` / `actionCall.execute` – action-server lifecycle callbacks
  - `actionCall.response` / `actionCall.feedback` / `actionCall.result` – action-client callbacks
- Log format: `[TRANSPORT IN] type=<type>  name='<name>'  protocol=<proto> | args=<...>  kwargs=<...>`



## [0.1.8+build.119] - 2026-03-19

### Build

update vyra log handler


## [0.1.8+build.118] - 2026-03-19

### Build

Add get_log_history and refactoring


## [0.1.8+build.117] - 2026-03-17

### Build

Change log warning for replacing transport client to debug level


## [0.1.8+build.116] - 2026-03-17

### Build

just update


## [0.1.8+build.115] - 2026-03-13

### Build

see below

### Added
- **`storage/tb_error_log.py`**: Added `error_id` column (indexed `Text`, nullable) to the
  `ErrorLog` model so each ring-buffer entry carries the UUID of the originating `ErrorEntry`.
- **`com/feeder/error_feeder.py`**: `_prepare_entry_for_publish()` now includes `error_id`
  (the string representation of `ErrorEntry.uuid`) in the wire dict sent via Zenoh.
- **`com/handler/error_log_database.py`**: `_build_record()` maps `error_id` / fallback `uuid`
  to the `error_id` column; `default_error_log_fields()` declares `error_id` as an optional,
  nullable `str` field.
- **`interfaces/config/vyra_com.meta.json`**: `ErrorFeed` returns now include `error_id`
  (string UUID). New `acknowledge_error` Zenoh service entry with params `error_id` and
  optional `user`.
- **`core/entity.py`** — `VyraEntity.acknowledge_error()`: new `@remote_service()` endpoint
  that accepts an `error_id` and optional `user`, looks up the matching row in `error_logs`
  via `DbManipulator`, and writes `{timestamp, user}` to the `acknowledged` column.
- **`tests/com/handler/test_error_log_database_handler.py`**: Added tests for `error_id`
  field mapping (`test_dispatch_maps_error_id`, `test_dispatch_maps_uuid_as_error_id`).


## [0.1.8+build.114] - 2026-03-13

### Build

update errorfeed with handler to store errorfeeds in database

### Added
- **`com/handler/interfaces.py`**: `activated: bool` attribute on `IFeederHandler` (default `True`).
  `activate()` / `deactivate()` convenience methods. `emit()` now short-circuits when `activated=False`.
- **`com/feeder/feeder.py`**: `_dispatch_to_handlers(wire_data)` helper — fire-and-forget
  `asyncio.create_task` dispatch to all active handlers after every `_publish()` call (success and failure).
- **`storage/tb_error_log.py`**: New SQLAlchemy model `ErrorLog` for the `error_logs` ring-buffer
  table (fields: `id`, `occured_at`, `error_code`, `severity`, `context_snap`, `message`,
  `acknowledged`). `ERROR_LOG_MAX_ROWS = 10_000`.
- **`com/handler/error_log_database.py`**: New generic `ErrorLogDatabaseHandler` with
  `FieldSpec`-based validation and ID-rotation ring buffer (`(counter % max_rows) + 1`).
  Writes via `DbManipulator`; upserts existing slots, inserts empty ones. `configure(db_access)`
  enables late binding after `setup_storage()`.
- **`core/entity.py`** — `VyraEntity`:
  - `__init_feeder()` registers an **inactive** `ErrorLogDatabaseHandler` placeholder on the
    error feeder at startup (no `DbAccess` needed yet).
  - `_activate_errorfeed_db_handler()` helper: creates `error_logs` table and activates the handler
    once persistent storage is ready.
  - `setup_storage()` calls `_activate_errorfeed_db_handler()` automatically after storage init.

### Fixed
- **`com/feeder/feeder.py`**: Handlers in `self._handler` were never called during the publish path.
  They are now dispatched by `_dispatch_to_handlers()`.


## [0.1.8+build.113] - 2026-03-13

### Build

Rebuild all the changed stuff

### Fixed
- **`__init__.py` / `extract_interfaces()`**: Fehlender `mkdir` für `msg/`, `srv/`, `action/` Unterverzeichnisse im Ziel-Interface-Paket. `shutil.copy2` scheiterte stumm, wenn das Unterverzeichnis noch nicht existierte — dadurch wurden `.msg`-Dateien aus `vyra_base/interfaces/msg/` nicht ins Modul-Interface-Paket kopiert. Jetzt wird `t_subs_path.mkdir(parents=True, exist_ok=True)` vor dem Kopier-Loop ausgeführt. Zusätzlich wird übersprungen, wenn das Quell-Unterverzeichnis nicht existiert (`continue`).

### Changed (Plugin HostFunctions Refactoring)
- **`plugin/host_functions.py`**: `zenoh_get`/`zenoh_put` aus Protocol, `BaseHostFunctions` und `NullHostFunctions` entfernt. Neue abstrakte Methoden: `create_publisher`, `create_subscriber`, `create_server`, `create_client` \u2014 spiegeln `InterfaceFactory`-Signaturen wider.

### Added
- **`plugin/plugin_facade.py`**: Neuer `PluginFacade` als Permission-Gate + Rate-Limiter \u00fcber `BaseHostFunctions`. Pr\u00fcft `permissions.interfaces` aus `metadata.json` (Transport-Typ: subscribe/publish/server/client; Zielmodul: module_name, module_id). Blockiert oder verlangsamt Plugins bei \u00fcberm\u00e4\u00dfigen Anfragen.


## [0.1.8+build.110] - 2026-03-09

### Build

add new plugin structure

### Changed (Plugin-System Generalisierung)
- **`plugin/runtime.py` — WasmRuntime generisch**: Hardcodierte counter-widget-Logik (`init`, `increment`, `reset`) entfernt. Die Runtime liest jetzt `metadata.json exports[]` beim Start und dispatcht alle Funktionen dynamisch (metadata-gesteuerter i32-Dispatch). Kein statisches Wissen über einzelne Plugins mehr.
- **`plugin/runtime.py` — StubRuntime bereinigt**: Counter-spezifische Methoden (`_fn_init`, `_fn_increment`, `_fn_reset`) entfernt. Nur generische Operationen bleiben (`ping`, `get_state`, `set_state`).
- **`plugin/host_functions.py` — BaseHostFunctions**: Neue abstrakte Basisklasse `BaseHostFunctions(ABC)` hinzugefügt. Konkretes `log()`, abstrakte `notify_ui()`, `zenoh_get()`, `zenoh_put()`.
- **`plugin/__init__.py`**: `BaseHostFunctions` exportiert.

### Added
- **`interfaces/config/vyra_plugin.meta.json`**: Neue Zenoh-Interface-Definition für `plugin_event` Publisher (Typ: message, Namespace: plugin).
- **`docs/backend/PLUGIN_SYSTEM.md`**: Vollständige Architekturdokumentation des Plugin-Systems.

## [0.1.9] - 2026-03-06

### Added

- `vyra_base.plugin` Modul: Plugin-Runtime-Abstraktion für WASM-Plugins
  - `PluginRuntime` (ABC): Gemeinsame API für WASM- und Stub-Implementierungen
  - `StubRuntime`: Python-Stub ohne WASM-Execution — vollständig funktional für Entwicklung/Tests; implementiert `init`, `increment`, `reset`, `get_state`, `ping`, `set_state`
  - `HostFunctions` (Protocol): Schnittstelle für Host-Funktionen (`notify_ui`, `zenoh_get`, `zenoh_put`, `log`)
  - `NullHostFunctions`: No-op Implementierung für Tests
  - `PluginCallError`: Exception bei fehlgeschlagenen Plugin-Aufrufen
- WASM-Execution (Extism) als Phase 2 vorbereitet — API bleibt kompatibel

## [0.1.8+build.109] - 2026-03-06

### Build

Update parameter structure. Implementing ruleset and create_parameter

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

