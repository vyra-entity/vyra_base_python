# VYRA Plugin-System — Architektur & Entwicklerdokumentation

## Übersicht

Das VYRA Plugin-System ermöglicht die dynamische Erweiterung von Modulen durch WebAssembly-Plugins (WASM), die zur Laufzeit installiert, ausgeführt und deinstalliert werden können.

```
┌────────────────────────────────────────────────────────────────────────┐
│  Browser (v2_dashboard)                                                │
│    └─► GET /v2_modulemanager/api/plugin/ui-manifest?scope=MODULE       │
│          └─► v2_modulemanager FastAPI                                  │
│                └─► DB: plugin_assignments ∝ ui_component_registry      │
│                      └─► Returniert: { slots: { "home-widget": [...] }}│
│    └─► import(js_entry_point) → WASM-Widget lädt sich selbst           │
│                                                                        │
│  Modul X Backend ──Zenoh──► v2_modulemanager                           │
│    "plugin/get_plugin_ui_manifest"  ◄── modulemanager_plugin.meta.json │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Komponenten

### 1. `vyra_base.plugin.runtime` — WasmRuntime / StubRuntime

**Pfad**: `src/vyra_base/plugin/runtime.py`

Führt WASM-Plugin-Funktionen aus. Die Runtime ist **generisch** — kein statisches Wissen über einzelne Plugins.

#### Metadaten-gesteuerter Dispatch

Alle exportierten Funktionen werden aus `metadata.json` geladen:

```json
{
  "exports": [
    { "name": "init",      "args": [{"name": "initial_value", "type": "i32"}, {"name": "step", "type": "i32"}] },
    { "name": "increment", "args": [{"name": "step", "type": "i32"}] },
    { "name": "get_count", "args": [] }
  ]
}
```

Die `WasmRuntime` mappt beim `call()` automatisch Dict-Keys zu i32-Argumenten in der Reihenfolge, die `exports[]` vorgibt.

```python
# Aufruf:
await runtime.call("increment", {"step": 3})
# → WASM-Export: increment(3: i32) → i32
```

#### StubRuntime

Fallback wenn `wasmtime` nicht verfügbar oder WASM-Datei fehlt. Unterstützt nur generische Operationen (`ping`, `get_state`, `set_state`).

---

### 2. `vyra_base.plugin.host_functions` — HostFunctions-Protokoll

**Pfad**: `src/vyra_base/plugin/host_functions.py`

Definiert die Schnittstelle zwischen WASM-Plugin und dem Host-Modul.

#### Klassen

| Klasse | Beschreibung |
|--------|-------------|
| `HostFunctions` | Protocol (Structural Typing) — minimale Interface-Definition |
| `NullHostFunctions` | No-Op-Implementierung für Tests/Stub |
| `BaseHostFunctions` | Abstrakte Basisklasse mit konkretem `log()`, abstrakter `notify_ui()`, `zenoh_get()`, `zenoh_put()` |

#### Implementierung in einem Modul

```python
from vyra_base.plugin import BaseHostFunctions
from vyra_base.com.transport import VyraPublisher

class MyModuleHostFunctions(BaseHostFunctions):
    def __init__(self, publisher: VyraPublisher, zenoh_session):
        self._publisher = publisher
        self._session = zenoh_session

    async def notify_ui(self, plugin_id: str, event_name: str, data: dict) -> None:
        await self._publisher.publish({
            "plugin_id": plugin_id,
            "event_name": event_name,
            "data": data,
        })

    async def zenoh_get(self, key: str) -> dict:
        return await self._session.get(key)

    async def zenoh_put(self, key: str, value: dict) -> None:
        await self._session.put(key, value)
```

Die konkreten HostFunctions für `v2_modulemanager` befinden sich in:
`v2_modulemanager/backend_webserver/plugin/host_functions_impl.py`

---

### 3. Zenoh-Interfaces (`vyra_plugin.meta.json`)

**Pfad**: `src/vyra_base/interfaces/config/vyra_plugin.meta.json`

Definiert den `plugin_event`-Publisher für UI-Benachrichtigungen von WASM-Plugins.

```json
[{
  "type":         "message",
  "namespace":    "plugin",
  "functionname": "plugin_event",
  "displayname":  "Plugin Event",
  "description":  "UI-Benachrichtigung von einem laufenden WASM-Plugin",
  "filetype":     "publisher",
  "params": [...],
  "returns": [
    {"name": "event_name", "datatype": "string"},
    {"name": "plugin_id",  "datatype": "string"},
    {"name": "data",       "datatype": "object"}
  ]
}]
```

Topic: `{module_name}_{module_id}/plugin/plugin_event`

---

## Datenfluss: Plugin installieren

```
Browser  ──POST /plugin/install──►  v2_modulemanager FastAPI
                                         │
                                         ▼
                              POST http://container-manager:8080
                              /plugins/install  (Checksum + NFS-Copy)
                                         │
                                         ▼
                              Lese metadata.json aus pool_path/
                                         │
                                         ▼
                              Schreibe in DB:
                              - plugin_pool (p_id, nfs_path, metadata)
                              - plugin_assignments (scope, active)
                              - ui_component_registry (slot, js_entry_point)
```

---

## Datenfluss: Plugin UI laden (Frontend)

```
Browser  ──GET /plugin/ui-manifest──►  v2_modulemanager FastAPI
                                            │
                                            ▼
                                SQL: ui_component_registry
                                     JOIN plugin_assignments
                                     WHERE scope_type = X
                                            │
                                            ▼
                              { slots: { "home-widget": [{
                                  component_name: "CounterWidget",
                                  js_entry_point: "/v2_modulemanager/api/plugin/assets/..."
                                }]}}
                                            │
Browser  ──import(js_entry_point)──────────►  WVM lädt Vue-Komponente
```

---

## Datenfluss: Plugin-Funktion aufrufen

```
Browser  ──POST /plugin/{pluginId}/call { function_name, data }──►
                v2_modulemanager FastAPI
                    │
                    ▼
              DB: SELECT nfs_path FROM plugin_pool WHERE plugin_name_id = pluginId
                    │
                    ▼
              plugin_manager.get_or_start_runtime(plugin_id, nfs_wasm_path)
                    │
                    ▼
              WasmRuntime._dispatch_wasm(function_name, data)
                → metadata.json exports[] → arg-mapping → WASM-Call
                    │
                    ▼
              notify_ui("plugin_event", {"event_name": "counter.result", ...})
              → Zenoh VyraPublisher → Frontend EventBus
```

---

## Plugin-Metadata Format

Jedes Plugin hat im Repository eine `metadata.json`:

```json
{
  "id":      "my-plugin",
  "name":    "My Plugin",
  "version": "1.0.0",
  "description": "...",
  "exports": [
    {
      "name": "init",
      "args": [
        {"name": "initial_value", "type": "i32"},
        {"name": "step",          "type": "i32"}
      ]
    },
    {
      "name": "my_function",
      "args": [{"name": "param1", "type": "i32"}]
    }
  ],
  "entry_points": {
    "wasm":     "logic.wasm",
    "frontend": {
      "index": "ui/index.js",
      "slots": [
        {"id": "home-widget", "component": "MyPluginWidget"}
      ]
    }
  },
  "checksum": "sha256:..."
}
```

---

## In vyra_base exportierte Symbole

```python
from vyra_base.plugin import (
    BaseHostFunctions,    # Abstrakte Basisklasse für HostFunctions
    NullHostFunctions,    # No-Op für Tests
    HostFunctions,        # Protocol
    PluginCallError,      # Exception bei WASM-Fehler
    PluginRuntime,        # Basis-Klasse der Runtimes
    WasmRuntime,          # Produktive WASM Runtime
    StubRuntime,          # Stub/Fallback Runtime
    create_plugin_runtime, # Factory-Funktion
)
```

---

## Verwandte Dateien

| Datei | Beschreibung |
|-------|-------------|
| `vyra_base/plugin/runtime.py` | WasmRuntime + StubRuntime |
| `vyra_base/plugin/host_functions.py` | HostFunctions-Protokoll + BaseHostFunctions |
| `vyra_base/interfaces/config/vyra_plugin.meta.json` | Zenoh plugin_event Publisher |
| `v2_modulemanager/plugin/host_functions_impl.py` | Konkrete ModuleManagerHostFunctions |
| `v2_modulemanager/plugin/models.py` | PluginManager-Singleton |
| `v2_modulemanager/plugin/router.py` | REST-Endpunkte |
| `v2_modulemanager_interfaces/config/modulemanager_plugin.meta.json` | Zenoh-Services |
| `local_module_repository/plugins/*/metadata.json` | Plugin-Metadaten (inkl. exports[]) |
