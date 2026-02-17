# VYRA Decorator System Refactoring - Status Document

**Datum:** 17. Februar 2026  
**Status:** 🟡 In Arbeit - Phase 1 & 2 abgeschlossen, Phase 3-5 ausstehend  
**Branch:** vyra_base_python / main

## Übersicht

Umfassende Refaktorierung des VYRA Decorator-Systems von einem monolithischen Ansatz zu einem **Two-Phase Late-Binding System** mit Blueprint-Infrastruktur.

### Ziel

Trennung von Interface-**Definition** (was) und **Implementation** (wie) durch:
- **Phase 1 (Definition):** Blueprints werden registriert (aus Decorators oder JSON Metadata)
- **Phase 2 (Binding):** Callbacks werden an Blueprints gebunden (wenn Component geladen wird)
- **Phase 3 (Creation):** Interfaces werden aus gebundenen Blueprints erstellt

### Kernprobleme der alten Architektur (GELÖST)

❌ **Tight Coupling:** Interface-Erstellung erforderte sofort verfügbare Callbacks  
❌ **Keine Late Binding:** Interfaces konnten nicht vor Components existieren  
❌ **Schwer testbar:** Mock-Callbacks erforderten volle Component-Struktur  
❌ **Pending Mechanism:** Existierte aber wurde nie aufgerufen (`loop_check_pending()`)  

✅ **Neue Lösung:** Blueprint-Registry mit explizitem Late Binding

---

## ✅ FERTIGGESTELLT (Implementierte Komponenten)

### 1. Blueprint Infrastructure (`blueprints.py`) ✅

**Datei:** `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/blueprints.py`

**Implementiert:**
- `HandlerBlueprint` (ABC) - Basis für alle Blueprints
- `ServiceBlueprint` - Request/Response (ROS2 Service, gRPC)
- `PublisherBlueprint` - Pub/Sub Sender (ROS2 Topic, Redis)
- `SubscriberBlueprint` - Pub/Sub Empfänger
- `ActionBlueprint` - Long-running Tasks (ROS2 Action)
- `InterfaceType` Enum
- Callback Validation (Signatur-Checks)
- `bind_callback()` / `unbind_callback()` / `is_bound()` Methoden

**Key Features:**
```python
blueprint = ServiceBlueprint(
    name="calculate",
    protocols=[ProtocolType.ROS2, ProtocolType.ZENOH],
    metadata={"qos": 10},
    service_type=MyServiceType
)

# Später:
blueprint.bind_callback(my_calculate_function)
```

### 2. Callback Registry (`callback_registry.py`) ✅

**Datei:** `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/callback_registry.py`

**Implementiert:**
- Thread-safe Singleton Pattern
- Per-Module Namespacing (`module_name/interface_name`)
- Late Binding Support
- Debugging Tools (`debug_print()`, `list_unbound()`)

**Key Methods:**
```python
# Registrierung
CallbackRegistry.register_blueprint(blueprint, namespace="v2_modulemanager")

# Binding
CallbackRegistry.bind_callback("calculate", my_callback, namespace="v2_modulemanager")

# Queries
CallbackRegistry.list_unbound(namespace="v2_modulemanager")
CallbackRegistry.get_statistics(namespace="v2_modulemanager")
CallbackRegistry.debug_print()
```

### 3. Refaktorierte Decorators (`decorators.py`) ✅

**Datei:** `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/decorators.py`

**⚠️ WICHTIG: Namensänderung durchgeführt:**
- ~~`_vyra_remote_callable`~~ → `_vyra_remote_server`
- ~~`_vyra_remote_speaker`~~ → `_vyra_remote_publisher`
- ~~`_vyra_remote_listener`~~ → `_vyra_remote_subscriber`
- ~~`_vyra_remote_job`~~ → `_vyra_remote_action`

**Implementierte Decorators:**
```python
@remote_service(name="calculate", protocols=[...], namespace="module")
async def calculate(self, request, response=None):
    return {"result": 42}

@remote_publisher(name="status", protocols=[...])
async def publish_status(self, message):
    pass

@remote_subscriber(name="updates", protocols=[...])
async def on_update(self, message):
    pass

@remote_actionServer(name="process", protocols=[...])
async def process_batch(self, goal_handle):
    return {"processed": 100}
```

**Helper Functions:**
```python
# Alle decorated methods finden
methods = get_decorated_methods(component)
# Returns: {"servers": [...], "publishers": [...], "subscribers": [...], "actions": [...]}

# Callbacks binden
results = bind_decorated_callbacks(component, namespace="module_name")
```

### 4. InterfaceFactory Erweiterungen (`factory.py`) ✅

**Datei:** `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/factory.py`

**Neue Methoden:**
```python
# Aus Blueprint erstellen
interface = await InterfaceFactory.create_from_blueprint(blueprint, **overrides)

# Pending Callback binden
interface = await InterfaceFactory.bind_pending_callback(name, callback)

# Alle Pending verarbeiten
results = await InterfaceFactory.process_pending_interfaces()

# Utilities
count = InterfaceFactory.get_pending_count()
names = InterfaceFactory.list_pending()
has = InterfaceFactory.has_pending("my_service")
```

**Verbesserte Pending Logic:**
- `_pending_interface` Dict wird nun aktiv verwendet
- Background Processing möglich via `process_pending_interfaces()`
- Blueprint-aware Pending Management

### 5. Exports & Integration (`__init__.py`) ✅

**Dateien:**
- `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/__init__.py`
- `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/__init__.py`

**Exports:**
```python
from vyra_base.com import (
    # Blueprints
    HandlerBlueprint,
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    CallbackRegistry,
    
    # Decorators
    remote_service,
    remote_publisher,
    remote_subscriber,
    remote_actionServer,
    get_decorated_methods,
    bind_decorated_callbacks,
    
    # Factory
    InterfaceFactory,
    
    # Backward Compatibility (DEPRECATED)
    remote_callable,      # → remote_service
    remote_speaker,       # → remote_publisher
    remote_listener,      # → remote_subscriber
    remote_job,           # → remote_actionServer
)
```

### 6. Examples (`examples/`) ✅

**Dateien:**
- `example_basic_service.py` - Two-Phase Pattern Demonstration
- `example_publisher_property.py` - Property Setter Auto-Publishing Pattern
- `README.md` - Vollständige Dokumentation der Examples

**Features:**
- ✅ Old vs. New Comparison
- ✅ Blueprint Registration Demo
- ✅ Late Binding Demo
- ✅ Property Setter Pattern (wie Untitled-2 Sketch)
- ✅ Ausführbare Beispiele

---

## 🟡 TEILWEISE FERTIG / ÜBERSPRUNGEN

### 7. Abstract Handler Interfaces (Optional)

**Status:** ⚠️ Übersprungen (niedrige Priorität)

**Geplant war:** 
```python
# handler_interfaces.py
class IServiceHandler(ABC):
    @abstractmethod
    async def handle_request(self, request): pass

class IActionHandler(ABC):
    @abstractmethod
    async def execute(self, goal_handle): pass
```

**Entscheidung:** 
- Decorators funktionieren ohne Inheritance
- ABCs sind optional für komplexe Handler
- **Kann später bei Bedarf hinzugefügt werden**

---

## ❌ NOCH ZU TUN (Nächste Schritte)

### 8. Entity Integration ❌ WICHTIG

**Datei:** `/home/holgder/VYRA/vyra_base_python/src/vyra_base/core/entity.py`

**Was zu tun ist:**

#### A) `set_interfaces()` Methode erweitern (ca. Zeile 870-955)

**Aktueller Code:**
```python
async def set_interfaces(self, settings: list[FunctionConfigEntry]) -> None:
    for setting in settings:
        if setting.type == FunctionConfigBaseTypes.service.value:
            await InterfaceFactory.create_server(
                name=setting.functionname,
                response_callback=setting.callback,  # ← Direkt übergeben
                protocols=[ProtocolType.ROS2],
                service_type=setting.interfacetypes,
                node=self._node
            )
```

**Zu ändern auf:**
```python
async def set_interfaces(
    self, 
    settings: Union[list[FunctionConfigEntry], list[HandlerBlueprint]]
) -> None:
    """
    Initialize interfaces from FunctionConfigEntry (legacy) or HandlerBlueprint (new).
    
    Args:
        settings: List of interface configurations or blueprints
    """
    for setting in settings:
        # NEW: Check if it's a blueprint
        if isinstance(setting, HandlerBlueprint):
            interface = await InterfaceFactory.create_from_blueprint(
                setting,
                node=self._node,  # Add node from entity
                module_name=self.namespace
            )
            if interface is None:
                logger.info(f"Interface '{setting.name}' pending (awaiting callback)")
            continue
        
        # LEGACY: Handle FunctionConfigEntry
        if setting.type == FunctionConfigBaseTypes.service.value:
            # ... existing code ...
```

#### B) `bind_interface_callbacks()` Methode hinzufügen

**Neu erstellen:**
```python
def bind_interface_callbacks(self, component: Any) -> Dict[str, bool]:
    """
    Bind decorated methods from component to registered blueprints.
    
    This is called after component initialization to bind callbacks to
    blueprints that were registered during entity creation.
    
    Args:
        component: Component instance with decorated methods
        
    Returns:
        Dictionary mapping interface names to binding success
        
    Example:
        >>> entity = await build_entity(...)
        >>> component = Component(entity=entity)
        >>> entity.bind_interface_callbacks(component)
    """
    return bind_decorated_callbacks(
        component, 
        namespace=self.namespace
    )
```

#### C) Background Task für Pending Processing starten

**In `startup_entity()` ergänzen (ca. Zeile 644-712):**
```python
async def startup_entity(self) -> bool:
    """Startup entity and begin processing pending interfaces."""
    # ... existing code ...
    
    # NEW: Start background task for pending interface processing
    if hasattr(self, '_pending_processor_task'):
        self._pending_processor_task.cancel()
    
    self._pending_processor_task = asyncio.create_task(
        self._process_pending_interfaces_loop()
    )
    logger.info("✅ Pending interface processor started")
    
    return True

async def _process_pending_interfaces_loop(self):
    """Background task to process pending interfaces."""
    while True:
        try:
            await asyncio.sleep(1.0)  # Check every second
            results = await InterfaceFactory.process_pending_interfaces()
            if results:
                logger.debug(f"Processed {len(results)} pending interfaces")
        except asyncio.CancelledError:
            logger.info("Pending interface processor stopped")
            break
        except Exception as e:
            logger.error(f"Error in pending processor: {e}")
```

**In `shutdown_entity()` ergänzen:**
```python
async def shutdown_entity(self) -> bool:
    # ... existing code ...
    
    # NEW: Stop pending processor
    if hasattr(self, '_pending_processor_task'):
        self._pending_processor_task.cancel()
        try:
            await self._pending_processor_task
        except asyncio.CancelledError:
            pass
```

### 9. Module Integration Pattern ❌ WICHTIG

**Dateien zu aktualisieren:**
- `/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_*/src/v2_modulemanager/v2_modulemanager/interface.py`
- `/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_*/src/v2_modulemanager/v2_modulemanager/_base_.py`

#### A) `_create_base_interfaces()` in `_base_.py` anpassen

**Aktuell (ca. Zeile 120-300):**
```python
async def _create_base_interfaces():
    """Create interface configs from JSON metadata."""
    interface_functions = []
    
    for metadata in load_json(...):
        interface_functions.append(
            FunctionConfigEntry(
                functionname=metadata["functionname"],
                callback=None,  # ← Problem: Kein Callback!
                # ...
            )
        )
    
    return interface_functions
```

**Ändern auf:**
```python
async def _create_base_interfaces():
    """Create blueprints from JSON metadata (Phase 1)."""
    blueprints = []
    
    for metadata in load_json(...):
        # Determine blueprint type
        if metadata["type"] == "service":
            blueprint = ServiceBlueprint(
                name=metadata["functionname"],
                protocols=[ProtocolType.ROS2],
                metadata=metadata,
                service_type=load_ros2_type(metadata["filetype"])
            )
        elif metadata["type"] == "publisher":
            blueprint = PublisherBlueprint(...)
        # etc.
        
        # Register in global registry
        CallbackRegistry.register_blueprint(
            blueprint, 
            namespace="v2_modulemanager"
        )
        
        blueprints.append(blueprint)
    
    return blueprints
```

#### B) `auto_register_callable_interfaces()` in `interface.py` anpassen

**Aktuell (ca. Zeile 26-99):**
```python
async def auto_register_callable_interfaces(entity, callback_parent):
    """Discover decorated methods and create FunctionConfigEntry."""
    callback_list = _autoload_all_remote_callable_from_parent(callback_parent)
    interface_metadata = _load_metadata(...)
    
    for callback in callback_list:
        # Match by name
        configs = [m for m in interface_metadata if m['functionname'] == callback.__name__]
        # Create FunctionConfigEntry with callback
        interface_functions.append(_register_callable_interface(..., callback=callback))
    
    await entity.set_interfaces(interface_functions)
```

**Ändern auf:**
```python
async def auto_register_callable_interfaces(entity, callback_parent):
    """Bind callbacks to registered blueprints (Phase 2)."""
    
    # Discover decorated methods
    decorated = get_decorated_methods(callback_parent)
    
    # Bind to blueprints in registry
    results = bind_decorated_callbacks(
        callback_parent,
        namespace="v2_modulemanager"
    )
    
    # Create interfaces from bound blueprints
    all_bound = CallbackRegistry.list_bound(namespace="v2_modulemanager")
    
    for name in all_bound:
        blueprint = CallbackRegistry.get_blueprint(name, namespace="v2_modulemanager")
        if blueprint:
            interface = await InterfaceFactory.create_from_blueprint(
                blueprint,
                node=entity.node
            )
            if interface:
                logger.info(f"✅ Created interface: {name}")
    
    logger.info(f"📊 Interface registration complete: {len(results)} bound")
```

### 10. Tests ❌

**Zu erstellen:**
- `tests/com/test_blueprints.py` - Blueprint Creation, Binding, Validation
- `tests/com/test_callback_registry.py` - Registry Operations, Thread Safety
- `tests/com/test_decorators_new.py` - New Decorator Behavior
- `tests/com/test_integration_late_binding.py` - End-to-End Late Binding

**Beispiel Test Structure:**
```python
# tests/com/test_blueprints.py
import pytest
from vyra_base.com.core.blueprints import ServiceBlueprint

def test_blueprint_creation():
    bp = ServiceBlueprint(name="test", protocols=[])
    assert bp.name == "test"
    assert not bp.is_bound()

def test_blueprint_binding():
    bp = ServiceBlueprint(name="test", protocols=[])
    
    async def callback(request, response=None):
        return {"result": 42}
    
    bp.bind_callback(callback)
    assert bp.is_bound()
    assert bp.callback == callback

def test_blueprint_signature_validation():
    bp = ServiceBlueprint(name="test", protocols=[])
    
    def invalid_callback():  # Missing request parameter
        pass
    
    with pytest.raises(ValueError, match="must accept at least 1 parameter"):
        bp.bind_callback(invalid_callback)
```

### 11. Dokumentation ❌

**Zu erstellen:**

#### A) `DECORATOR_GUIDE.md`
```markdown
# VYRA Decorator Guide

## Overview
Two-phase initialization pattern for communication interfaces.

## Decorators
### @remote_service
### @remote_publisher
### @remote_subscriber
### @remote_actionServer

## API Reference
## Migration from Old System
## Best Practices
```

#### B) `MIGRATION_GUIDE.md`
```markdown
# Migration Guide: Old → New Decorator System

## Breaking Changes
- `remote_callable` → `remote_service`
- `remote_speaker` → `remote_publisher`
- Internal attributes renamed:
  - `_vyra_remote_callable` → `_vyra_remote_server`
  - `_vyra_callable_name` → `_vyra_server_name`
  - etc.

## Step-by-Step Migration
1. Update decorators
2. Update interface.py
3. Update _base_.py
4. Test

## Example Before/After
```

#### C) Update `com/README.md`
- Neue Architecture Diagram
- Blueprint System Erklärung
- Two-Phase Pattern Flowchart

#### D) Update `.github/copilot-instructions.md`
- Decorator System Section aktualisieren
- Blueprint Pattern dokumentieren

---

## 🎯 PRIORITÄTEN FÜR MORGEN

### Phase 3: Entity Integration (KRITISCH)
**Geschätzte Zeit:** 2-3 Stunden

1. ✅ `entity.py` Zeile 870-955: `set_interfaces()` erweitern für Blueprints
2. ✅ `entity.py`: `bind_interface_callbacks()` Methode hinzufügen
3. ✅ `entity.py` Zeile 644-712: Background Task für Pending Processing
4. ✅ Testen mit Simple Example

### Phase 4: Module Migration (KRITISCH)
**Geschätzte Zeit:** 3-4 Stunden

1. ✅ `v2_modulemanager/_base_.py`: `_create_base_interfaces()` → Blueprints
2. ✅ `v2_modulemanager/interface.py`: `auto_register_callable_interfaces()` → Binding
3. ✅ `v2_modulemanager/application.py`: Decorators auf neue Namen prüfen
4. ✅ Full Module Startup Test

### Phase 5: Testing & Documentation (MITTEL)
**Geschätzte Zeit:** 2-3 Stunden

1. ⚠️ Basis Tests für Blueprints & Registry
2. ⚠️ Integration Test: Full Two-Phase Flow
3. ⚠️ DECORATOR_GUIDE.md erstellen
4. ⚠️ MIGRATION_GUIDE.md erstellen

---

## 📐 ARCHITEKTUR-ÜBERSICHT

### Datenfluss (Neu)

```
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 1: BLUEPRINT REGISTRATION (Entity Init)                   │
└─────────────────────────────────────────────────────────────────┘
                              │
    ┌─────────────────────────┼─────────────────────────┐
    │                         │                         │
    v                         v                         v
JSON Metadata          Class Definition          Manual Creation
    │                  (@remote_service)                │
    │                         │                         │
    v                         v                         v
┌───────────────────────────────────────────────────────────────┐
│ ServiceBlueprint(name="calc", protocols=[...])                │
│ - name: "calculate"                                           │
│ - protocols: [ROS2, Zenoh]                                    │
│ - metadata: {qos:10, ...}                                     │
│ - callback: None  ← NOT YET BOUND                             │
└───────────────────────────────────────────────────────────────┘
                              │
                              v
              ┌───────────────────────────────┐
              │ CallbackRegistry.register()   │
              │ Namespace: "v2_modulemanager" │
              └───────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ PHASE 2: CALLBACK BINDING (Component Init)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
    Component instance created with decorated methods
                              │
                              v
              ┌───────────────────────────────┐
              │ get_decorated_methods()       │
              │ Returns: {servers: [...], ... }│
              └───────────────────────────────┘
                              │
                              v
              ┌───────────────────────────────┐
              │ bind_decorated_callbacks()    │
              │ Binds methods to blueprints   │
              └───────────────────────────────┘
                              │
                              v
              Blueprint.bind_callback(method)
              blueprint.is_bound() == True ✅

┌─────────────────────────────────────────────────────────────────┐
│ PHASE 3: INTERFACE CREATION (Late Init or Immediate)            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              v
          ┌───────────────────────────────────┐
          │ InterfaceFactory.create_from_    │
          │ _blueprint(blueprint)             │
          └───────────────────────────────────┘
                              │
          ┌─────────────────┴─────────────────┐
          │                                   │
          v                                   v
    Callback Bound?                    Callback Missing?
          │                                   │
          v                                   v
┌─────────────────────┐            ┌──────────────────────┐
│ Create Interface    │            │ Add to Pending       │
│ - ROS2 Service      │            │ Queue                │
│ - Redis Server      │            │                      │
│ - etc.              │            │ Background task      │
│                     │            │ retries when bound   │
└─────────────────────┘            └──────────────────────┘
          │
          v
  ✅ Interface Active
```

### Klassen-Beziehungen

```
HandlerBlueprint (ABC)
  ├── ServiceBlueprint
  ├── PublisherBlueprint
  ├── SubscriberBlueprint
  └── ActionBlueprint
          │
          │ registered in
          v
   CallbackRegistry (Singleton)
          │
          │ used by
          v
   InterfaceFactory
          │
          │ creates
          v
   VyraServer / VyraPublisher / etc.
```

---

## 📝 WICHTIGE ENTSCHEIDUNGEN

### 1. Namenskonvention (17.02.2026)

**ALTE Namen (DEPRECATED):**
- `remote_callable` → `remote_service` ✅
- `remote_speaker` → `remote_publisher` ✅
- `remote_listener` → `remote_subscriber` ✅
- `remote_job` → `remote_actionServer` ✅

**Interne Attribute:**
- `_vyra_remote_callable` → `_vyra_remote_server` ✅
- `_vyra_callable_name` → `_vyra_server_name` ✅
- `_vyra_remote_speaker` → `_vyra_remote_publisher` ✅
- `_vyra_speaker_name` → `_vyra_publisher_name` ✅
- `_vyra_remote_listener` → `_vyra_remote_subscriber` ✅
- `_vyra_listener_name` → `_vyra_subscriber_name` ✅
- `_vyra_remote_job` → `_vyra_remote_action` ✅
- `_vyra_job_name` → `_vyra_action_name` ✅

**get_decorated_methods() Return Keys:**
- `"callables"` → `"servers"` ✅
- `"speakers"` → `"publishers"` ✅
- `"listeners"` → `"subscribers"` ✅
- `"jobs"` → `"actions"` ✅

### 2. Backward Compatibility

**Entscheidung:** Soft Break mit Deprecation Warnings
- Alte Decorator-Namen als Aliases beibehalten (mit DEPRECATED Kommentar)
- Interne Attribute umbenannt (Clean Break für interne API)
- Module müssen migriert werden

### 3. Metadata Source

**Entscheidung:** JSON bleibt Source of Truth
- Decorators referenzieren JSON Metadata
- Blueprints werden aus JSON erstellt
- Decorators markieren nur Callbacks für Binding

### 4. Binding Timing

**Entscheidung:** Two-Phase mit Background Processing
- Phase 1: Blueprints bei Entity Init
- Phase 2: Callbacks bei Component Init
- Background Task verarbeitet Pending Interfaces kontinuierlich

### 5. Optional ABCs

**Entscheidung:** NICHT implementiert (jetzt)
- Blueprints + Decorators sind ausreichend
- Inheritance nicht erforderlich
- Kann später bei Bedarf hinzugefügt werden

---

## 🔍 CODE-SNIPPETS FÜR FORTSETZUNG

### Entity Integration Template

```python
# In entity.py - set_interfaces() erweitern
from vyra_base.com.core.blueprints import HandlerBlueprint

async def set_interfaces(
    self, 
    settings: Union[list[FunctionConfigEntry], list[HandlerBlueprint]]
) -> None:
    for setting in settings:
        if isinstance(setting, HandlerBlueprint):
            # NEW PATH
            interface = await InterfaceFactory.create_from_blueprint(
                setting,
                node=self._node,
                module_name=self.namespace
            )
            logger.info(f"Created interface from blueprint: {setting.name}")
        else:
            # LEGACY PATH (existing code)
            # ... handle FunctionConfigEntry ...
```

### Module Integration Template

```python
# In _base_.py - _create_base_interfaces()
from vyra_base.com import ServiceBlueprint, CallbackRegistry

async def _create_base_interfaces():
    blueprints = []
    metadata_list = await _load_interface_metadata()
    
    for meta in metadata_list:
        if meta["type"] == "service":
            bp = ServiceBlueprint(
                name=meta["functionname"],
                protocols=[ProtocolType.ROS2],
                metadata=meta,
                service_type=_load_service_type(meta["filetype"])
            )
            CallbackRegistry.register_blueprint(
                bp, 
                namespace="v2_modulemanager"
            )
            blueprints.append(bp)
    
    return blueprints
```

### Interface.py Template

```python
# In interface.py - auto_register_callable_interfaces()
from vyra_base.com import (
    bind_decorated_callbacks,
    CallbackRegistry,
    InterfaceFactory
)

async def auto_register_callable_interfaces(entity, callback_parent):
    # Bind callbacks to blueprints
    results = bind_decorated_callbacks(
        callback_parent,
        namespace="v2_modulemanager"
    )
    
    # Create interfaces from bound blueprints
    for name in CallbackRegistry.list_bound(namespace="v2_modulemanager"):
        blueprint = CallbackRegistry.get_blueprint(name, namespace="v2_modulemanager")
        interface = await InterfaceFactory.create_from_blueprint(
            blueprint,
            node=entity.node
        )
        logger.info(f"✅ Interface created: {name}")
```

---

## 🧪 TEST-STRATEGIE

### Unit Tests

```python
# test_blueprints.py
def test_service_blueprint_creation()
def test_blueprint_binding()
def test_blueprint_validation()
def test_blueprint_unbinding()

# test_callback_registry.py
def test_registry_initialization()
def test_blueprint_registration()
def test_blueprint_retrieval()
def test_namespace_isolation()
def test_thread_safety()
def test_statistics()
```

### Integration Tests

```python
# test_integration_late_binding.py
async def test_two_phase_flow():
    # Phase 1: Create blueprint
    bp = ServiceBlueprint(...)
    CallbackRegistry.register_blueprint(bp)
    
    # Phase 2: Create component, bind callback
    component = TestComponent()
    bind_decorated_callbacks(component)
    
    # Phase 3: Create interface
    interface = await InterfaceFactory.create_from_blueprint(bp)
    
    # Test: Call service
    result = await interface.call({"test": "data"})
    assert result is not None
```

---

## 📚 REFERENZEN

### Geänderte Dateien

1. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/blueprints.py` (NEU)
2. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/callback_registry.py` (NEU)
3. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/decorators.py` (REFACTORED)
4. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/factory.py` (ERWEITERT)
5. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/__init__.py` (ERWEITERT)
6. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/__init__.py` (ERWEITERT)
7. ✅ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/com/core/examples/` (NEU)

### Zu ändernde Dateien (Morgen)

8. ❌ `/home/holgder/VYRA/vyra_base_python/src/vyra_base/core/entity.py`
9. ❌ `/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_*/src/v2_modulemanager/v2_modulemanager/_base_.py`
10. ❌ `/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_*/src/v2_modulemanager/v2_modulemanager/interface.py`
11. ❌ `/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_*/src/v2_modulemanager/v2_modulemanager/application/application.py`

### Beispiele zum Testen

```bash
# Blueprints & Registry testen
python -m vyra_base.com.core.examples.example_basic_service

# Property Setter Pattern testen
python -m vyra_base.com.core.examples.example_publisher_property

# Interaktives Testing
python
>>> from vyra_base.com import CallbackRegistry
>>> CallbackRegistry.debug_print()
```

---

## ⚠️ BEKANNTE PROBLEME / OFFENE FRAGEN

### 1. Publisher Lazy Creation Fallback

In `decorators.py`, `remote_publisher()` hat einen Fallback:
```python
if not hasattr(self_obj, publisher_attr):
    logger.warning("Publisher not initialized. This should be created via InterfaceFactory...")
    # Lazy creation as fallback
```

**Frage:** Ist das gewünscht oder sollten wir Exception werfen?

### 2. Background Task Lifecycle

Pending Processing Background Task in Entity - wann starten/stoppen?
- Starten: In `startup_entity()`?
- Stoppen: In `shutdown_entity()`?
- Oder in `set_interfaces()`?

### 3. Error Handling bei Pending

Was passiert wenn Blueprint binding fehlschlägt im Background Task?
- Retry?
- Max Retries?
- Error Event publishen?

---

## 🎯 SUCCESS CRITERIA

### Minimal Viable Product (MVP)

✅ Blueprints können registriert werden  
✅ Callbacks können später gebunden werden  
✅ InterfaceFactory erstellt Interfaces aus Blueprints  
❌ V2_modulemanager startet mit neuem System  
❌ Alle base interfaces laden erfolgreich  
❌ ROS2 Services sind aufrufbar  

### Full Success

❌ Alle Module migriert  
❌ Tests > 80% Coverage  
❌ Dokumentation vollständig  
❌ Performance gleich oder besser  
❌ Keine breaking changes für externe APIs  

---

## 📞 NÄCHSTE SCHRITTE MORGEN (17.02.2026)

### 1. Morning Review (30 min)
- ✅ Dieses Dokument lesen
- ✅ Code Review der 7 fertiggestellten Files
- ✅ Example ausführen: `example_basic_service.py`

### 2. Entity Integration (2-3h)  
- 📝 `entity.py` öffnen, Zeile 870 finden
- 🔧 `set_interfaces()` erweitern für Blueprint Support
- 🔧 `bind_interface_callbacks()` Methode hinzufügen
- 🔧 Background Task in `startup_entity()`
- 🧪 Simple Test: Blueprint → Entity → Interface

### 3. Module Migration (3-4h)
- 📝 `v2_modulemanager/_base_.py` öffnen
- 🔧 `_create_base_interfaces()` → Blueprints
- 📝 `v2_modulemanager/interface.py` öffnen  
- 🔧 `auto_register_callable_interfaces()` → Binding Pattern
- 🧪 Full Module Test: `./tools/vyra_up.sh`
- 🔍 ROS2 Service Call Test

### 4. Testing & Docs (Optional, wenn Zeit)
- 📝 `test_blueprints.py` erstellen
- 📝 `DECORATOR_GUIDE.md` beginnen
- 📝 Update `.github/copilot-instructions.md`

---

## 📄 ANHANG: Quick Reference

### Import Statement für neue Code

```python
from vyra_base.com import (
    # Blueprints
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    CallbackRegistry,
    
    # Decorators (NEW NAMES!)
    remote_service,        # NOT remote_callable!
    remote_publisher,      # NOT remote_speaker!
    remote_subscriber,     # NOT remote_listener!
    remote_actionServer,   # NOT remote_job!
    
    # Helpers
    bind_decorated_callbacks,
    get_decorated_methods,
    
    # Factory
    InterfaceFactory,
)
```

### Decorator Usage Pattern

```python
class MyComponent:
    @remote_service(name="my_service", namespace="my_module")
    async def my_service(self, request, response=None):
        return {"result": 42}
    
    @remote_publisher(name="status", namespace="my_module")
    async def publish_status(self, message):
        pass
```

### Registry Operations

```python
# Register
CallbackRegistry.register_blueprint(blueprint, namespace="module")

# Bind
CallbackRegistry.bind_callback("name", callback, namespace="module")

# Query
CallbackRegistry.list_unbound(namespace="module")
CallbackRegistry.get_blueprint("name", namespace="module")
CallbackRegistry.debug_print()
```

---

**STATUS:** Bereit für Phase 3 (Entity Integration) 🚀  
**Nächster Entwickler:** Lies "NÄCHSTE SCHRITTE MORGEN" oben ☝️
