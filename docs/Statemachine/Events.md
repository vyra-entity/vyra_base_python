# **Events - State Machine Event System**

Die State Machine verwendet ein **Event-driven Architecture** Pattern. Alle State-Transitions werden durch Events ausgelöst.

---

## **Event-Struktur**

Jedes Event ist eine immutable `StateEvent` Instanz:

```python
@dataclass(frozen=True)
class StateEvent:
    event_type: EventType              # Art des Events
    timestamp: datetime                # Zeitstempel
    origin_layer: Optional[str]        # Ursprungs-Layer (lifecycle/operational/health)
    payload: Optional[Dict[str, Any]]  # Event-spezifische Daten
    event_id: Optional[str]            # Eindeutige Event-ID
```

### Beispiel:

```python
from vyra_base.state.state_events import StateEvent, EventType

event = StateEvent(
    event_type=EventType.TASK_START,
    origin_layer="operational",
    payload={"task_id": "task_123", "priority": "high"}
)
```

---

## **Event-Typen Übersicht**

### **A) Lifecycle Events**

| Event             | Von → Nach               | Beschreibung                            |
| ----------------- | ------------------------ | --------------------------------------- |
| START             | Offline → Initializing   | Modul-Initialisierung starten           |
| INIT_SUCCESS      | Initializing → Active    | Initialisierung erfolgreich             |
| INIT_FAILURE      | Initializing → Recovering| Initialisierung fehlgeschlagen          |
| SHUTDOWN          | Active → ShuttingDown    | Kontrolliertes Herunterfahren           |
| FINISHED          | ShuttingDown → Offline   | Shutdown abgeschlossen                  |
| FAULT_DETECTED    | Active → Recovering      | Fehler erkannt, Recovery nötig          |
| RECOVERY_SUCCESS  | Recovering → Active      | Recovery erfolgreich                    |
| RECOVERY_FAILED   | Recovering → ShuttingDown| Recovery fehlgeschlagen                 |

### **B) Operational Events**

| Event          | Von → Nach                | Beschreibung                            |
| -------------- | ------------------------- | --------------------------------------- |
| SET_READY      | Idle → Ready              | Bereitschaft für Tasks signalisieren    |
| TASK_START     | Ready → Running           | Task-Ausführung starten                 |
| SET_BACKGROUND | Running → BackgroundRunning| Wechsel zu Hintergrundverarbeitung     |
| SET_FOREGROUND | BackgroundRunning → Running| Zurück zur Vordergrund-Ausführung      |
| TASK_PAUSE     | Running → Paused          | Task pausieren                          |
| TASK_RESUME    | Paused → Ready            | Pausierte Task fortsetzen               |
| TASK_COMPLETE  | Running → Ready           | Task erfolgreich abgeschlossen          |
| TASK_STOP      | Running → Stopped         | Task stoppen (requires reset)           |
| TASK_RESET     | Stopped → Idle            | Operational state zurücksetzen          |

### **C) Health Events**

| Event         | Von → Nach              | Beschreibung                            |
| ------------- | ----------------------- | --------------------------------------- |
| WARN          | Healthy → Warning       | Nicht-kritische Warnung melden          |
| CLEAR_WARNING | Warning → Healthy       | Warnung behoben                         |
| FAULT         | Any → Critical          | Kritischer Fehler erkannt               |
| RECOVER       | Critical → Healthy/Warning | Recovery von kritischem Fehler       |
| RESET         | Any → Healthy           | Health state zurücksetzen               |

### **D) Interrupt Events (Cross-Layer)**

| Event             | Auswirkung                                                                    |
| ----------------- | ----------------------------------------------------------------------------- |
| INTERRUPT         | Unterbricht aktuelle Operation, kann alle Layer betreffen                     |
| EMERGENCY_STOP    | Not-Aus: Health→Warning, Operational→Stopped         |
| PRIORITY_OVERRIDE | Admin/Debug override für spezielle Systemzustände                             |

---

## **Event Routing**

Events werden automatisch zum zuständigen Layer geroutet:

```python
EVENT_LAYER_MAP = {
    EventType.START: "lifecycle",
    EventType.TASK_START: "operational",
    EventType.WARN: "health",
    EventType.EMERGENCY_STOP: "interrupt",  # Cross-layer
    # ...
}
```

### Interrupt Events (höchste Priorität):

Interrupt-Events werden **vor allen anderen** verarbeitet und können alle Layer gleichzeitig beeinflussen.

---

## **Event Senden**

### Mit StateMachine:

```python
from vyra_base.state import StateMachine, StateEvent, EventType

fsm = StateMachine()
event = StateEvent(EventType.START)
new_state = fsm.send_event(event)
```

### Mit UnifiedStateMachine (empfohlen):

```python
from vyra_base.state.unified import UnifiedStateMachine

usm = UnifiedStateMachine()
usm.start()  # Sendet intern START event
```

---

## **Event Callbacks**

Callbacks können registriert werden, um auf State-Changes zu reagieren:

```python
def on_lifecycle_change(event, old_state, new_state):
    print(f"Lifecycle: {old_state} → {new_state} (event: {event})")

usm.on_lifecycle_change(on_lifecycle_change)
usm.on_operational_change(on_operational_change)
usm.on_health_change(on_health_change)
usm.on_any_change(on_any_change)  # Alle Layer
```

### Callback-Signatur:

```python
def callback(
    event: StateEvent,
    old_state: State,
    new_state: State
) -> None:
    ...
```

---

## **Event Payload Beispiele**

### Task Start mit Kontext:

```python
event = StateEvent(
    event_type=EventType.TASK_START,
    payload={
        "task_id": "task_123",
        "priority": "high",
        "estimated_duration": 5.0,
        "resources": ["cpu", "memory"]
    }
)
```

### Fault Detection mit Error-Info:

```python
event = StateEvent(
    event_type=EventType.FAULT_DETECTED,
    payload={
        "error_code": "ERR_CONN_LOST",
        "error_message": "Connection to service X lost",
        "severity": "critical",
        "affected_components": ["network", "ros2_client"]
    }
)
```

### Warning mit Metrics:

```python
event = StateEvent(
    event_type=EventType.WARN,
    payload={
        "metric": "cpu_usage",
        "current_value": 85.0,
        "threshold": 80.0,
        "unit": "percent"
    }
)
```

---

## **Event History & Tracing**

Die State Machine speichert alle Events für Debugging und Audit:

```python
# Letzte 10 Transitions abrufen
history = fsm.get_history(limit=10)

for transition in history:
    print(f"{transition.timestamp}: {transition.from_state} → {transition.to_state}")
    print(f"  Event: {transition.event}")
    print(f"  Layer: {transition.layer}")
```

### Transition Record:

```python
@dataclass
class StateTransition:
    from_state: str
    to_state: str
    layer: str
    event: StateEvent
    timestamp: datetime
    success: bool
    error_message: Optional[str]
```

---

## **Event-basierte Workflows**

### Standard Startup Sequence:

```python
# Event 1: START
fsm.send_event(StateEvent(EventType.START))
# Offline → Initializing

# Event 2: INIT_SUCCESS
fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
# Initializing → Active

# Event 3: SET_READY
fsm.send_event(StateEvent(EventType.SET_READY))
# Operational: Idle → Ready
```

### Task Execution:

```python
# Start
fsm.send_event(StateEvent(EventType.TASK_START, payload={"task_id": "123"}))

# Pause
fsm.send_event(StateEvent(EventType.TASK_PAUSE))

# Resume
fsm.send_event(StateEvent(EventType.TASK_RESUME))

# Complete
fsm.send_event(StateEvent(EventType.TASK_COMPLETE, payload={"result": "success"}))
```

### Error Handling:

```python
# Fehler erkennen
fsm.send_event(StateEvent(
    EventType.FAULT_DETECTED,
    payload={"error": "Connection lost"}
))
# Lifecycle: Active → Recovering

# Recovery erfolgreich
fsm.send_event(StateEvent(EventType.RECOVERY_SUCCESS))
# Lifecycle: Recovering → Active
```

---

## **Event Validation**

Die State Machine validiert Events automatisch:

### Invalid Transition:

```python
# Wenn Lifecycle = Offline, kann kein TASK_START ausgeführt werden
event = StateEvent(EventType.TASK_START)
try:
    fsm.send_event(event)
except InvalidTransitionError as e:
    print(f"Transition nicht erlaubt: {e}")
```

### Layer Violation:

```python
# Operational darf nicht direkt Lifecycle beeinflussen
# Dies wird durch das Event-System verhindert
```

---

## **Best Practices**

1. **Payload nutzen**: Immer relevante Kontext-Informationen mitgeben
2. **Event-IDs**: Automatisch generiert für Tracing
3. **Origin-Layer**: Hilft bei der Fehlersuche
4. **Callbacks**: Für reaktive Logik nutzen statt Polling
5. **History**: Für Post-Mortem-Analyse aufbewahren
6. **Interrupt-Events**: Nur für echte Notfälle (Emergency Stop)
7. **Strict Mode**: In Production aktivieren für Event-Validation

---

## **Custom Events**

Für modulspezifische Events kann der EventType erweitert werden:

```python
from vyra_base.state.state_events import EventType

# Custom event type
class ModuleEventType(EventType):
    MODULE_SPECIFIC = "module_specific"
    CUSTOM_ACTION = "custom_action"
```

**Hinweis**: Custom events sollten sparsam verwendet werden. Die Standard-Events decken die meisten Use-Cases ab.

---

## **Event Flow Diagramm**

```
User/Module
    |
    | send_event(event)
    ▼
+------------------+
| StateMachine     |
+------------------+
    |
    | 1. Validate event
    | 2. Route to layer
    |
    ▼
+------------------+
| Layer Handler    | (lifecycle/operational/health)
+------------------+
    |
    | 3. Validate transition
    | 4. Execute transition
    |
    ▼
+------------------+
| Update State     |
+------------------+
    |
    | 5. Record in history
    | 6. Notify callbacks
    |
    ▼
+------------------+
| Callbacks        |
+------------------+
```

---

## **Siehe auch**

- `3_Layer_Statemachine.md` - Layer-Architektur
- `States.md` - State-Definitionen
- `Transitions.md` - Erlaubte Transitions
- `state_events.py` - Event-Implementierung
