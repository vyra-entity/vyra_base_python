# **3-Layer State Machine - VYRA Base Python**

Die State Machine folgt industriellen Standards (IEC 61508, IEC 61131-3, ISO 13849) und besteht aus drei hierarchischen Layern:

- **Lifecycle-States** (Existenz / hochfahren / runterfahren)
- **Operational-States** (Aktivitätszustände)
- **Health-States** (Funktions- und Fehlerstatus)

# **Kurzüberblick: Die drei Layer**

1. **Lifecycle Layer**  
    _Wann_ ein Modul existiert, initialisiert, aktiviert oder heruntergefahren wird.  
    → entspricht Node-Lifecycle in ROS2, ECS Lifecycle in Unreal, Driver-Lifecycle in Linux.
    
2. **Operational Layer**  
    _Was_ das Modul gerade tut.  
    → Laufzeitverhalten (Idle, Ready, Running, Paused, Stopped …)
    
3. **Health Layer**  
    _Wie gut_ das Modul funktioniert.  
    → Diagnose, Healthy, Warning, Critical
    

Sie sind **nicht parallel**, sondern **ineinander verschachtelt**:

```
Lifecycle steuert Operational
Operational wird durch Health reguliert
Health kann Lifecycle überschreiben (Eskalation)
```

#### Jeder Layer darf nur eine Richtung beeinflussen:

| Quelle         | Darf beeinflussen                | Darf NICHT beeinflussen |
| -------------- | -------------------------------- | ----------------------- |
| **Lifecycle**  | Operational + Health             | —                       |
| **Operational**| —                                | Lifecycle, Health       |
| **Health**     | Lifecycle (hart) + Operational (weich) | —                       |

---

# Lifecycle → Operational

Der Lifecycle legt fest, **welche Operational-States überhaupt erlaubt sind**.

```
Initializing: keine Operational-Zustände erlaubt (FSM gesperrt)
Active: Operational FSM voll aktiv (alle Zustände erlaubt)
Recovering: Operational FSM pausiert / limitiert (nur Idle, Paused, Stopped)
ShuttingDown: Operational FSM wird eingefroren (nur Idle erlaubt)
Offline: Operational nur Idle oder Stopped
```

### Regeln aus Code (`LIFECYCLE_OPERATIONAL_RULES`):

```python
LifecycleState.INITIALIZING: set()  # Keine operational states
LifecycleState.ACTIVE: {            # Alle operational states erlaubt
    IDLE, READY, RUNNING, BACKGROUND_RUNNING, PAUSED, STOPPED
}
LifecycleState.RECOVERING: {        # Limitierte states
    IDLE, PAUSED, STOPPED
}
LifecycleState.SHUTTING_DOWN: {     # Nur Idle
    IDLE
}
LifecycleState.OFFLINE: {           # Nur Idle/Stopped
    IDLE, STOPPED
}
```

---

# Health → Operational

Health wirkt wie ein **Regulationslayer**, der Operational korrigiert.

### Regeln:

- **Warning** → Operational bleibt aktiv, aber mit Einschränkungen (z.B. reduzierte Last)
- **Critical** → Operational **wird sofort unterbrochen**, springt zu Stopped

> Die konkrete Umsetzung erfolgt über Event-Handler in der State Machine.

---

# Health → Lifecycle

Nur Health darf den Lifecycle **von unten nach oben eskalieren**:

- **Warning** → Lifecycle bleibt Active (nur Monitoring)
- **Critical** + fault → Lifecycle springt von Active → Recovering
- **Critical** + emergency_stop → Lifecycle → ShuttingDown (Not-Aus)

> „Safety health states override everything."

### Events für Eskalation:

```python
EventType.FAULT_DETECTED    # Health Critical → Lifecycle Recovering
EventType.EMERGENCY_STOP    # Highest priority → Lifecycle ShuttingDown
```

---

# Operational darf die anderen Layer NICHT beeinflussen

Operational darf nur Events nach oben melden, aber nicht selbst springen.

**Beispiel:**

```
Running erkennt einen Fehler 
  → event fault_detected gesendet
  → NICHT selbst lifecycle wechseln
  → sondern Health FSM übernimmt
  → Health setzt Critical
  → Critical triggert Lifecycle → Recovering
```

---

# **Diagramm der Verbindungen**

```lua
           +----------------+
           |   Lifecycle    |
           +----------------+
             |          ▲
    controls |          | escalates (fault_detected)
             ▼          |
      +----------------------+
      |     Operational      |
      +----------------------+
             ▲          |
   regulates |          | reports (warning/fault)
             |          ▼
       +-------------------+
       |       Health      |
       +-------------------+
```

---

# 🧬 **Konkretes Regelwerk (Best-Practice-Set)**

Hier das verbindende Regelwerk, wie es z. B. AWS, ABB, ROS2 Lifecycle nutzen:

## **Lifecycle → Operational Regeln**

```python
if lifecycle in [Initializing, Offline]:
  operational_allowed = set() or {IDLE, STOPPED}

if lifecycle == Active:
  operational_allowed = all_standard_states

if lifecycle == Recovering:
  operational_allowed = {IDLE, PAUSED, STOPPED}

if lifecycle == ShuttingDown:
  operational_target = IDLE
```

---

## **Health → Operational Regeln**

```python
if health == Warning:
  operational_constraints = limited

if health == Critical:
  operational_target = Stopped
```

---

## **Health → Lifecycle Regeln**

```python
if health == Critical and event == FAULT_DETECTED:
  lifecycle_target = Recovering

if health == Critical and event == EMERGENCY_STOP:
  lifecycle_target = ShuttingDown
```

---

# **Implementierung Details**

## State Machine Core

Die State Machine (`state_machine.py`) ist **thread-safe** (RLock) und verwendet **Event-driven Architecture**.

### Event-Typen (aus `state_events.py`):

#### Lifecycle Events:
- `START` - Modul starten
- `INIT_SUCCESS` - Initialisierung erfolgreich
- `INIT_FAILURE` - Initialisierung fehlgeschlagen
- `SHUTDOWN` - Herunterfahren
- `FINISHED` - Offline gehen
- `FAULT_DETECTED` - Fehler erkannt
- `RECOVERY_SUCCESS` - Recovery erfolgreich
- `RECOVERY_FAILED` - Recovery fehlgeschlagen

#### Operational Events:
- `SET_READY` - Ready state setzen
- `TASK_START` - Task starten
- `SET_BACKGROUND` - Background processing
- `SET_FOREGROUND` - Foreground processing
- `TASK_PAUSE` - Task pausieren
- `TASK_RESUME` - Task fortsetzen
- `TASK_STOP` - Task stoppen
- `TASK_RESET` - Operational reset

#### Health Events:
- `WARN` - Warnung melden
- `CLEAR_WARNING` - Warnung löschen
- `FAULT` - Fehler melden
- `RECOVER` - Recovery versuchen
- `RESET` - Health reset

#### Interrupt Events (Cross-Layer):
- `INTERRUPT` - System interrupt
- `EMERGENCY_STOP` - Not-Aus
- `PRIORITY_OVERRIDE` - Admin override

---

## Layer APIs

### LifecycleLayer (`lifecycle_layer.py`)

```python
lifecycle = LifecycleLayer(fsm)
lifecycle.start()                    # Offline → Initializing
lifecycle.complete_initialization()  # Initializing → Active
lifecycle.fail_initialization()      # Initializing → Recovering
lifecycle.shutdown()                 # Active → ShuttingDown
lifecycle.complete_shutdown()        # ShuttingDown → Offline
```

### OperationalLayer (`operational_layer.py`)

```python
operational = OperationalLayer(fsm)
operational.set_ready()              # Idle → Ready
operational.start_task(task_info)    # Ready → Running
operational.pause()                  # Running → Paused
operational.resume()                 # Paused → Ready
operational.stop()                   # Running → Stopped
operational.reset()                  # Stopped → Idle
```

### HealthLayer (`health_layer.py`)

```python
health = HealthLayer(fsm)
health.report_warning(warning_info)  # Healthy → Warning
health.clear_warning()               # Warning → Healthy
health.report_fault(fault_info)      # Any → Critical
health.recover()                     # Critical → Healthy/Warning
health.emergency_stop(reason)        # Critical → Lifecycle shutdown
```

---

## Unified State Machine

Die **UnifiedStateMachine** (`unified.py`) kombiniert alle drei Layer in einer einfachen API:

```python
from vyra_base.state.unified import UnifiedStateMachine

# Initialisierung
usm = UnifiedStateMachine()

# Standard Startup
usm.standard_startup()  # Initializing → Active → Ready

# Task ausführen
usm.start_task({'task_id': '123'})
usm.stop({'result': 'success'})

# Warnung melden
usm.report_warning({'cpu': '85%'})

# States abfragen
states = usm.get_all_states()
# {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Warning'}

# Callbacks registrieren
usm.on_lifecycle_change(my_callback)
usm.on_operational_change(my_callback)
usm.on_health_change(my_callback)

# Standard Shutdown
usm.standard_shutdown()  # ShuttingDown → Offline
```

---

## Transition History & Diagnostics

```python
# History abrufen
history = usm.get_history(limit=10)

# Diagnostic Info
diag = usm.get_diagnostic_info()
# {
#   "states": {...},
#   "lifecycle_info": {...},
#   "operational_info": {...},
#   "health_info": {...},
#   "fsm_diagnostics": {...}
# }
```

---

# **Best Practices**

1. **Immer UnifiedStateMachine verwenden** für Standard-Use-Cases
2. **Layer-APIs direkt** nur für spezielle Anforderungen nutzen
3. **Events** statt direkter State-Änderungen für Traceability
4. **Callbacks** für State-Change-Reaktionen registrieren
5. **Diagnostic Info** für Debugging und Monitoring nutzen
6. **Strict Mode** in Production für Fehler-Erkennung aktivieren
7. **History** für Audit-Logs und Post-Mortem-Analyse nutzen

---

# **Siehe auch**

- `States.md` - Detaillierte State-Definitionen
- `Transitions.md` - Alle erlaubten Transitions
- `src/vyra_base/state/` - Implementierung
- `tests/` - Test-Cases und Beispiele
