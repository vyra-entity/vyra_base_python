# State Machine Dokumentation

Vollständige Dokumentation der 3-Layer State Machine für VYRA Base Python.

---

## 📚 **Dokumentations-Übersicht**

### **1. [3_Layer_Statemachine.md](./3_Layer_Statemachine.md)**
**Hauptdokumentation** - Architektur, Layer-Interaktionen, Regelwerk

Themen:
- Überblick über die 3 Layer (Lifecycle, Operational, Health)
- Layer-Hierarchie und Beeinflussungsregeln
- Lifecycle → Operational Kontrolle
- Health → Lifecycle Eskalation
- Implementierungs-Details
- UnifiedStateMachine API
- Best Practices

**Start hier für:** Gesamtverständnis, Architektur-Übersicht, API-Nutzung

---

### **2. [States.md](./States.md)**
**State-Definitionen** - Alle States der drei Layer

Themen:
- Lifecycle States (Initializing, Active, Recovering, ShuttingDown, Offline)
- Operational States (Idle, Ready, Running, BackgroundRunning, Paused, Stopped)
- Health States (Healthy, Warning, Critical)

**Start hier für:** State-Bedeutungen verstehen, Referenz

---

### **3. [Transitions.md](./Transitions.md)**
**State Transitions** - Erlaubte Übergänge zwischen States

Themen:
- Lifecycle Transitions (mit Events)
- Operational Transitions (mit Events)
- Health Transitions (mit Events)
- Cross-Layer Events (Interrupt, Emergency Stop)

**Start hier für:** Welche Transitions sind möglich, Transition-Regeln

---

### **4. [Events.md](./Events.md)**
**Event System** - Event-driven Architecture

Themen:
- Event-Struktur (StateEvent)
- Event-Typen (Lifecycle, Operational, Health, Interrupt)
- Event Routing und Layer-Zuordnung
- Event Payloads und Beispiele
- Event Callbacks
- Event History & Tracing
- Event-basierte Workflows

**Start hier für:** Events senden, Callbacks nutzen, Event-Handling

---

## 🚀 **Quick Start**

### Minimal Example:

```python
from vyra_base.state.unified import UnifiedStateMachine

# State Machine erstellen
usm = UnifiedStateMachine()

# Modul starten
usm.start()                      # Offline → Initializing
usm.complete_initialization()    # Initializing → Active

# Operational bereit machen
usm.set_ready()                  # Idle → Ready

# Task ausführen
usm.start_task({'task_id': '123'})  # Ready → Running
usm.stop({'result': 'success'})     # Running → Stopped

# States abfragen
states = usm.get_all_states()
print(states)
# {'lifecycle': 'Active', 'operational': 'Stopped', 'health': 'Healthy'}

# Shutdown
usm.shutdown()                   # Active → ShuttingDown
usm.complete_shutdown()          # ShuttingDown → Offline
```

---

## 📖 **Leseempfehlung nach Use-Case**

### **Ich will verstehen, wie das System funktioniert:**
1. [3_Layer_Statemachine.md](./3_Layer_Statemachine.md) - Gesamtarchitektur
2. [States.md](./States.md) - Was bedeuten die einzelnen States
3. [Transitions.md](./Transitions.md) - Wie wechseln States

### **Ich will die State Machine nutzen:**
1. [3_Layer_Statemachine.md](./3_Layer_Statemachine.md) - Section "UnifiedStateMachine"
2. [Events.md](./Events.md) - Events senden und empfangen
3. Code: `src/vyra_base/state/unified.py`

### **Ich will Events verstehen:**
1. [Events.md](./Events.md) - Komplette Event-Dokumentation
2. [Transitions.md](./Transitions.md) - Welche Events welche Transitions auslösen
3. Code: `src/vyra_base/state/state_events.py`

### **Ich will eigene Layer-Logik implementieren:**
1. [3_Layer_Statemachine.md](./3_Layer_Statemachine.md) - Layer-Regeln
2. Code: `src/vyra_base/state/lifecycle_layer.py`
3. Code: `src/vyra_base/state/operational_layer.py`
4. Code: `src/vyra_base/state/health_layer.py`

### **Ich will testen/debuggen:**
1. [Events.md](./Events.md) - Section "Event History & Tracing"
2. [3_Layer_Statemachine.md](./3_Layer_Statemachine.md) - Section "Diagnostics"
3. Code: `tests/test_state_machine.py`

---

## 🏗️ **Architektur auf einen Blick**

```
┌─────────────────────────────────────────┐
│      UnifiedStateMachine (API)          │  ← Empfohlener Entry Point
├─────────────────────────────────────────┤
│  LifecycleLayer │ OperationalLayer │ HealthLayer │
├─────────────────────────────────────────┤
│         StateMachine (Core FSM)         │  ← Event-Handler, Validation
├─────────────────────────────────────────┤
│     States │ Transitions │ Events       │  ← Type Definitions
└─────────────────────────────────────────┘
```

### Layer-Beziehungen:

```
Lifecycle
    ↓ controls
Operational
    ↑ reports     ↓ regulates
        Health
```

---

## 📦 **Code-Struktur**

```
src/vyra_base/state/
├── __init__.py                # Public API exports
├── state_machine.py           # Core FSM engine
├── state_types.py             # State enums and validation
├── state_events.py            # Event definitions
├── lifecycle_layer.py         # Lifecycle API
├── operational_layer.py       # Operational API
├── health_layer.py            # Health API
└── unified.py                 # Unified high-level API ⭐
```

---

## 🔍 **Wichtige Konzepte**

### **1. Event-Driven**
Alle State-Changes erfolgen durch Events, nie direkt.

### **2. Layer-Hierarchie**
- Lifecycle steuert Operational
- Health kann Lifecycle eskalieren
- Operational darf andere Layer NICHT beeinflussen

### **3. Thread-Safe**
Alle Operationen sind thread-safe (RLock).

### **4. Immutable Events**
Events sind unveränderlich (frozen dataclass).

### **5. History & Tracing**
Alle Transitions werden für Audit protokolliert.

### **6. Validation**
Transitions werden automatisch validiert (strict mode).

---

## 🎯 **Standards und Compliance**

Die State Machine folgt industriellen Standards:

- **IEC 61508** - Functional Safety
- **IEC 61131-3** - PLC Programming
- **ISO 13849** - Safety of Machinery
- **ROS2 Lifecycle** - Node Management Pattern

---

## 📝 **Zusätzliche Ressourcen**

### Code-Beispiele:
- `tests/test_state_machine.py` - Unit tests mit Beispielen
- `tests/test_unified.py` - UnifiedStateMachine tests
- `examples/` (falls vorhanden) - Vollständige Beispiele

### API-Referenz:
- Docstrings in allen Modulen
- Type hints für alle öffentlichen APIs
- Logging für Debugging

---

## 🆘 **Troubleshooting**

### Problem: InvalidTransitionError
**Lösung:** Siehe [Transitions.md](./Transitions.md) für erlaubte Transitions

### Problem: LayerViolationError
**Lösung:** Siehe [3_Layer_Statemachine.md](./3_Layer_Statemachine.md) für Layer-Regeln

### Problem: Events werden nicht verarbeitet
**Lösung:** Check Event-Routing in [Events.md](./Events.md)

### Problem: Callbacks werden nicht aufgerufen
**Lösung:** Check Callback-Registrierung in [Events.md](./Events.md)

---

## 📧 **Feedback & Contributions**

Fragen oder Verbesserungsvorschläge? Siehe:
- Issues im Repository
- CONTRIBUTING.md (falls vorhanden)
- Team-Dokumentation

---

**Letzte Aktualisierung:** 2024-12-16
**Version:** vyra_base_python v0.x.x
