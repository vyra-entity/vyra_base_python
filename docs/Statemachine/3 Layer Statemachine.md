- Lifecycle-States (Existenz / hochfahren / runterfahren)
- Operational-States (Aktivitätszustände)
- Health-States (Funktions- und Fehlerstatus)

# **Kurzüberblick: Die drei Layer**

1. **Lifecycle Layer**  
    _Wann_ ein Modul existiert, initialisiert, aktiviert oder heruntergefahren wird.  
    → entspricht Node-Lifecycle in ROS2, ECS Lifecycle in Unreal, Driver-Lifecycle in Linux.
    
2. **Operational Layer**  
    _Was_ das Modul gerade tut.  
    → Laufzeitverhalten (Idle, Running, Paused, Blocked …)
    
3. **Health Layer**  
    _Wie gut_ das Modul funktioniert.  
    → Diagnose, Faults, Overload, Critical
    

Sie sind **nicht parallel**, sondern **ineinander verschachtelt**:

`Lifecycle steuert Operational Operational wird durch Health reguliert Health kann Lifecycle überschreiben`

#### Jeder Layer darf nur eine Richtung beeinflussen:

|Quelle|Darf beeinflussen|Darf NICHT beeinflussen|
|---|---|---|
|**Lifecycle**|Operational + Health|—|
|**Operational**|—|Lifecycle|
|**Health**|Lifecycle (hart) + Operational (weich)|—|

# Lifecycle → Operational

Der Lifecycle legt fest, **welche Operational-States überhaupt erlaubt sind**.

`Uninitialized: keine Operational-Zustände möglich Initializing: Operational FSM gesperrt Active: Operational FSM voll aktiv Recovering: Operational FSM pausiert / limitiert ShuttingDown: Operational FSM wird eingefroren Deactivated: Operational FSM deaktiviert`


# Health → Operational

Health wirkt wie ein **Regulationslayer**, der Operational korrigiert.

### Regeln:

- **Warning** → Operational bleibt aktiv, aber mit Einschränkungen
- **Overloaded** → Operational springt zwangsweise in „Paused“ oder „Processing“
- **Faulted** → Operational wird gestoppt und blockiert
- **Critical** → Operational **wird sofort unterbrochen**, egal was gerade läuft


# Health → Lifecycle

Nur Health darf den Lifecycle **von unten nach oben wärts kicken**, z. B.:
- Faulted → Lifecycle springt von Active → Recovering
- Critical → Lifecycle springt nach Active → ShuttingDown → Deactivated
- Critical (Emergency) → Lifecycle → Deactivated sofort (Not-Aus)

> „Safety health states override everything.“


# Operational darf die anderen Layer NICHT beeinflussen

Operational darf nur Events nach oben melden, aber nicht selbst springen.

Beispiel:

`Running erkennt einen Fehler → event fault_detected → aber NICHT selbst lifecycle wechseln → sondern Health FSM übernimmt → Health setzt z.B. Faulted → Faulted triggert Lifecycle → Recovering`


# **Diagramm der Verbindungen (Textversion)**

```lua


           +----------------+
           |   Lifecycle    |
           +----------------+
             |          ▲
    controls |          | escalates
             ▼          |
      +----------------------+
      |     Operational      |
      +----------------------+
             ▲          |
   regulates |          | not allowed
             |          ▼
       +-------------------+
       |       Health      |
       +-------------------+
```

# 🧬 **Konkretes Regelwerk (Best-Practice-Set)**

Hier das verbindende Regelwerk, wie es z. B. AWS, ABB, ROS2 Lifecycle nutzen:

---

## **Lifecycle → Operational Regeln**

```yaml
if lifecycle in [Uninitialized, Initializing, Deactivated]:
  operational = null

if lifecycle == Active:
  allow_operational = all_standard_states

if lifecycle == Recovering:
  operational = Paused or Blocked

if lifecycle == ShuttingDown:
  operational = Paused until idle

```
---

## **Health → Operational Regeln**

```yaml
if health == Warning:
  operational_constraints = limited

if health == Overloaded:
  operational_target = Paused

if health == Faulted:
  operational_target = Blocked

if health == Critical:
  operational_target = Interrupt
```
---

## **Health → Lifecycle Regeln**

```yaml
if health == Faulted:
  lifecycle_target = Recovering

if health == Critical:
  lifecycle_target = ShuttingDown or Deactivated
```