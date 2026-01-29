```yaml
module_state_system:
  version: "1.0"
  description: "3-Layer State Machine für VYRA Module"
  standards:
    - IEC 61508
    - IEC 61131-3
    - ISO 13849

  # ============================================
  #           LIFECYCLE LAYER
  # ============================================
  lifecycle:
    description: "Lifecycle states define existence and high-level life of the module."
    
    states:
      - name: Offline
        description: "Modul ist deaktiviert/ausgeschaltet"
        operational_allowed: [Idle, Stopped]
        
      - name: Initializing
        description: "Modul wird initialisiert"
        operational_allowed: []  # Keine operational states erlaubt
        
      - name: Active
        description: "Modul ist voll operational"
        operational_allowed: [Idle, Ready, Running, BackgroundRunning, Paused, Stopped]
        
      - name: Recovering
        description: "Modul führt Recovery durch"
        operational_allowed: [Idle, Paused, Stopped]
        
      - name: ShuttingDown
        description: "Modul wird heruntergefahren"
        operational_allowed: [Idle]

    transitions:
      - from: Offline
        event: START
        to: Initializing
        description: "Modul-Initialisierung starten"

      - from: Initializing
        event: INIT_SUCCESS
        to: Active
        description: "Initialisierung erfolgreich"
        
      - from: Initializing
        event: INIT_FAILURE
        to: Recovering
        description: "Initialisierung fehlgeschlagen"

      - from: Active
        event: SHUTDOWN
        to: ShuttingDown
        description: "Kontrolliertes Herunterfahren"
        
      - from: Active
        event: FAULT_DETECTED
        to: Recovering
        description: "Fehler erkannt, Recovery nötig"

      - from: Recovering
        event: RECOVERY_SUCCESS
        to: Active
        description: "Recovery erfolgreich"
        
      - from: Recovering
        event: RECOVERY_FAILED
        to: ShuttingDown
        description: "Recovery fehlgeschlagen"

      - from: ShuttingDown
        event: FINISHED
        to: Offline
        description: "Shutdown abgeschlossen"

  # ============================================
  #         OPERATIONAL LAYER
  # ============================================
  operational:
    description: "Operational states define the activity of a module during runtime."
    
    states:
      - name: Idle
        description: "Inaktiv, wartet (Resting)"
        
      - name: Ready
        description: "Bereit für Tasks (Attentive)"
        
      - name: Running
        description: "Task wird aktiv ausgeführt (Active)"
        
      - name: BackgroundRunning
        description: "Hintergrundverarbeitung läuft"
        
      - name: Paused
        description: "Task pausiert, kann fortgesetzt werden"
        
      - name: Stopped
        description: "Task gestoppt, Reset erforderlich"

    transitions:
      - from: Idle
        event: SET_READY
        to: Ready
        description: "Bereitschaft für Tasks signalisieren"

      - from: Ready
        event: TASK_START
        to: Running
        description: "Task-Ausführung starten"

      - from: Running
        event: SET_BACKGROUND
        to: BackgroundRunning
        description: "Wechsel zu Hintergrundverarbeitung"
        
      - from: BackgroundRunning
        event: SET_FOREGROUND
        to: Running
        description: "Zurück zur Vordergrund-Ausführung"

      - from: Running
        event: TASK_PAUSE
        to: Paused
        description: "Task pausieren"
        
      - from: BackgroundRunning
        event: TASK_PAUSE
        to: Paused
        description: "Hintergrundverarbeitung pausieren"

      - from: Paused
        event: TASK_RESUME
        to: Ready
        description: "Pausierte Task fortsetzen"

      - from: Running
        event: TASK_STOP
        to: Stopped
        description: "Task stoppen"
        
      - from: BackgroundRunning
        event: TASK_STOP
        to: Stopped
        description: "Hintergrundverarbeitung stoppen"

      - from: Stopped
        event: TASK_RESET
        to: Idle
        description: "Operational state zurücksetzen"

  # ============================================
  #            HEALTH LAYER
  # ============================================
  health:
    description: "Health states describe integrity and error conditions."
    
    states:
      - name: Healthy
        description: "Alles gesund, keine Probleme"
        severity: info
        
      - name: Warning
        description: "Nicht-kritische Warnung (Alert)"
        severity: warning
        
      - name: Critical
        description: "Schwerer Fehler, sofortiges Handeln erforderlich"
        severity: critical

    transitions:
      - from: Healthy
        event: WARN
        to: Warning
        description: "Warnung erkannt"

      - from: Healthy
        event: FAULT
        to: Critical
        description: "Kritischer Fehler direkt erkannt"

      - from: Warning
        event: CLEAR_WARNING
        to: Healthy
        description: "Warnung behoben"

      - from: Warning
        event: FAULT
        to: Critical
        description: "Warnung eskaliert zu kritischem Fehler"

      - from: Critical
        event: RECOVER
        to: Healthy
        description: "Kritischer Fehler vollständig behoben"

      - from: Critical
        event: RECOVER
        to: Warning
        description: "Teilweise Recovery, noch Warnungen aktiv"

  # ============================================
  #         INTERRUPT EVENTS
  # ============================================
  interrupts:
    description: "Cross-layer interrupt events with highest priority"
    
    events:
      - name: INTERRUPT
        description: "Unterbricht aktuelle Operation"
        affects: [lifecycle, operational, health]
        
      - name: EMERGENCY_STOP
        description: "Not-Aus: Health→Critical, Lifecycle→ShuttingDown"
        affects: [lifecycle, operational, health]
        priority: highest
        
      - name: PRIORITY_OVERRIDE
        description: "Admin/Debug override für spezielle Systemzustände"
        affects: [lifecycle, operational, health]
        priority: high

  # ============================================
  #        LAYER INTERACTION RULES
  # ============================================
  interaction_rules:
    lifecycle_to_operational:
      description: "Lifecycle kontrolliert erlaubte Operational states"
      type: control
      
    health_to_lifecycle:
      description: "Health kann Lifecycle eskalieren (FAULT_DETECTED)"
      type: escalation
      events: [FAULT_DETECTED, EMERGENCY_STOP]
      
    health_to_operational:
      description: "Health reguliert Operational behavior"
      type: regulation
      
    operational_to_others:
      description: "Operational darf andere Layer NICHT direkt beeinflussen"
      type: forbidden
      allowed: [event_reporting]  # Nur Events senden erlaubt

  # ============================================
  #         EVENT TYPES SUMMARY
  # ============================================
  event_types:
    lifecycle_events:
      - START
      - INIT_SUCCESS
      - INIT_FAILURE
      - SHUTDOWN
      - FINISHED
      - FAULT_DETECTED
      - RECOVERY_SUCCESS
      - RECOVERY_FAILED
      
    operational_events:
      - SET_READY
      - TASK_START
      - SET_BACKGROUND
      - SET_FOREGROUND
      - TASK_PAUSE
      - TASK_RESUME
      - TASK_COMPLETE
      - TASK_STOP
      - TASK_RESET
      
    health_events:
      - WARN
      - CLEAR_WARNING
      - FAULT
      - RECOVER
      - RESET
      
    interrupt_events:
      - INTERRUPT
      - EMERGENCY_STOP
      - PRIORITY_OVERRIDE
```
