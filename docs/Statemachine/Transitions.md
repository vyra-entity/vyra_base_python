
**A) Lifecycle-State-Transitions**

| From           | Event             | To            | Beschreibung                                          |
| -------------- | ----------------- | ------------- | ----------------------------------------------------- |
| Initializing   | init_success      | Active        | Initialisierung erfolgreich abgeschlossen.            |
| Initializing   | init_failure      | Recovering    | Initialisierung fehlgeschlagen, Recovery nötig.       |
| Active         | shutdown          | ShuttingDown  | Kontrolliertes Herunterfahren eingeleitet.            |
| Active         | fault_detected    | Recovering    | Fehler erkannt, Recovery-Prozess wird gestartet.      |
| Recovering     | recovery_success  | Active        | Recovery erfolgreich, zurück zu aktiv.                |
| Recovering     | recovery_failed   | ShuttingDown  | Recovery fehlgeschlagen, Shutdown nötig.              |
| ShuttingDown   | finished          | Offline       | Shutdown abgeschlossen, Modul offline.                |


**B) Operational-State-Transitions**

| From              | Event          | To                | Beschreibung                                                       |
| ----------------- | -------------- | ----------------- | ------------------------------------------------------------------ |
| Idle              | set_ready      | Ready             | Modul bereit für Tasks.                                            |
| Ready             | task_start     | Running           | Task-Ausführung gestartet.                                         |
| Running           | task_pause     | Paused            | Task pausiert, kann fortgesetzt werden.                            |
| Running           | task_stop      | Stopped           | Task gestoppt, Reset erforderlich.                                 |
| Running           | set_background | BackgroundRunning | Wechsel zu Hintergrundverarbeitung.                                |
| BackgroundRunning | set_foreground | Running           | Zurück zur Vordergrund-Task-Ausführung.                            |
| BackgroundRunning | task_pause     | Paused            | Hintergrundverarbeitung pausiert.                                  |
| BackgroundRunning | task_stop      | Stopped           | Hintergrundverarbeitung gestoppt.                                  |
| Paused            | task_resume    | Ready             | Task fortgesetzt, zurück zu Ready.                                 |
| Stopped           | task_reset     | Idle              | Operational-State zurückgesetzt zu Idle.                           |


**C) Health-State-Transitions**

| From     | Event         | To       | Beschreibung                                                        |
| -------- | ------------- | -------- | ------------------------------------------------------------------- |
| Healthy  | warn          | Warning  | Warnung erkannt (nicht-kritisch).                                   |
| Healthy  | fault         | Critical | Kritischer Fehler direkt erkannt.                                   |
| Warning  | clear_warning | Healthy  | Warnung behoben, zurück zu gesund.                                  |
| Warning  | fault         | Critical | Warnung eskaliert zu kritischem Fehler.                             |
| Critical | recover       | Healthy  | Kritischer Fehler behoben, zurück zu gesund.                        |
| Critical | recover       | Warning  | Teilweise Recovery, noch Warnungen aktiv.                           |


**D) Spezielle Cross-Layer Events**

| Event           | Auswirkung                                                                                              |
| --------------- | ------------------------------------------------------------------------------------------------------- |
| interrupt       | Unterbricht aktuelle Operation, kann alle Layer betreffen.                                              |
| emergency_stop  | Not-Aus: Health → Critical, Lifecycle → ShuttingDown, Operational → Stopped (höchste Priorität).       |
| priority_override | Override-Event für spezielle Systemzustände (Admin/Debug).                                           |