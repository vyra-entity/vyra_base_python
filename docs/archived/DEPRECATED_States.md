
**A) Lifecycle-States (Existenz / hochfahren / runterfahren)**

| State              | Bedeutung                                                                  |
| ------------------ | -------------------------------------------------------------------------- |
| **Initializing**   | Start-Prozess läuft, Modul wird initialisiert.                           |
| **Active**         | Modul ist vollständig operational und kann Aufgaben annehmen.             |
| **Recovering**     | Modul führt Fehlerbehebung/Reset durch, limitierte operationale Zustände. |
| **Suspended**      | Modul ist temporär pausiert (z.B. für Wartung oder Updates).              |
| **ShuttingDown**   | Modul wird kontrolliert heruntergefahren, operationale Zustände frieren ein. |
| **Offline**        | Modul ist deaktiviert/ausgeschaltet, keine operationalen Zustände möglich. |


**B) Operational-States (Aktivitätszustände)**

| State                 | Bedeutung                                                                     |
| --------------------- | ----------------------------------------------------------------------------- |
| **Idle**              | Modul ist inaktiv und wartet (Resting).                                       |
| **Ready**             | Modul ist bereit, Aufträge/Tasks können starten (Attentive).                  |
| **Running**           | Modul führt gerade aktiv einen Task aus (Active).                             |
| **BackgroundRunning** | Modul führt Hintergrundverarbeitung aus (z.B. Analyse, Cleanup).              |
| **Paused**            | Aufgabe wurde pausiert, kann fortgesetzt werden.                              |
| **Stopped**           | Task wurde gestoppt, Modul muss zurück zu Idle reset werden.                  |


**C) Health-States (Funktions- und Fehlerstatus)**

| State        | Bedeutung                                                                           |
| ------------ | ----------------------------------------------------------------------------------- |
| **Healthy**  | Alles gesund, keine Probleme erkannt.                                               |
| **Warning**  | Nicht-kritische Warnung, Modul läuft noch, aber mit leichten Einschränkungen (Alert). |
| **Critical** | Schwere Fehler, sofortiges Handeln erforderlich, kann Lifecycle beeinflussen.        |