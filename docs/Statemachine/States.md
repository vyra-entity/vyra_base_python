
**A) Lifecycle-States (Existenz / hochfahren / runterfahren)**

| State                        | Bedeutung                                  |
| ---------------------------- | ------------------------------------------ |
| **Uninitialized**            | Modul ist angelegt, aber noch nicht aktiv. |
| **Initializing** (Awakening) | Start-Prozess läuft.                       |
| **Active**                   | Modul ist gestartet und operational.       |
| **Recovering**               | Modul führt Fehlerbehebung/Reset durch.    |
| **ShuttingDown**             | Modul wird kontrolliert heruntergefahren.  |
| **Deactivated**              | Modul ist ausgeschaltet oder deaktiviert.  |
|                              |                                            |


**B) Operational-States (Aktivitätszustände)**

|State|Bedeutung|
|---|---|
|**Idle/Resting**|(Resting) Modul tut nichts und wartet.|
|**Ready/Attentive**|(Attentive) Modul ist bereitet, Aufträge können starten.|
|**Running/Active**|(Active) Modul führt gerade eine Aufgabe aus.|
|**Processing/Reflecting**|(Reflecting) Hintergrundverarbeitung / Analyse.|
|**Delegating**|(Delegating) Modul wartet auf Ergebnis anderer Module.|
|**Paused**|Aufgabe wurde pausiert.|
|**Blocked**|Modul wartet, weil Ressourcen fehlen / Lock aktiv.|
|**Completed**|Aufgabe fertig → kehrt später zu Ready oder Idle zurück.|

**C) Health-States (Funktions- und Fehlerstatus)**

|State|Bedeutung|
|---|---|
|**OK**|Alles gesund.|
|**Warning / Alert**|(Alert) Modul ist aufmerksam / leichte Störung / Vorwarnung.|
|**Overloaded**|(Overloaded) Modul ist überlastet.|
|**Faulted**|Modul befindet sich im Fehlerzustand.|
|**Critical**|Schwere Fehler; keine Operation möglich.|