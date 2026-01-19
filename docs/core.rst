Kernkomponenten (Core)
======================

Das Core-Modul bildet das Herzstück des VYRA-Frameworks. Hier werden alle zentralen Komponenten initialisiert und verwaltet.

.. toctree::
   :maxdepth: 2
   :caption: Core Komponenten

   core/entity
   core/parameter
   core/volatile

Übersicht
---------

Das Core-Modul besteht aus drei Hauptkomponenten:

* **Entity**: Zentrale Verwaltungseinheit für Module
* **Parameter**: Persistente Konfigurationsdaten in SQLite
* **Volatile**: Flüchtige, schnelle Datenspeicherung mit Redis

Zusammenspiel der Komponenten
------------------------------

Die drei Komponenten arbeiten eng zusammen:

1. **VyraEntity** initialisiert und orchestriert alle Module-Komponenten
2. **Parameter** verwalten persistente Konfigurationen (langsam, dauerhaft)
3. **Volatile** verwalten flüchtige Echtzeitdaten (schnell, temporär)

.. tip::
   Verwenden Sie Parameter für Konfigurationsdaten, die zwischen Neustarts erhalten bleiben sollen.
   Verwenden Sie Volatiles für Echtzeitdaten und schnelle Zwischenspeicherung.

Weiterführende Informationen
-----------------------------

* :doc:`core/entity` - Zentrale Entity-Verwaltung
* :doc:`core/parameter` - Persistente Parameter
* :doc:`core/volatile` - Flüchtige Daten
* :doc:`vyra_base.core` - API-Referenz (automatisch generiert)
