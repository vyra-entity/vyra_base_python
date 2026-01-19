Storage - Datenspeicherung
==========================

Das Storage-Modul bietet Zugriff auf persistente (SQLite) und flüchtige (Redis) Datenspeicherung.

Übersicht
---------

VYRA unterstützt zwei Storage-Backends:

1. **SQLite-Datenbank**: Persistente Speicherung (dauerhaft)
2. **Redis**: Flüchtige In-Memory-Speicherung (schnell)

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Backend
     - Anwendungsfall
     - Zugriff über
   * - **SQLite**
     - Parameter, Konfiguration, Logs
     - :class:`~vyra_base.storage.db_access.DbAccess`
   * - **Redis**
     - Volatiles, Echtzeitdaten, Caching
     - :class:`~vyra_base.storage.redis_client.RedisClient`

Datenbank-Zugriff (SQLite)
---------------------------

DbAccess-Klasse
^^^^^^^^^^^^^^^

Die :class:`~vyra_base.storage.db_access.DbAccess`-Klasse verwaltet SQLite-Datenbanken:

.. code-block:: python

   from vyra_base.storage.db_access import DbAccess, DBTYPE
   
   # Datenbank-Zugriff erstellen
   db = DbAccess(
       db_type=DBTYPE.SQLITE,
       db_path="/workspace/storage/data/module.db"
   )
   
   # Verbindung initialisieren
   await db.initialize_connection()
   
   # Tabellen erstellen
   await db.create_all_tables()

**Hauptmethoden:**

* ``initialize_connection()``: Verbindung aufbauen
* ``create_all_tables()``: Alle Tabellen erstellen
* ``create_selected_table(tables)``: Spezifische Tabellen erstellen
* ``drop_table(table)``: Tabelle löschen
* ``check_table_exists(table)``: Tabellen-Existenz prüfen
* ``session``: Session-Maker für Abfragen

Tabellen-Definition
-------------------

Namenskonvention
^^^^^^^^^^^^^^^^

**Alle Datenbank-Tabellen müssen mit** ``tb_`` **beginnen!**

.. code-block:: python

   # ✅ Richtig
   class tb_parameters(Base):
       pass
   
   class tb_logs(Base):
       pass
   
   # ❌ Falsch
   class parameters(Base):  # Fehlt tb_ Prefix!
       pass

Tabelle erstellen
^^^^^^^^^^^^^^^^^

.. code-block:: python

   from sqlalchemy.orm import Mapped, mapped_column
   from sqlalchemy import String, Integer, DateTime
   from vyra_base.storage import Base
   import uuid
   from datetime import datetime
   
   class tb_sensor_data(Base):
       """Tabelle für Sensor-Daten"""
       __tablename__ = 'tb_sensor_data'
       
       # Primary Key (UUID empfohlen)
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       
       # Datenfelder
       sensor_name: Mapped[str] = mapped_column(String(100), nullable=False)
       value: Mapped[float] = mapped_column(nullable=False)
       unit: Mapped[str] = mapped_column(String(20))
       timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
       
       # Optional: String-Repräsentation
       def __repr__(self):
           return f"<SensorData(sensor={self.sensor_name}, value={self.value})>"

**Feldtypen:**

* ``String(length)``: Text mit fester Länge
* ``Integer``: Ganzzahl
* ``Float``: Gleitkommazahl
* ``Boolean``: True/False
* ``DateTime``: Zeitstempel
* ``UUID``: Unique Identifier (empfohlen für IDs)

Beispiel-Tabellen
^^^^^^^^^^^^^^^^^

**Parameter-Tabelle**:

.. code-block:: python

   class tb_parameters(Base):
       __tablename__ = 'tb_parameters'
       
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       name: Mapped[str] = mapped_column(String(100), unique=True, nullable=False)
       value: Mapped[str] = mapped_column(String(500))
       description: Mapped[str] = mapped_column(String(500))
       updated_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

**Log-Tabelle**:

.. code-block:: python

   class tb_logs(Base):
       __tablename__ = 'tb_logs'
       
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       level: Mapped[str] = mapped_column(String(20))  # INFO, WARNING, ERROR
       message: Mapped[str] = mapped_column(String(1000))
       module: Mapped[str] = mapped_column(String(100))
       timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

Datenbank-Operationen
---------------------

DbManipulator-Klasse
^^^^^^^^^^^^^^^^^^^^

Die :class:`~vyra_base.storage.db_manipulator.DbManipulator`-Klasse vereinfacht CRUD-Operationen:

.. code-block:: python

   from vyra_base.storage.db_manipulator import DbManipulator
   
   # Manipulator für eine Tabelle erstellen
   manipulator = DbManipulator(
       db_access=db,
       table_structure=tb_sensor_data
   )

Daten einfügen (Create)
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Einzelner Eintrag
   result = await manipulator.insert({
       "sensor_name": "temperature_1",
       "value": 23.5,
       "unit": "°C"
   })
   
   if result.success:
       print(f"Eingefügt mit ID: {result.details['id']}")

Daten lesen (Read)
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach ID
   result = await manipulator.get_by_id(sensor_id)
   if result.success:
       sensor = result.details
       print(f"Sensor: {sensor.sensor_name}, Wert: {sensor.value}")
   
   # Alle Einträge
   result = await manipulator.get_all()
   for sensor in result.details:
       print(f"{sensor.sensor_name}: {sensor.value} {sensor.unit}")
   
   # Mit Filtern
   result = await manipulator.get_all(filters={
       "sensor_name": "temperature_1"
   })
   
   # Mit Sortierung und Limit
   result = await manipulator.get_all(
       order_by=tb_sensor_data.timestamp.desc(),
       limit=10
   )

Daten aktualisieren (Update)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach Filter aktualisieren
   result = await manipulator.update(
       data={"value": 25.0},
       filters={"sensor_name": "temperature_1"}
   )
   
   if result.success:
       print(f"{result.details} Einträge aktualisiert")

Daten löschen (Delete)
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach ID löschen
   result = await manipulator.delete_by_id(sensor_id)
   
   # Nach Filter löschen
   result = await manipulator.delete(filters={
       "sensor_name": "old_sensor"
   })

Redis-Zugriff
-------------

RedisClient-Klasse
^^^^^^^^^^^^^^^^^^

Die :class:`~vyra_base.storage.redis_client.RedisClient`-Klasse verwaltet Redis-Verbindungen:

.. code-block:: python

   from vyra_base.storage.redis_client import RedisClient
   
   # Redis-Client erstellen
   redis = RedisClient(
       host="redis",
       port=6379,
       ssl=True,  # TLS-Verschlüsselung
       ssl_cert_reqs="required",
       ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem"
   )
   
   # Wert setzen
   await redis.set("key", "value")
   
   # Wert lesen
   value = await redis.get("key")
   
   # Alle Keys auflisten
   keys = await redis.keys("*")

.. note::
   Redis wird typischerweise über die :doc:`core/volatile`-Klasse verwendet,
   nicht direkt über RedisClient. Siehe :doc:`core/volatile` für Details.

Integration mit Entity
----------------------

Über VyraEntity
^^^^^^^^^^^^^^^

Entity stellt automatisch Storage-Zugriff bereit:

.. code-block:: python

   # Datenbank-Zugriff
   db = entity.storage.db_access
   
   # Redis-Zugriff
   redis = entity.storage.redis_client
   
   # Custom Tabelle registrieren
   from vyra_base.storage.db_manipulator import DbManipulator
   
   sensor_manipulator = DbManipulator(
       db_access=entity.storage.db_access,
       table_structure=tb_sensor_data
   )

Storage-Setup
^^^^^^^^^^^^^

.. code-block:: python

   # In _base_.py oder Entity-Initialisierung
   await entity.setup_storage(
       db_path="/workspace/storage/data/module.db",
       redis_host="redis",
       redis_port=6379
   )

Best Practices
--------------

Datenbank
^^^^^^^^^

✅ **Empfohlen:**

* Verwenden Sie ``tb_`` Prefix für alle Tabellen
* Nutzen Sie UUID für Primary Keys
* Fügen Sie ``timestamp`` Felder hinzu
* Verwenden Sie ``DbManipulator`` für CRUD-Operationen
* Definieren Sie Constraints (unique, nullable)

❌ **Vermeiden:**

* Tabellen ohne ``tb_`` Prefix
* Sehr große Textfelder (> 10 KB, nutzen Sie Dateien)
* Blocking-Operationen (immer ``await`` verwenden)
* SQL-Injection-anfällige Raw-Queries

Redis
^^^^^

✅ **Empfohlen:**

* Nutzen Sie Volatiles (siehe :doc:`core/volatile`)
* Verwenden Sie kurze, aussagekräftige Keys
* Implementieren Sie TTL für temporäre Daten
* Nutzen Sie TLS für Verbindungen

❌ **Vermeiden:**

* Direkter RedisClient-Zugriff (nutzen Sie Volatile)
* Sehr große Werte (> 1 MB)
* Persistente Daten in Redis

Fehlerbehandlung
----------------

.. code-block:: python

   result = await manipulator.get_by_id(sensor_id)
   
   if result.success:
       # Erfolg
       data = result.details
       print(f"Gefunden: {data}")
   else:
       # Fehler
       print(f"Fehler: {result.message}")
       print(f"Details: {result.details}")

Migration & Schema-Updates
--------------------------

Bei Schema-Änderungen:

.. code-block:: python

   # Alte Tabelle löschen (Vorsicht: Datenverlust!)
   await db.drop_table(tb_old_data)
   
   # Neue Tabelle erstellen
   await db.create_selected_table([tb_new_data])
   
   # Oder: Alembic für Migrationen verwenden (empfohlen)

.. tip::
   Für Produktionsumgebungen sollten Sie Alembic für Datenbank-Migrationen verwenden.
   Dies ermöglicht versionierte Schema-Updates ohne Datenverlust.
   
   Weitere Informationen: https://alembic.sqlalchemy.org/

Speicherorte
------------

**SQLite-Datenbanken**: ``/workspace/storage/data/*.db``

**Redis-Daten**: In-Memory (temporär)

**Redis-Zertifikate**: ``/workspace/storage/certificates/redis/``

Weiterführende Informationen
-----------------------------

* :doc:`core/parameter` - Parameter mit SQLite
* :doc:`core/volatile` - Volatiles mit Redis
* :doc:`vyra_base.storage` - API-Referenz
* :class:`~vyra_base.storage.db_access.DbAccess` - Datenbank-Zugriff
* :class:`~vyra_base.storage.db_manipulator.DbManipulator` - CRUD-Operationen
* :class:`~vyra_base.storage.redis_client.RedisClient` - Redis-Client
* SQLAlchemy Dokumentation: https://docs.sqlalchemy.org/
* Redis Dokumentation: https://redis.io/docs/
