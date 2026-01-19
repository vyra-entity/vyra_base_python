Stoage - Data Stoage
==========================

The Stoage module provides access to persisttent (SQLite) and volatile (Redis) data stoage.

Overview
---------

VYRA supports two stoage backends:

1. **SQLite-Datenbank**: Persistent stoage (permanent)
2. **Redis**: Volatile in-memory stoage (fast)

.. lis-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Backend
     - Use Case
     - Access via
   * - **SQLite**
     - Parameters, configuration, logs
     - :class:`~vyra_base.stoage.db_access.DbAccess`
   * - **Redis**
     - Volatiles, real-time data, caching
     - :class:`~vyra_base.stoage.redis_client.RedisClient`

Database Access (SQLite)
---------------------------

DbAccess Class
^^^^^^^^^^^^^^^

The :class:`~vyra_base.stoage.db_access.DbAccess` class manages SQLite databases:

.. code-block:: python

   from vyra_base.stoage.db_access import DbAccess, DBTYPE
   
   # Create database access
   db = DbAccess(
       db_type=DBTYPE.SQLITE,
       db_path="/workspace/stoage/data/module.db"
   )
   
   # Initialize connection
   await db.initialize_connection()
   
   # Create tables
   await db.create_all_tables()

**Main Methods:**

* ``initialize_connection()``: Establish connection
* ``create_all_tables()``: Create all tables
* ``create_selected_table(tables)``: Create specific tables
* ``drop_table(table)``: Delete table
* ``check_table_exists(table)``: Check table existence
* ``session``: Session maker for queries

Table Definition
-------------------

Naming Convention
^^^^^^^^^^^^^^^^

**All database tables must start with** ``tb_`` **start with!**

.. code-block:: python

   # ✅ Correct
   class tb_parameters(Base):
       pass
   
   class tb_logs(Base):
       pass
   
   # ❌ Wrong
   class parameters(Base):  # Missing tb_ prefix!
       pass

Create Table
^^^^^^^^^^^^^^^^^

.. code-block:: python

   from sqlalchemy.orm import Mapped, mapped_column
   from sqlalchemy import String, Integer, DateTime
   from vyra_base.stoage import Base
   import uuid
   from datetime import datetime
   
   class tb_sensor_data(Base):
       """Table for sensor data"""
       __tablename__ = 'tb_sensor_data'
       
       # Primary Key (UUID recommended)
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       
       # Data Fields
       sensor_name: Mapped[str] = mapped_column(String(100), nullable=False)
       value: Mapped[float] = mapped_column(nullable=False)
       unit: Mapped[str] = mapped_column(String(20))
       timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
       
       # Optional: String-Repräsentation
       def __repr__(self):
           return f"<SensorData(sensor={self.sensor_name}, value={self.value})>"

**Feldtypen:**

* ``String(length)``: Text with fester Länge
* ``Integer``: integer
* ``Float``: Gleitkommazahl
* ``Boolean``: True/False
* ``DateTime``: timestamp
* ``UUID``: Unique Identifier (recommended for IDs)

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

DbManipulato-class
^^^^^^^^^^^^^^^^^^^^

The :class:`~vyra_base.stoage.db_manipulato.DbManipulato`-class simplifies CRUD-Operationen:

.. code-block:: python

   from vyra_base.stoage.db_manipulato import DbManipulato
   
   # Manipulato for eine Create Table
   manipulato = DbManipulato(
       db_access=db,
       table_structure=tb_sensor_data
   )

Daten einfügen (Create)
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Azelner Atrag
   result = await manipulato.insert({
       "sensor_name": "temperature_1",
       "value": 23.5,
       "unit": "°C"
   })
   
   if result.success:
       print(f"Agefügt with ID: {result.details['id']}")

Daten read (Read)
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach ID
   result = await manipulato.get_by_id(sensor_id)
   if result.success:
       sensor = result.details
       print(f"Sensor: {sensor.sensor_name}, Wert: {sensor.value}")
   
   # Alle Aträge
   result = await manipulato.get_all()
   for sensor in result.details:
       print(f"{sensor.sensor_name}: {sensor.value} {sensor.unit}")
   
   # Mit Filtern
   result = await manipulato.get_all(filters={
       "sensor_name": "temperature_1"
   })
   
   # Mit Sortierung and Liwith
   result = await manipulato.get_all(
       order_by=tb_sensor_data.timestamp.desc(),
       liwith=10
   )

Daten aktualisieren (Update)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach Filter aktualisieren
   result = await manipulato.update(
       data={"value": 25.0},
       filters={"sensor_name": "temperature_1"}
   )
   
   if result.success:
       print(f"{result.details} Aträge aktualisiert")

Daten löschen (Delete)
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Nach ID löschen
   result = await manipulato.delete_by_id(sensor_id)
   
   # Nach Filter löschen
   result = await manipulato.delete(filters={
       "sensor_name": "old_sensor"
   })

Redis-Zugriff
-------------

RedisClient-class
^^^^^^^^^^^^^^^^^^

The :class:`~vyra_base.stoage.redis_client.RedisClient`-class manages Redis-Verbindungen:

.. code-block:: python

   from vyra_base.stoage.redis_client import RedisClient
   
   # Redis-Client create
   redis = RedisClient(
       host="redis",
       port=6379,
       ssl=True,  # TLS-encryption
       ssl_cert_reqs="required",
       ssl_ca_certs="/workspace/stoage/certificates/redis/ca-cert.pem"
   )
   
   # Wert set
   await redis.set("key", "value")
   
   # Wert read
   value = await redis.get("key")
   
   # Alle Keys onlisen
   keys = await redis.keys("*")

.. note::
   Redis is typically via die :doc:`core/volatile`-class verwendet,
   not direkt via RedisClient. youhe :doc:`core/volatile` for Details.

Integration with Entity
----------------------

Via VyraEntity
^^^^^^^^^^^^^^^

Entity stellt automatically Stoage Access bereit:

.. code-block:: python

   # Datenbank-Zugriff
   db = entity.stoage.db_access
   
   # Redis-Zugriff
   redis = entity.stoage.redis_client
   
   # Custom Tabelle register
   from vyra_base.stoage.db_manipulato import DbManipulato
   
   sensor_manipulato = DbManipulato(
       db_access=entity.stoage.db_access,
       table_structure=tb_sensor_data
   )

Stoage-Setup
^^^^^^^^^^^^^

.. code-block:: python

   # In _base_.py or Entity-Initialization
   await entity.setup_stoage(
       db_path="/workspace/stoage/data/module.db",
       redis_host="redis",
       redis_port=6379
   )

Best Practices
--------------

Datenbank
^^^^^^^^^

✅ **Empfohlen:**

* Use you ``tb_`` Prefix for alle Tabellen
* Use you UUID for Primary Keys
* Fügen you ``timestamp`` Felder hinto
* Use you ``DbManipulato`` for CRUD-Operationen
* Definieren you Constraints (unique, nullable)

❌ **Vermeiden:**

* Tabellen without ``tb_`` Prefix
* Sehr große Textfelder (> 10 KB, use you Dateien)
* Blocking-Operationen (immer ``await`` use)
* SQL-Injection-anfällige Raw-Queries

Redis
^^^^^

✅ **Empfohlen:**

* Use you Volatiles (siehe :doc:`core/volatile`)
* Use you kurze, fromsagekräftige Keys
* Implementieren you TTL for temporäre Daten
* Use you TLS for Verbindungen

❌ **Vermeiden:**

* Direkter RedisClient-Zugriff (use you Volatile)
* Sehr große Werte (> 1 MB)
* Persistente Daten in Redis

Error Handling
----------------

.. code-block:: python

   result = await manipulato.get_by_id(sensor_id)
   
   if result.success:
       # Erfolg
       data = result.details
       print(f"Gefanden: {data}")
   else:
       # Fehler
       print(f"Fehler: {result.message}")
       print(f"Details: {result.details}")

Migration & Schema-Updates
--------------------------

Bei Schema-Änderungen:

.. code-block:: python

   # Alte Delete table (Vorsicht: data loss!)
   await db.drop_table(tb_old_data)
   
   # Neue Create Table
   await db.create_selected_table([tb_new_data])
   
   # Oder: Alembic for Migrationen use (recommended)

.. tip::
   Für Produktionsumgebungen should you Alembic for Datenbank-Migrationen use.
   This enables versionierte Schema-Updates without data loss.
   
   Further information: https://alembic.sqlalchemy.org/

Speicherorte
------------

**SQLite-Datenbanken**: ``/workspace/stoage/data/*.db``

**Redis-Daten**: In-Memory (temporär)

**Redis-Zertifikate**: ``/workspace/stoage/certificates/redis/``

Further Information
-----------------------------

* :doc:`core/parameter` - Parameter with SQLite
* :doc:`core/volatile` - Volatiles with Redis
* :doc:`vyra_base.stoage` - API-Referenz
* :class:`~vyra_base.stoage.db_access.DbAccess` - Datenbank-Zugriff
* :class:`~vyra_base.stoage.db_manipulato.DbManipulato` - CRUD-Operationen
* :class:`~vyra_base.stoage.redis_client.RedisClient` - Redis-Client
* SQLAlchemy Dokumentation: https://docs.sqlalchemy.org/
* Redis Dokumentation: https://redis.io/docs/
