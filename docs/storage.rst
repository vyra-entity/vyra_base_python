Storage - Data Storage
======================

The Storage module provides access to persisttent (SQLite) and volatile (Redis) data storage.

Overview
--------

VYRA supports two storage backends:

1. **SQLite-Database**: Persistent storage (permanent)
2. **Redis**: Volatile in-memory storage (fast)

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Backend
     - Use Case
     - Access via
   * - **SQLite**
     - Parameters, configuration, logs
     - :class:`~vyra_base.storage.db_access.DbAccess`
   * - **Redis**
     - Volatiles, real-time data, caching
     - :class:`~vyra_base.storage.redis_client.RedisClient`

Database Access (SQLite)
------------------------

DbAccess Class
^^^^^^^^^^^^^^

The :class:`~vyra_base.storage.db_access.DbAccess` class manages SQLite databases:

.. code-block:: python

   from vyra_base.storage.db_access import DbAccess, DBTYPE
   
   # Create database access
   db = DbAccess(
       db_type=DBTYPE.SQLITE,
       db_path="/workspace/storage/data/module.db"
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
----------------

Naming Convention
^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^

.. code-block:: python

   from sqlalchemy.orm import Mapped, mapped_column
   from sqlalchemy import String, Integer, DateTime
   from vyra_base.storage import Base
   import uuid
   from datetime import datetime
   
   class tb_sensor_data(Base):
       """Table for Sensor data"""
       __tablename__ = 'tb_sensor_data'
       
       # Primary Key (UUID recommended)
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       
       # Data Fields
       sensor_name: Mapped[str] = mapped_column(String(100), nullable=False)
       value: Mapped[float] = mapped_column(nullable=False)
       unit: Mapped[str] = mapped_column(String(20))
       timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
       
       # Optional: String representation
       def __repr__(self):
           return f"<SensorData(Sensor={self.sensor_name}, value={self.value})>"

**Field Types:**

* ``String(length)``: Text with fester Länge
* ``Integer``: integer
* ``Float``: Floating point number
* ``Boolean``: True/False
* ``DateTime``: timestamp
* ``UUID``: Unique Identifier (recommended for IDs)

Example-Tabellen
^^^^^^^^^^^^^^^^

**Parameter Table**:

.. code-block:: python

   class tb_parameters(Base):
       __tablename__ = 'tb_parameters'
       
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       name: Mapped[str] = mapped_column(String(100), unique=True, nullable=False)
       value: Mapped[str] = mapped_column(String(500))
       description: Mapped[str] = mapped_column(String(500))
       updated_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

**Log Table**:

.. code-block:: python

   class tb_logs(Base):
       __tablename__ = 'tb_logs'
       
       id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
       level: Mapped[str] = mapped_column(String(20))  # INFO, WARNING, ERROR
       message: Mapped[str] = mapped_column(String(1000))
       module: Mapped[str] = mapped_column(String(100))
       timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

Database-Operationen
--------------------

DbManipulato-class
^^^^^^^^^^^^^^^^^^

The :class:`~vyra_base.storage.db_manipulato.DbManipulato`-class simplifies CRUD Operations:

.. code-block:: python

   from vyra_base.storage.db_manipulato import DbManipulato
   
   # Manipulato for eine Create Table
   manipulato = DbManipulato(
       db_access=db,
       table_structure=tb_sensor_data
   )

Insert Data (Create)
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Azelner Atrag
   result = await manipulato.insert({
       "sensor_name": "temperature_1",
       "value": 23.5,
       "unit": "°C"
   })
   
   if result.success:
       print(f"Agefügt with ID: {result.Details['id']}")

Daten read (Read)
^^^^^^^^^^^^^^^^^

.. code-block:: python

   # By ID
   result = await manipulato.get_by_id(sensor_id)
   if result.success:
       Sensor = result.Details
       print(f"Sensor: {Sensor.sensor_name}, Value: {Sensor.value}")
   
   # Alle Aträge
   result = await manipulato.get_all()
   for Sensor in result.Details:
       print(f"{Sensor.sensor_name}: {Sensor.value} {Sensor.unit}")
   
   # With Filters
   result = await manipulato.get_all(filters={
       "sensor_name": "temperature_1"
   })
   
   # Mit Sortierung and Liwith
   result = await manipulato.get_all(
       order_by=tb_sensor_data.timestamp.desc(),
       liwith=10
   )

Update Data (Update)
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # By Filter aktualisieren
   result = await manipulato.update(
       data={"value": 25.0},
       filters={"sensor_name": "temperature_1"}
   )
   
   if result.success:
       print(f"{result.Details} Aträge aktualisiert")

Delete Data (Delete)
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # By ID löschen
   result = await manipulato.delete_by_id(sensor_id)
   
   # By Filter löschen
   result = await manipulato.delete(filters={
       "sensor_name": "old_sensor"
   })

Redis-Access
------------

RedisClient-class
^^^^^^^^^^^^^^^^^

The :class:`~vyra_base.storage.redis_client.RedisClient`-class manages Redis-Verbindungen:

.. code-block:: python

   from vyra_base.storage.redis_client import RedisClient
   
   # Redis-Client create
   redis = RedisClient(
       host="redis",
       port=6379,
       ssl=True,  # TLS-encryption
       ssl_cert_reqs="required",
       ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem"
   )
   
   # Value set
   await redis.set("key", "value")
   
   # Value read
   value = await redis.get("key")
   
   # Alle Keys onlisen
   keys = await redis.keys("*")

.. note::
   Redis is typically via die :doc:`core/volatile`-class verwendet,
   not direkt via RedisClient. youhe :doc:`core/volatile` for Details.

Integration with Entity
-----------------------

Via VyraEntity
^^^^^^^^^^^^^^^

Entity stellt automatically Storage Access bereit:

.. code-block:: python

   # Database-Access
   db = entity.storage.db_access
   
   # Redis-Access
   redis = entity.storage.redis_client
   
   # Custom Tabelle register
   from vyra_base.storage.db_manipulato import DbManipulato
   
   sensor_manipulato = DbManipulato(
       db_access=entity.storage.db_access,
       table_structure=tb_sensor_data
   )

Storage-Setup
^^^^^^^^^^^^^

.. code-block:: python

   # In _base_.py or Entity-Initialization
   await entity.setup_storage(
       db_path="/workspace/storage/data/module.db",
       redis_host="redis",
       redis_port=6379
   )

Best Practices
--------------

Database
^^^^^^^^^

✅ **Recommended:**

* Use you ``tb_`` Prefix for alle Tabellen
* Use you UUID for Primary Keys
* Fügen you ``timestamp`` Felder hinto
* Use you ``DbManipulato`` for CRUD Operations
* Definieren you Constraints (unique, nullable)

❌ **Avoid:**

* Tabellen without ``tb_`` Prefix
* Very large text fields (> 10 KB, use you Dateien)
* Blocking operations (always ``await`` use)
* SQL injection vulnerable raw queries

Redis
^^^^^

✅ **Recommended:**

* Use you Volatiles (See :doc:`core/volatile`)
* Use you kurze, fromsagekräftige Keys
* Implementieren you TTL for temporäre Daten
* Use you TLS for Verbindungen

❌ **Avoid:**

* Direkter RedisClient-Access (use you Volatile)
* Very large values (> 1 MB)
* Persistent data in Redis

Error Handling
--------------

.. code-block:: python

   result = await manipulato.get_by_id(sensor_id)
   
   if result.success:
       # Success
       data = result.Details
       print(f"Gefanden: {data}")
   else:
       # Error
       print(f"Error: {result.message}")
       print(f"Details: {result.Details}")

Migration & Schema Updates
--------------------------

For schema changes:

.. code-block:: python

   # Alte Delete table (Vorsicht: data loss!)
   await db.drop_table(tb_old_data)
   
   # Neue Create Table
   await db.create_selected_table([tb_new_data])
   
   # Oder: Alembic for Migrationen use (recommended)

.. tip::
   Für Produktionsumgebungen should you Alembic for Database-Migrationen use.
   This enables versionierte Schema-Updates without data loss.
   
   Further information: https://alembic.sqlalchemy.org/

Storage Locations
-----------------

**SQLite-Databases**: ``/workspace/storage/data/*.db``

**Redis-Daten**: In-Memory (temporär)

**Redis Certificates**: ``/workspace/storage/certificates/redis/``

Further Information
-------------------

* :doc:`core/parameter` - Parameter with SQLite
* :doc:`core/volatile` - Volatiles with Redis
* :doc:`vyra_base.storage` - API-Referenz
* :class:`~vyra_base.storage.db_access.DbAccess` - Database-Access
* :class:`~vyra_base.storage.db_manipulato.DbManipulato` - CRUD Operations
* :class:`~vyra_base.storage.redis_client.RedisClient` - Redis-Client
* SQLAlchemy Dokumentation: https://docs.sqlalchemy.org/
* Redis Dokumentation: https://redis.io/docs/
