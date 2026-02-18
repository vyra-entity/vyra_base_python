Quickstart — Building a VYRA Module
=====================================

This guide walks you through building a minimal VYRA module from scratch.
It covers every layer of ``vyra_base`` you will use day-to-day.

.. note::

   For a complete, working module see the ``vyra_module_template`` repository.
   This guide focuses on explaining *why* and *how* each piece fits together.

.. contents::
   :local:
   :depth: 2

---

Prerequisites
-------------

.. code-block:: bash

   pip install vyra_base

   # Transport (choose at least one):
   pip install eclipse-zenoh   # Zenoh — recommended default
   pip install redis            # Redis transport

   # Database (SQLAlchemy is included with vyra_base):
   # SQLite works out of the box — no extra install needed

---

Step 1 — Load the Entity (_base_.py)
--------------------------------------

Every VYRA module has a ``_base_.py`` that loads the central ``VyraEntity``.
The entity is the glue between your component, the communication layer,
and the VYRA framework.

.. code-block:: python

   # src/my_module/my_module/_base_.py
   from vyra_base.core.entity import VyraEntity, build_entity
   from vyra_base.defaults.entries import ModuleEntry

   PROJECT_SETTINGS = ModuleEntry(
       module_name="my_module",
       module_version="0.1.0",
       module_id="my_module_abc123",
   )

   async def build_base() -> VyraEntity:
       """Create and configure the VyraEntity for this module."""
       entity = await build_entity(PROJECT_SETTINGS)
       base_interfaces = await _create_base_interfaces()
       await entity.set_interfaces(base_interfaces)   # registers all @remote_service methods
       return entity

   async def _create_base_interfaces():
       """Return configuration for built-in feeders and base services."""
       # ... see module template for full example
       return []

.. important::

   Always call ``entity.set_interfaces()`` before using any communication.
   This is what binds your decorated methods to live ROS2/Zenoh/Redis interfaces.

---

Step 2 — Communication Interfaces
-----------------------------------

Use the four decorators to expose your component's methods over the network.

Services (request / response)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.com import remote_service, ProtocolType, AccessLevel

   class MyComponent:
       @remote_service(
           name="get_status",
           protocols=[ProtocolType.ZENOH],
           namespace="my_module",
           access_level=AccessLevel.PUBLIC,
       )
       async def get_status(self, request, response=None):
           return {"status": "running", "uptime": 42}

Publisher (fire and forget)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.com import remote_publisher, ProtocolType

   class MyComponent:
       @remote_publisher(name="events", protocols=[ProtocolType.ZENOH], namespace="my_module")
       async def publish_event(self, event_type: str, data: dict):
           return {"type": event_type, "data": data}

Subscriber (callback)
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.com import remote_subscriber, ProtocolType

   class MyComponent:
       @remote_subscriber(name="commands", protocols=[ProtocolType.ZENOH], namespace="my_module")
       async def on_command(self, message: dict):
           print(f"Command received: {message}")

Action Server (long-running task)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.com import remote_actionServer, IActionHandler, IGoalHandle, ProtocolType

   class MyComponent(IActionHandler):
       @remote_actionServer.on_goal(name="run_task", protocols=[ProtocolType.ZENOH])
       async def on_goal(self, goal_request) -> bool:
           return goal_request.get("target") is not None  # accept/reject

       @remote_actionServer.execute(name="run_task")
       async def execute(self, goal_handle: IGoalHandle):
           for i in range(100):
               await goal_handle.publish_feedback({"progress": i})
           goal_handle.succeed()
           return {"result": "done"}

       @remote_actionServer.on_cancel(name="run_task")
       async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
           return True

.. seealso::

   :doc:`com` — Full communication module reference

---

Step 3 — Storage (Database & Redis)
-------------------------------------

SQLAlchemy Database
~~~~~~~~~~~~~~~~~~~~

``vyra_base`` ships a full SQLAlchemy async ORM layer.
All table models **must** use the ``tb_`` prefix.

.. code-block:: python

   from vyra_base.storage import Base, DbAccess, DbManipulator, DBReturnValue, DBTYPE
   from sqlalchemy.orm import Mapped, mapped_column
   from sqlalchemy import String, Float, DateTime
   import uuid
   from datetime import datetime

   # ── 1. Define your table ──────────────────────────────────────────────────
   class tb_sensor_data(Base):
       __tablename__ = "tb_sensor_data"

       id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
       sensor_id: Mapped[str] = mapped_column(String(64), index=True)
       value: Mapped[float] = mapped_column(Float)
       unit: Mapped[str] = mapped_column(String(16))
       measured_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

   # ── 2. Open the database ──────────────────────────────────────────────────
   db = DbAccess()
   await db.connect(
       db_type=DBTYPE.SQLITE,
       db_path="/workspace/storage/my_module.db",
   )
   await db.create_tables([tb_sensor_data])     # creates tables if they don't exist

   # ── 3. Write data ────────────────────────────────────────────────────────
   manipulator = DbManipulator(db)

   row = tb_sensor_data(sensor_id="sensor_01", value=23.4, unit="°C")
   result: DBReturnValue = await manipulator.insert(row)

   # ── 4. Read data ─────────────────────────────────────────────────────────
   from sqlalchemy import select
   from sqlalchemy.ext.asyncio import AsyncSession

   async with db.session() as session:
       stmt = select(tb_sensor_data).where(tb_sensor_data.sensor_id == "sensor_01")
       rows = (await session.execute(stmt)).scalars().all()
       for row in rows:
           print(row.value, row.unit)

Supported database backends:

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Backend
     - ``DBTYPE``
     - Notes
   * - SQLite (default)
     - ``DBTYPE.SQLITE``
     - No extra install; file-based; ideal for single-node modules
   * - MySQL
     - ``DBTYPE.MYSQL``
     - Requires ``aiomysql``
   * - PostgreSQL
     - ``DBTYPE.POSTGRESQL``
     - Requires ``asyncpg``

Redis Key-Value / Pub-Sub
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.storage import RedisClient, REDIS_TYPE

   redis = RedisClient(
       host="redis",
       port=6379,
       ssl=True,
       ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem",
   )
   await redis.connect()

   # Store a value
   await redis.set("my_module:config:threshold", "42")

   # Read a value
   value = await redis.get("my_module:config:threshold")

   # Publish
   await redis.publish("my_module:events", '{"type": "started"}')

.. seealso::

   :doc:`storage` — Complete storage module reference

---

Step 4 — Logging
-----------------

Use Python's standard ``logging`` module.

.. code-block:: python

   import logging

   logger = logging.getLogger(__name__)

   logger.info("✅ Module started")
   logger.warning("⚠  Low disk space")
   logger.error("❌ Connection failed: %s", error)

Log configuration is loaded from ``config/core_logging.json`` in your module directory.
The default format includes timestamp, module name, level, and message.

.. tip::

   Use structured log messages for better filtering in production:

   .. code-block:: python

      logger.info("Request processed: service=%s, duration_ms=%d", service_name, duration)

---

Step 5 — State Machine
------------------------

Every VYRA component inherits its lifecycle from ``OperationalStateMachine``.
The metaclass handles state transitions automatically — **never set state manually**.

Three-Layer Architecture
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Layer
     - States
     - Controls
   * - **Lifecycle**
     - ``UNINITIALIZED → INITIALIZING → READY → DECOMMISSIONING → DECOMMISSIONED``
     - Module existence and boot sequence
   * - **Operational**
     - ``IDLE → RUNNING → PAUSED → STOPPED``
     - Runtime activity
   * - **Health**
     - ``HEALTHY → DEGRADED → FAILED → CRITICAL_FAILURE``
     - Diagnostics and error escalation

Using ``OperationalStateMachine``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.state import OperationalStateMachine

   class MyComponent(OperationalStateMachine):
       async def initialize(self, request=None, response=None) -> bool:
           # Called automatically when entity transitions to READY
           await self._setup()
           return True   # returning True triggers READY → RUNNING

       async def stop(self, request=None, response=None) -> bool:
           await self._cleanup()
           return True

.. code-block:: python

   from vyra_base.state import UnifiedStateMachine, LifecycleState, OperationalState

   # Direct state machine (lower level):
   sm = UnifiedStateMachine(component_id="my_module")
   await sm.transition_lifecycle(LifecycleState.READY)
   await sm.transition_operational(OperationalState.RUNNING)

   print(sm.lifecycle_state)    # LifecycleState.READY
   print(sm.operational_state)  # OperationalState.RUNNING

.. seealso::

   :doc:`state` — Complete state machine documentation

---

Step 6 — Security
------------------

VYRA provides a built-in 5-level security model for inter-module communication.

Security Levels
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 10 20 70

   * - Level
     - ``SecurityLevel``
     - Description
   * - 1
     - ``NONE``
     - No checks — development only
   * - 2
     - ``BASIC_AUTH``
     - Module ID verification
   * - 3
     - ``TIMESTAMP``
     - ID + timestamp validation (replay protection)
   * - 4
     - ``HMAC``
     - HMAC-SHA256 message signatures
   * - 5
     - ``DIGITAL_SIGNATURE``
     - Certificate-based PKI

Server Side (SecurityManager)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.security import SecurityManager, SecurityLevel
   import rclpy

   class MyModule(rclpy.node.Node, SecurityManager):
       def __init__(self):
           super().__init__("my_module")
           SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)

Client Side (SecurePublisher)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from vyra_base.security import SecurePublisher, create_security_context, SecurityLevel

   ctx = create_security_context(
       module_id="my_module_abc123",
       level=SecurityLevel.HMAC,
       token="shared_secret",
       hmac_key=b"super_secret_key",
   )
   pub = SecurePublisher(node, MsgType, "/topic", ctx)
   pub.publish(msg)

.. seealso::

   :doc:`security` — Complete security framework reference

---

Step 7 — Communication Overview
---------------------------------

The communication module is organized into **four sub-systems**:

Transport Layer
~~~~~~~~~~~~~~~~

Low-latency protocols for in-process or DDS-based communication.

.. list-table::
   :header-rows: 1
   :widths: 15 20 65

   * - Protocol
     - ``ProtocolType``
     - Use case
   * - **Zenoh** (default)
     - ``ZENOH``
     - High-performance pub/sub; recommended for all new modules
   * - **ROS2**
     - ``ROS2``
     - Robotics / DDS ecosystem; requires ROS2 installation
   * - **Redis**
     - ``REDIS``
     - Pub/sub + key-value; good for dashboard ↔ module communication
   * - **UDS**
     - ``UDS``
     - Unix Domain Sockets; zero-dependency local IPC

Automatic fallback chain: ``Zenoh → ROS2 → Redis → UDS``

External Layer
~~~~~~~~~~~~~~~

Cross-service protocols for integration with external systems.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Protocol
     - Use case
   * - **gRPC** (``GRPC``)
     - High-performance RPC over Unix sockets; backend-to-backend
   * - **MQTT** (``MQTT``)
     - IoT devices; constrained networks; QoS 0/1/2
   * - **REST** (``REST``)
     - HTTP API integration; web services
   * - **WebSocket** (``WEBSOCKET``)
     - Real-time browser clients; bidirectional streaming
   * - **Shared Memory** (``SHARED_MEMORY``)
     - Zero-copy local IPC; sub-millisecond latency

Industrial Layer
~~~~~~~~~~~~~~~~~

Northbound integration with industrial automation systems.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Protocol
     - Use case
   * - **Modbus** (``MODBUS``)
     - PLC/SCADA integration; TCP or RTU
   * - **OPC UA** (``OPCUA``)
     - MES/SCADA northbound; rich data modelling

Feeders
~~~~~~~~

Feeders automatically publish internal module state over the transport layer.
They are set up in ``_base_.py`` and run in the background.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Feeder
     - Publishes
   * - ``StateFeeder``
     - ``LifecycleState`` + ``OperationalState`` changes
   * - ``NewsFeeder``
     - Informational messages (``entity.publish_news("…")``)
   * - ``ErrorFeeder``
     - Error reports (``entity.publish_error("…")``)
   * - ``AvailableModuleFeeder``
     - Heartbeat / module availability discovery

.. code-block:: python

   # Use feeders through the entity (simplest way):
   entity.publish_news("Module started successfully")
   entity.publish_error("Redis connection lost")

.. seealso::

   :doc:`com` — Full communication module with all transport, external and industrial details
