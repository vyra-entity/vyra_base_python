Zenoh Transport
===============

The Zenoh transport is the **default** and recommended protocol for VYRA modules.

.. automodule:: vyra_base.com.transport.t_zenoh
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

Overview
--------

Zenoh is a high-performance publish/subscribe and query/reply protocol
designed for distributed systems. It requires no central broker, works
through NAT, and supports zero-copy for high-throughput scenarios.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Property
     - Value
   * - Module
     - ``vyra_base.com.transport.t_zenoh``
   * - ``ProtocolType``
     - ``ProtocolType.ZENOH``
   * - Default
     - Yes — first in fallback chain
   * - Install
     - ``pip install eclipse-zenoh``
   * - Pattern
     - Pub/Sub + Query/Reply (queryable = server, query_client = client)

Key Classes
-----------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Class
     - Description
   * - ``ZenohProvider``
     - Protocol provider; registers Zenoh with ``ProviderRegistry``
   * - ``ZenohSession``
     - Manages the Zenoh session lifecycle
   * - ``SessionConfig``
     - Configuration dataclass for the Zenoh session
   * - ``SessionMode``
     - Enum: ``CLIENT``, ``PEER``, ``ROUTER``

Usage
-----

Zenoh is used automatically via ``InterfaceFactory`` when available.
To configure the session manually:

.. code-block:: python

   from vyra_base.com.transport.t_zenoh import ZenohProvider, ZenohSession, SessionConfig, SessionMode

   config = SessionConfig(
       mode=SessionMode.PEER,
       connect=["tcp/localhost:7447"],
   )
   session = ZenohSession(config)
   await session.open()

   provider = ZenohProvider(session=session)
   await provider.initialize()

For standard module use, Zenoh is initialized automatically at startup.
See :doc:`../../quickstart` for the full module initialization pattern.
