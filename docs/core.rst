Core Components
===============

The Core module forms the heart of the VYRA framework. All central components are initialized and managed here.

.. toctree::
   :maxdepth: 2
   :caption: Core Components

   core/entity
   core/parameter
   core/volatile

Overview
--------

The Core module consists of three main components:

* **Entity**: Central management unit for modules
* **Parameter**: Persistent configuration data in SQLite
* **Volatile**: Volatile, fast data storage with Redis

Component Interaction
---------------------

The three components work closely together:

1. **VyraEntity** initializes and orchestrates all module components
2. **Parameter** manages persisttent configurations (slow, permanent)
3. **Volatile** manages volatile real-time data (fast, temporary)

.. tip::
   Use Parameter for configuration data that should persist between restarts.
   Use Volatiles for real-time data and fast caching.

Further Information
-------------------

* :doc:`core/entity` - Central Entity Management
* :doc:`core/parameter` - Persistent Parameters
* :doc:`core/volatile` - Volatile Data
* :doc:`vyra_base.core` - API Reference (auto-generated)
