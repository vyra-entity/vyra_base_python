.. vyra_base documentation master file, created by
   sphinx-quickstart on Thu Jun 26 16:16:53 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Vyra-Base Documentation
=======================

VYRA Base Python is the core library of the VYRA Framework — a standalone Python
package providing transport-agnostic communication (Zenoh, ROS2, Redis, UDS), a
3-layer state machine, structured parameter + volatile storage, a 5-level security
framework, and a WASM plugin runtime.

.. toctree::
   :maxdepth: 2
   :caption: Quick Start

   installation
   guides/quickstart
   guides/state-machine-quick-guide

.. toctree::
   :maxdepth: 2
   :caption: Developer Guides

   guides/interfaces-how-to
   guides/logging-setup

.. toctree::
   :maxdepth: 2
   :caption: State Machine

   components/state-machine/state-machine_FULL
   components/state-machine/state-machine_QUICK

.. toctree::
   :maxdepth: 2
   :caption: Components

   core
   components/communication/README
   components/storage/README
   helper
   components/security/README
   interfaces
   plugin

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   modules

