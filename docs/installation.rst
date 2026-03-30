Installation
============

.. contents::
   :local:
   :depth: 1

Prerequisites
-------------

* Python 3.11 or later
* (Optional) `eclipse-zenoh <https://pypi.org/project/eclipse-zenoh/>`_ for Zenoh transport
* (Optional) `redis <https://pypi.org/project/redis/>`_ for Redis transport
* (Optional) `wasmtime <https://pypi.org/project/wasmtime/>`_ for WASM plugin runtime

Install from PyPI
-----------------

.. code-block:: bash

   pip install vyra_base

Install optional extras
-----------------------

.. code-block:: bash

   # Zenoh transport (recommended default)
   pip install vyra_base eclipse-zenoh

   # Redis transport
   pip install vyra_base redis

   # WASM plugin runtime
   pip install vyra_base wasmtime

   # Full installation (all extras)
   pip install vyra_base eclipse-zenoh redis wasmtime

Install from source
-------------------

.. code-block:: bash

   git clone https://github.com/vyra-entity/vyra_base_python.git
   cd vyra_base_python
   pip install -e .

Using the copier template
--------------------------

The fastest way to start a new VYRA module is via the
`vyra_module_template <https://github.com/vyra-entity/vyra_module_template>`_:

.. code-block:: bash

   pip install copier
   copier copy gh:vyra-entity/vyra_module_template my_new_module
   cd my_new_module
   pip install -e .

The template pre-configures the project layout, ``_base_.py``, Dockerfile,
and interface build pipeline for you.

Verify installation
-------------------

.. code-block:: python

   import vyra_base
   print(vyra_base.__version__)
