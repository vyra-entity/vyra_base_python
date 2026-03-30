Plugin Runtime
==============

The VYRA plugin system allows modules to load and execute sandboxed WASM
plugins at runtime.  Python host functions are exposed to the plugin sandbox
so that plugins can call back into the host application.

.. contents::
   :local:
   :depth: 2

Overview
--------

Two backends are available:

.. list-table::
   :header-rows: 1
   :widths: 25 45 30

   * - Backend
     - When selected
     - Dependency
   * - ``WasmRuntime``
     - ``wasmtime`` installed **and** ``.wasm`` file exists
     - ``pip install wasmtime``
   * - ``StubRuntime``
     - All other cases (pure Python fallback)
     - None

``create_plugin_runtime()`` selects the backend automatically.

Quick Start
-----------

.. code-block:: python

   from pathlib import Path
   from vyra_base.plugin.runtime import create_plugin_runtime

   runtime = create_plugin_runtime(
       plugin_id="my-plugin",
       wasm_path=Path("/opt/plugins/my-plugin/logic.wasm"),
       initial_state={"counter": 0},
   )
   await runtime.start()

   result = await runtime.call("increment", {"step": 1})
   await runtime.stop()

Plugin File Layout
------------------

.. code-block:: text

   my_plugin/
   ├── logic.wasm          # Compiled WASM module
   └── metadata.json       # Plugin manifest

``metadata.json`` format:

.. code-block:: json

   {
       "id": "my-plugin",
       "version": "1.0.0",
       "description": "An example counter plugin",
       "exports": ["increment", "reset", "get_value"],
       "imports": ["host_log", "host_emit_event"],
       "initial_state": {
           "counter": {"type": "int", "default": 0},
           "step":    {"type": "int", "default": 1}
       }
   }

Host Functions
--------------

Host functions are Python callbacks that the plugin can invoke from within
the WASM sandbox.  Subclass ``BaseHostFunctions`` and prefix methods with
``host_``:

.. code-block:: python

   from vyra_base.plugin.host_functions import BaseHostFunctions

   class MyHostFunctions(BaseHostFunctions):
       def host_log(self, message: str) -> None:
           print(f"[plugin] {message}")

       def host_emit_event(self, event_type: str, payload: str) -> None:
           print(f"[event:{event_type}] {payload}")

   runtime = create_plugin_runtime(
       plugin_id="my-plugin",
       wasm_path=...,
       host=MyHostFunctions(),
   )

Use ``NullHostFunctions`` when no callbacks are needed.

API Reference
-------------

.. autofunction:: vyra_base.plugin.runtime.create_plugin_runtime

.. autoclass:: vyra_base.plugin.runtime.WasmRuntime
   :members:
   :inherited-members:

.. autoclass:: vyra_base.plugin.runtime.StubRuntime
   :members:
   :inherited-members:

.. autoclass:: vyra_base.plugin.host_functions.BaseHostFunctions
   :members:

.. autoclass:: vyra_base.plugin.runtime.NullHostFunctions
   :members:

Examples
--------

See `examples/13_plugin/ <../examples/13_plugin/>`_ for runnable examples.
