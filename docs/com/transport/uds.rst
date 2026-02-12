UDS Transport Module
====================

Provides Unix Domain Socket-based transport implementation with layered architecture.

Overview
--------

The UDS transport module implements the ``AbstractProtocolProvider`` interface
for low-latency local inter-process communication using Unix Domain Sockets.

**Architecture Layers:**

- **communication/**: Core UDS functionality (UnixSocket, socket management)
- **vyra_models/**: VYRA abstractions (UDSCallable)
- **provider.py**: Protocol provider implementation

**Features:**

- ✅ Stream-based local IPC
- ✅ Low-latency request-response
- ✅ Server/Client pattern with ``is_callable`` flag
- ✅ Automatic connection management
- ✅ JSON message serialization
- ✅ File-based socket addressing
- ✅ Async socket operations

Usage
-----

Basic Provider Setup
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.transport.t_uds import UDSProvider, UDS_AVAILABLE
   from vyra_base.com.core.types import ProtocolType
   import os
   
   if UDS_AVAILABLE:
       # Create provider
       provider = UDSProvider(
           module_name="my_module",
           module_id="abc123",
           protocol=ProtocolType.UDS
       )
       
       # Initialize with socket directory
       socket_dir = "/tmp/vyra_sockets"
       os.makedirs(socket_dir, exist_ok=True)
       
       await provider.initialize(config={
           "socket_dir": socket_dir
       })

Create Request-Response Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Server callback
   async def handle_calculation(request_data):
       result = request_data["a"] + request_data["b"]
       return {"sum": result}
   
   # Create server (is_callable=True)
   server = await provider.create_callable(
       name="calculator",
       callback=handle_calculation
   )
   # Socket created at: /tmp/vyra_sockets/my_module_calculator.sock

Create Request-Response Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Create client (is_callable=False)
   client = await provider.create_callable(
       name="calculator",
       callback=None,  # No callback for client
       is_callable=False
   )
   
   # Make request
   request = {"a": 10, "b": 32}
   response = await client.call(request, timeout=2.0)
   print(f"Result: {response['sum']}")

Configuration
-------------

Socket Configuration
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   config = {
       "socket_dir": "/var/run/vyra",
       "socket_permissions": 0o660,  # rw-rw----
       "buffer_size": 8192,
       "timeout": 5.0,
       "max_connections": 10
   }
   
   await provider.initialize(config=config)

Socket Naming
^^^^^^^^^^^^^

Sockets are named using the pattern: ``{module_name}_{service_name}.sock``

.. code-block:: python

   # Example socket paths
   provider = UDSProvider(module_name="sensor_manager")
   
   # Server: /tmp/vyra_sockets/sensor_manager_get_data.sock
   await provider.create_callable("get_data", callback=handler)
   
   # Client connects to same path
   client = await provider.create_callable(
       "get_data", 
       callback=None, 
       is_callable=False
   )

Server/Client Flags
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Interface Type
     - Flag
     - Behavior
   * - Request-Response Server
     - ``is_callable=True``
     - Creates socket server, responds to requests (callback required)
   * - Request-Response Client
     - ``is_callable=False``
     - Connects to socket server, makes requests (no callback)

Advanced Usage
--------------

Custom Socket Path
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Override default socket naming
   server = await provider.create_callable(
       name="custom",
       callback=handler,
       socket_path="/var/run/myapp/custom.sock"
   )

Error Handling
^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.transport.t_uds.exceptions import (
       UDSConnectionError,
       UDSTimeoutError
   )
   
   try:
       response = await client.call(request, timeout=1.0)
   except UDSTimeoutError:
       logger.error("Request timed out")
   except UDSConnectionError as e:
       logger.error(f"Connection failed: {e}")

Socket Cleanup
^^^^^^^^^^^^^^

.. code-block:: python

   # Cleanup on shutdown
   async def cleanup():
       await provider.shutdown()
       # Removes socket files
   
   # Auto-cleanup with context manager
   async with UDSProvider(...) as provider:
       await provider.initialize()
       # ... use provider ...
   # Sockets cleaned up automatically

Performance Considerations
--------------------------

UDS provides the lowest latency for local IPC:

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Transport
     - Typical Latency
     - Use Case
   * - UDS
     - < 100 μs
     - Local high-frequency IPC
   * - Redis (localhost)
     - ~ 200-500 μs
     - Local with pub/sub features
   * - ROS2 (DDS)
     - ~ 1-5 ms
     - Distributed systems, discovery

**Best Practices:**

- Use UDS for high-frequency local communication (e.g., control loops)
- Use Redis for pub/sub patterns or remote access
- Use ROS2 for distributed systems with discovery

API Reference
-------------

.. automodule:: vyra_base.com.transport.t_uds.provider
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: vyra_base.com.transport.t_uds.vyra_models
   :members:
   :undoc-members:
   :show-inheritance:

Dependencies
------------

- ``asyncio`` (Async I/O, built-in)
- ``socket`` (Unix sockets, built-in)
- ``json`` (Serialization, built-in)

See Also
--------

- :doc:`../core/factory` - InterfaceFactory for protocol-agnostic usage
- :doc:`ros2` - ROS2 transport provider
- :doc:`redis` - Redis transport provider
