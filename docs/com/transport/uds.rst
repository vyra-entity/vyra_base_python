# vyra_base.com.transport.uds

UDS Transport Module
====================

Provides Unix Domain Socket-based transport implementation with layered architecture:

Layers:
    - communication/: Core UDS functionality (UnixSocket, socket management)
    - vyra_models/: VYRA abstractions (UDSCallable)
    - provider.py: Interface layer for VYRA integration

**Features:**
- Stream-based local IPC
- Low-latency request-response
- Automatic connection management
- JSON serialization

**Usage:**

.. code-block:: python

    from vyra_base.com.transport.uds import UDSProvider, UDS_AVAILABLE
    
    if UDS_AVAILABLE:
        provider = UDSProvider(module_name="my_module")
        await provider.initialize()
        callable = await provider.create_callable("service", callback)
