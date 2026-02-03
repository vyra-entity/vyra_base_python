Interface Factory
=================

Unified interface creation with automatic protocol selection and fallback.

**Docstring:**

"""
Interface Factory

Unified interface creation with automatic protocol selection and fallback.
"""

**Features:**
- Automatic protocol selection
- Fallback chain (ROS2 → SharedMemory → UDS → Redis → gRPC)
- Protocol availability checking
- Graceful degradation

**Example:**

.. code-block:: python

   # Auto-select best available protocol
   callable = await InterfaceFactory.create_callable(
      "my_service",
      callback=handle_request
   )

   # Explicit protocol with fallback
