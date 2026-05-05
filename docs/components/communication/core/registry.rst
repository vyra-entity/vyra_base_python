DataSpace Registry
==================

Central registry for all VYRA communication interfaces (Callables, Speakers, Jobs).
Thread-safe singleton pattern for global access.

**Docstring:**

"""
DataSpace Registry

Central registry for all VYRA communication interfaces (Callables, Speakers, Jobs).
Thread-safe singleton pattern for global access.
"""

**Features:**
- Central management of all interfaces
- Thread-safe registration and lookup
- Lifecycle management

**Example:**

.. code-block:: python

   registry = DataSpaceRegistry()
   registry.register_callable("my_service", callable_obj)
