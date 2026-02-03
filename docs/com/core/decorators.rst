Communication Decorators
========================

Modern decorators for multi-protocol communication.

**Docstring:**

"""
Communication Decorators

Modern decorators for multi-protocol communication.
"""

**remote_callable**

Decorator for methods that should be accessible as remote callables.

**Parameters:**
- name: Callable name (defaults to function name)
- protocols: Preferred protocols (uses fallback if None)
- auto_register: Whether to auto-register
- **kwargs: Additional parameters

**Example:**

.. code-block:: python

   class MyComponent(OperationalStateMachine):
      @remote_callable
      async def calculate(self, request, response=None):
         result = request["x"] + request["y"]
         return {"result": result}
