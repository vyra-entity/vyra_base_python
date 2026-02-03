Communication Layer Exceptions
=============================

Centralized exception hierarchy for all communication protocols.

**Docstring:**

"""
Communication Layer Exceptions

Centralized exception hierarchy for all communication protocols.
"""

**Exception Hierarchy:**

- CommunicationError (base for all communication errors)
   - ProtocolUnavailableError (protocol not available)
   - ProtocolNotInitializedError (protocol not initialized)
   - TransportError (base for transport errors)
      - ConnectionError (connection failed)
      - TimeoutError (timeout exceeded)
      - SerializationError (serialization failed)

**Example:**

.. code-block:: python

   try:
      ...
   except CommunicationError as e:
      print(f"Error: {e}")
