Security Framework Examples
===========================

Runnable security examples are maintained in the repository root under
``examples/security/`` and ``examples/12_security_levels/``.

Why this layout?
----------------

- Keep executable code close to runtime dependencies
- Avoid drift between docs snippets and actual scripts
- Allow direct copy/run workflows for users

Example Index
-------------

.. list-table::
   :header-rows: 1
   :widths: 35 45 20

   * - File
     - Description
     - Runtime
   * - ``examples/security/secure_server_node.py``
     - Secure server with RequestAccess handling and session cleanup
     - ROS2
   * - ``examples/security/secure_client_node.py``
     - Secure client authentication and secure communication setup
     - ROS2
   * - ``examples/security/access_level_example.py``
     - Access-level design pattern with ``@security_required``
     - Reference
   * - ``examples/12_security_levels/security_level_matrix.py``
     - Security-level matrix and algorithm mapping helpers
     - Pure Python

Quick Run
---------

.. code-block:: bash

   cd /path/to/vyra_base_python
   python examples/12_security_levels/security_level_matrix.py

   # ROS2 examples
   python examples/security/secure_server_node.py
   python examples/security/secure_client_node.py

Validation Checklist
--------------------

Before using the ROS2 examples, verify:

- ROS2 environment is sourced
- Interfaces are built and discoverable
- ``VBASERequestAccess`` service type is available
- Security level and key/certificate settings match your environment
