"""
Communication handlers for V.Y.R.A. modules.

Handler hierarchy
-----------------

.. code-block:: text

    IFeederHandler  (interfaces.py)  — ABC: dispatch() + get_protocol() + logging.Handler
        └── CommunicationHandler  (communication.py)  — concrete base
                ├── ROS2Handler    (ros2.py)      — ROS2 CAL publisher (via InterfaceFactory)
                ├── ZenohHandler   (zenoh.py)     — Zenoh CAL publisher (via InterfaceFactory)
                ├── RedisHandler   (redis.py)     — Redis CAL publisher (via InterfaceFactory)
                ├── UDSHandler     (uds.py)       — UDS CAL publisher (via InterfaceFactory)
                └── DBCommunicationHandler (database.py) — async DB persistence

Use :class:`~vyra_base.com.handler.factory.HandlerFactory` to create handlers
without importing protocol-specific modules directly.
"""

from vyra_base.com.handler.interfaces import IFeederHandler

try:
    from vyra_base.com.handler.communication import CommunicationHandler
except ImportError:
    CommunicationHandler = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.database import DBCommunicationHandler, DatabaseWriter
except ImportError:
    DBCommunicationHandler = None  # type: ignore[assignment,misc]
    DatabaseWriter = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.ros2 import ROS2Handler
except ImportError:
    ROS2Handler = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.zenoh import ZenohHandler
except ImportError:
    ZenohHandler = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.redis import RedisHandler
except ImportError:
    RedisHandler = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.uds import UDSHandler
except ImportError:
    UDSHandler = None  # type: ignore[assignment,misc]

try:
    from vyra_base.com.handler.factory import HandlerFactory
except ImportError:
    HandlerFactory = None  # type: ignore[assignment,misc]

__all__ = [
    # Abstract interface
    "IFeederHandler",
    # Base class
    "CommunicationHandler",
    # Transport handlers
    "ROS2Handler",
    "ZenohHandler",
    "RedisHandler",
    "UDSHandler",
    # Database handler
    "DBCommunicationHandler",
    "DatabaseWriter",
    # Factory
    "HandlerFactory",
]

