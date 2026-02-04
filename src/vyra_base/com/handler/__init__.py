"""
Communication handlers for V.Y.R.A. modules.

This package provides various communication handlers:
- gRPC over Unix Domain Socket (UDS) - moved to vyra_base.com.external.grpc
- ROS2 handlers for ROS2 node communication
- Database handlers for persistent storage
"""

try:
    from vyra_base.com.handler.database import DBCommunicationHandler
except ImportError:
    DBCommunicationHandler = None

try:
    from vyra_base.com.handler.ros2 import ROS2Handler
except ImportError:
    ROS2Handler = None

__all__ = [
    "DBCommunicationHandler",
    "ROS2Handler"
]
