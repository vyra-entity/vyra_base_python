"""
Redis Communication Layer

Functional layer providing direct Redis protocol operations.
This layer implements the core Redis functionality without VYRA abstractions.

Available:
- RedisClient: Full-featured Redis client with Pub/Sub, Streams, Key-Value
- REDIS_TYPE: Enum for Redis data types

Example:
    >>> from vyra_base.com.transport.redis.communication import RedisClient, REDIS_TYPE
    >>> 
    >>> client = RedisClient(module_name="my_module")
    >>> await client.connect()
    >>> 
    >>> # Key-Value operations
    >>> await client.set("sensor:temp", 23.5)
    >>> value = await client.get("sensor:temp")
    >>> 
    >>> # Pub/Sub
    >>> await client.publish_message("events", {"type": "alarm"})
    >>> 
    >>> # Streams
    >>> await client.xadd("log_stream", {"level": "info", "msg": "Started"})
"""

try:
    from vyra_base.com.transport.redis.communication.redis_client import (
        RedisClient,
        REDIS_TYPE,
    )
    REDIS_COMMUNICATION_AVAILABLE = True
except ImportError:
    RedisClient = None
    REDIS_TYPE = None
    REDIS_COMMUNICATION_AVAILABLE = False

__all__ = [
    "RedisClient",
    "REDIS_TYPE",
    "REDIS_COMMUNICATION_AVAILABLE",
]
