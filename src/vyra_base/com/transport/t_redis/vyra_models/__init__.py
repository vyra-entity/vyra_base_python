"""
Redis VYRA Models

VYRA abstraction layer for Redis transport.

Legacy patterns (deprecated):
- Callable: Request-response via key-value
- Speaker: Pub/Sub via channels
- Job: Long-running tasks via Pub/Sub + key-value

Unified transport layer:
- Publisher/Subscriber (pub/sub)
- Server/Client (request/response via key-value)
- ActionServer/ActionClient (long-running tasks with feedback)

Example:
    >>> from vyra_base.com.transport.t_redis.vyra_models import (
    ...     RedisPublisherImpl, RedisSubscriberImpl, RedisServerImpl
    ... )
    >>> 
    >>> # Publisher
    >>> publisher = RedisPublisherImpl(
    ...     name="sensor_data",
    ...     topic_builder=builder,
    ...     message_type=SensorMsg,
    ...     redis_client=client
    ... )
    >>> await publisher.initialize()
    >>> await publisher.publish({"temperature": 23.5})
    >>> 
    >>> # Server
    >>> async def handle_request(req):
    ...     return {"result": req["value"] * 2}
    >>> 
    >>> server = RedisServerImpl(
    ...     name="calculate",
    ...     topic_builder=builder,
    ...     response_callback=handle_request,
    ...     service_type=CalcService,
    ...     redis_client=client
    ... )
    >>> await server.initialize()
"""

try:

    
    # Unified transport layer imports
    from vyra_base.com.transport.t_redis.vyra_models.publisher import RedisPublisherImpl
    from vyra_base.com.transport.t_redis.vyra_models.subscriber import RedisSubscriberImpl
    from vyra_base.com.transport.t_redis.vyra_models.server import RedisServerImpl
    from vyra_base.com.transport.t_redis.vyra_models.client import RedisClientImpl
    from vyra_base.com.transport.t_redis.vyra_models.action_server import RedisActionServerImpl
    from vyra_base.com.transport.t_redis.vyra_models.action_client import RedisActionClientImpl
    
    REDIS_MODELS_AVAILABLE = True
except ImportError:
    # Unified
    RedisPublisherImpl = None
    RedisSubscriberImpl = None
    RedisServerImpl = None
    RedisClientImpl = None
    RedisActionServerImpl = None
    RedisActionClientImpl = None
    REDIS_MODELS_AVAILABLE = False

__all__ = [

    # Unified transport layer
    "RedisPublisherImpl",
    "RedisSubscriberImpl",
    "RedisServerImpl",
    "RedisClientImpl",
    "RedisActionServerImpl",
    "RedisActionClientImpl",
    # Availability flag
    "REDIS_MODELS_AVAILABLE",
]
