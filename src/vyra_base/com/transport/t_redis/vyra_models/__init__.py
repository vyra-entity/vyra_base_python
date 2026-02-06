"""
Redis VYRA Models

VYRA abstraction layer for Redis transport.
Implements Callable, Speaker, and Job patterns for unified interface.

Available:
- RedisCallable: Request-response pattern via Redis key-value
- RedisSpeaker: Pub/Sub messaging via Redis channels
- RedisJob: Long-running tasks via Redis Streams (future)

Example:
    >>> from vyra_base.com.transport.t_redis.vyra_models import RedisCallable, RedisSpeaker
    >>> 
    >>> # Callable (request-response)
    >>> async def handle_request(req):
    ...     return {"result": req["value"] * 2}
    >>> 
    >>> callable = RedisCallable(
    ...     name="calculate",
    ...     callback=handle_request,
    ...     redis_client=client
    ... )
    >>> await callable.initialize()
    >>> 
    >>> # Speaker (pub/sub)
    >>> speaker = RedisSpeaker(
    ...     name="sensor_updates",
    ...     redis_client=client
    ... )
    >>> await speaker.initialize()
    >>> await speaker.shout({"temperature": 23.5})
"""

try:
    from vyra_base.com.transport.t_redis.vyra_models.callable import RedisCallable
    from vyra_base.com.transport.t_redis.vyra_models.speaker import RedisSpeaker
    REDIS_MODELS_AVAILABLE = True
except ImportError:
    RedisCallable = None
    RedisSpeaker = None
    REDIS_MODELS_AVAILABLE = False

__all__ = [
    "RedisCallable",
    "RedisSpeaker",
    "REDIS_MODELS_AVAILABLE",
]
