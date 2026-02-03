# vyra_base.com.transport.redis

Redis Transport Module
=====================

Provides Redis-based transport implementation with layered architecture:

Layers:
    - communication/: Core Redis functionality (RedisClient, connection handling)
    - vyra_models/: VYRA abstractions (RedisCallable, RedisSpeaker)
    - provider.py: Interface layer for VYRA integration

**Features:**
- Request-Response pattern (RedisCallable)
- Pub/Sub pattern (RedisSpeaker)
- Redis client with connection pooling
- TLS/SSL support
- Automatic reconnection

**Usage:**

.. code-block:: python

    from vyra_base.com.transport.redis import RedisProvider, REDIS_AVAILABLE
    
    if REDIS_AVAILABLE:
        provider = RedisProvider(
            host="redis.example.com",
            port=6379,
            ssl=True
        )
        await provider.initialize()
