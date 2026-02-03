"""
Redis Transport Module

Provides Redis-based transport implementation with layered architecture:

Layers:
    - communication/: Core Redis functionality (RedisClient, connection handling)
    - vyra_models/: VYRA abstractions (RedisCallable, RedisSpeaker)
    - provider.py: Interface layer for VYRA integration

Features:
    - Request-Response pattern (RedisCallable)
    - Pub/Sub pattern (RedisSpeaker)
    - Redis client with connection pooling
    - TLS/SSL support
    - Automatic reconnection

Usage:
    from vyra_base.com.transport.redis import RedisProvider, REDIS_AVAILABLE
    
    if REDIS_AVAILABLE:
        provider = RedisProvider(
            host="redis.example.com",
            port=6379,
            ssl=True
        )
        await provider.initialize()
"""
import logging

logger = logging.getLogger(__name__)

# Try importing communication layer (Redis client)
try:
    from vyra_base.com.transport.redis.communication import (
        RedisClient,
        REDIS_TYPE,
        REDIS_COMMUNICATION_AVAILABLE,
    )
    _communication_available = REDIS_COMMUNICATION_AVAILABLE
except ImportError as e:
    RedisClient = None
    REDIS_TYPE = None
    _communication_available = False
    logger.debug(f"⚠️  Redis communication layer unavailable: {e}")

# Try importing VYRA models layer
try:
    from vyra_base.com.transport.redis.vyra_models import (
        RedisCallable,
        RedisSpeaker,
        REDIS_MODELS_AVAILABLE,
    )
    _models_available = REDIS_MODELS_AVAILABLE
except ImportError as e:
    RedisCallable = None
    RedisSpeaker = None
    _models_available = False
    logger.debug(f"⚠️  Redis VYRA models layer unavailable: {e}")

# Try importing provider (interface layer)
try:
    from vyra_base.com.transport.redis.provider import RedisProvider
    _provider_available = True
except ImportError as e:
    RedisProvider = None
    _provider_available = False
    logger.debug(f"⚠️  Redis provider unavailable: {e}")

# Redis is fully available if all layers are available
REDIS_AVAILABLE = _communication_available and _models_available and _provider_available

if REDIS_AVAILABLE:
    logger.info("✅ Redis transport fully available (communication + models + provider)")
elif _communication_available:
    logger.info("⚠️  Redis transport partially available (communication only)")
else:
    logger.debug("❌ Redis transport unavailable")

__all__ = [
    # Interface layer
    "RedisProvider",
    # Communication layer
    "RedisClient",
    "REDIS_TYPE",
    # VYRA models layer
    "RedisCallable",
    "RedisSpeaker",
    # Availability flags
    "REDIS_AVAILABLE",
]
