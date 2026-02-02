"""
Redis External Communication

Professional Redis integration for vyra_base communication layer.
Wraps existing storage/redis_client.py with provider pattern.
"""
from vyra_base.com.external.redis.provider import RedisProvider

__all__ = [
    "RedisProvider",
]
