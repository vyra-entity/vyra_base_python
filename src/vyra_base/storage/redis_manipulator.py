"""
DEPRECATED: This module has been replaced by redis_client.py
Please use RedisClient from vyra_base.storage.redis_client instead

This file provides backward compatibility only
"""

import warnings
from vyra_base.storage.redis_client import RedisClient, REDIS_TYPE

# Backward compatibility warning
warnings.warn(
    "RedisManipulator is deprecated. Use RedisClient from vyra_base.storage.redis_client instead.",
    DeprecationWarning,
    stacklevel=2
)

# Backward compatibility aliases
RedisManipulator = RedisClient