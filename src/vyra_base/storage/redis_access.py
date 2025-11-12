
"""
DEPRECATED: This module has been replaced by redis_client.py
Please use RedisClient from vyra_base.storage.redis_client instead

This file provides backward compatibility only
"""

import warnings
from vyra_base.storage.redis_client import RedisClient

# Backward compatibility warning
warnings.warn(
    "RedisAccess is deprecated. Use RedisClient from vyra_base.storage.redis_client instead.",
    DeprecationWarning,
    stacklevel=2
)

# Backward compatibility alias
RedisAccess = RedisClient