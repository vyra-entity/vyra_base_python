from __future__ import annotations

from uuid import UUID
from enum import Enum
from typing import Any, Awaitable
from typing import Callable

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.storage.redis_access import RedisAccess

import redis.asyncio as redis


class REDIS_TYPE(str, Enum):
    STRING = "string"
    HASH = "hash"
    LIST = "list"
    SET = "set"


class RedisManipulator:
    """Datatable class manipulator
    for Redis data structures.
    This class provides methods to manipulate Redis data structures
    such as strings, hashes, lists, sets, and sorted sets.
    Provided attributes are:
    - redis: The Redis client instance.
    - pubsub: The Redis pubsub instance for subscribing to key changes.
    - module_id: The ID of the V.Y.R.A. module.

    Provided methods are:
    - get: Get a value from Redis.
    - set: Set a value in Redis.
    - delete: Delete a key from Redis.
    - exists: Check if a key exists in Redis.
    - clear: Clear all keys in the Redis database.
    - get_all_keys: Get all keys in the Redis database.
    - get_keys_by_pattern: Get keys matching a specific pattern in the Redis database.
    - get_type: Get the type of a key in Redis.
    - get_length: Get the length of a key in Redis.
    - subscribe_to_key: Subscribe to changes on a specific key in Redis.
    """

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, redis_access: RedisAccess, module_id: UUID):
        """
        Initialize datatable.

        :param redis_access: Redis access object.
        :type redis_access: RedisAccess
        :param model: SQLAlchemy model class.
        :type model: Type[Base]
        :param module_id: The id of the V.Y.R.A. module.
        :type module_id: str
        """
        if not isinstance(redis_access, RedisAccess):
            raise TypeError('redis_access must be of type RedisAccess.')

        self.redis: redis.Redis = redis_access.redis
        self.pubsub = self.redis.pubsub()
        self.module_id = module_id

    @ErrorTraceback.w_check_error_exist
    async def get(self, key: str) -> Any:
        """
        Get a value from Redis.
        :param key: The key to retrieve.
        :type key: str
        :return: The value associated with the key, or None if not found.
        :rtype: str | None
        """
        value_type = await self.redis.type(key)

        if value_type not in REDIS_TYPE:
            raise ValueError(
                f"Invalid Redis type: {value_type}. "
                f"Must be one of {list(REDIS_TYPE)}.")

        match value_type:
            case REDIS_TYPE.STRING:
                return await self.redis.get(key)
            case REDIS_TYPE.HASH:
                return self.redis.hgetall(key)
            case REDIS_TYPE.LIST:
                return self.redis.lrange(key, 0, -1)
            case REDIS_TYPE.SET:
                return self.redis.smembers(key)
            case _:
                raise ValueError(
                    f"Unsupported Redis type: {type}. "
                    f"Must be one of {list(REDIS_TYPE)}.")
        
        return await self.redis.get(key)

    @ErrorTraceback.w_check_error_exist
    async def set(self, key: str, value: Any) -> str:
        """
        Set a value in Redis.
        :param key: The key to set.
        :type key: str
        :param value: The value to set.
        :type value: Any
        :return: True if the operation was successful, False otherwise.
        :rtype: bool
        """
        value_type = type(value)
        if value_type not in REDIS_TYPE:
            value_type = REDIS_TYPE.STRING  # Default to STRING if type is not specified

        match value_type:
            case REDIS_TYPE.STRING:
                return await self.redis.set(key, value)
            case REDIS_TYPE.HASH:
                return str(self.redis.hset(key, mapping=value))
            case REDIS_TYPE.LIST:
                return str(self.redis.rpush(key, *value))
            case REDIS_TYPE.SET:
                return str(self.redis.sadd(key, *value))
            case _:
                raise ValueError(
                    f"Unsupported Redis type: {value_type}. "
                    f"Must be one of {list(REDIS_TYPE)}.")

    @ErrorTraceback.w_check_error_exist
    async def delete(self, key: str) -> bool:
        """
        Delete a key from Redis.
        
        :param key: The key to delete.
        :type key: str
        :return: True if the key was deleted, False if it did not exist.
        :rtype: bool
        """
        try:
            result = await self.redis.delete(key)
            return result > 0
        except redis.RedisError as e:
            Logger.error(f"Error deleting key {key}: {e}")
            return False
        
    @ErrorTraceback.w_check_error_exist
    async def exists(self, key: str) -> bool:
        """
        Check if a key exists in Redis.
        
        :param key: The key to check.
        :type key: str
        :return: True if the key exists, False otherwise.
        :rtype: bool
        """
        return await self.redis.exists(key) > 0
    
    @ErrorTraceback.w_check_error_exist
    async def clear(self) -> bool:
        """
        Clear all keys in the Redis database.
        
        :return: True if the operation was successful, False otherwise.
        :rtype: bool
        """
        try:
            await self.redis.flushdb()
            return True
        except redis.RedisError as e:
            Logger.error(f"Error clearing Redis database: {e}")
            return False
        
    @ErrorTraceback.w_check_error_exist
    async def get_all_keys(self) -> list[str]:
        """
        Get all keys in the Redis database.
        
        :return: A list of all keys.
        :rtype: list[str]
        """
        try:
            return await self.redis.keys('*')
        except redis.RedisError as e:
            Logger.error(f"Error retrieving keys from Redis: {e}")
            return []
        
    @ErrorTraceback.w_check_error_exist
    async def get_keys_by_pattern(self, pattern: str) -> list[str]:
        """
        Get keys matching a specific pattern in the Redis database.
        
        :param pattern: The pattern to match keys against.
        :type pattern: str
        :return: A list of keys matching the pattern.
        :rtype: list[str]
        """
        try:
            return await self.redis.keys(pattern)
        except redis.RedisError as e:
            Logger.error(f"Error retrieving keys with pattern {pattern}: {e}")
            return []
        
    @ErrorTraceback.w_check_error_exist
    async def get_type(self, key: str) -> REDIS_TYPE | None:
        """
        Get the type of a key in Redis.
        
        :param key: The key to check.
        :type key: str
        :return: The type of the key, or None if it does not exist.
        :rtype: REDIS_TYPE | None
        """
        try:
            type_str = await self.redis.type(key)
            return REDIS_TYPE(type_str) if type_str in REDIS_TYPE.__members__ else None
        except redis.RedisError as e:
            Logger.error(f"Error retrieving type for key {key}: {e}")
            return None
        
    @ErrorTraceback.w_check_error_exist
    async def get_length(self, key: str, type: REDIS_TYPE) -> int:
        """
        Get the length of a key in Redis.
        
        :param key: The key to check.
        :type key: str
        :param type: The type of the Redis data structure.
        :type type: REDIS_TYPE
        :return: The length of the key, or -1 if it does not exist.
        :rtype: int
        """
        if type not in REDIS_TYPE:
            raise ValueError(
                f"Invalid Redis type: {type}. "
                f"Must be one of {list(REDIS_TYPE)}.")
        
        match type:
            case REDIS_TYPE.STRING:
                return await self.redis.strlen(key)
            case REDIS_TYPE.HASH:
                return await self.redis.hlen(key) # pyright: ignore[reportGeneralTypeIssues]
            case REDIS_TYPE.LIST:
                return await self.redis.llen(key) # pyright: ignore[reportGeneralTypeIssues]
            case REDIS_TYPE.SET:
                return await self.redis.scard(key) # pyright: ignore[reportGeneralTypeIssues]
            case _:
                raise ValueError(
                    f"Unsupported Redis type: {type}. "
                    f"Must be one of {list(REDIS_TYPE)}.")

    async def subscribe_to_key(self, key: str) -> None:
        """
        Subscribe to changes on a specific key in Redis.
        
        :param key: The key to subscribe to.
        :type key: str
        :param callback: The callback function to call when the key changes.
        :type callback: Callable
        """
        await self.pubsub.subscribe(**{key: callback})

    async def unsubscribe_from_key(self, key: str) -> None:
        """
        Unsubscribe from changes on a specific key in Redis.
        
        :param key: The key to unsubscribe from.
        :type key: str
        :param callback: The callback function to remove from the subscription.
        :type callback: Callable
        """
        await self.pubsub.unsubscribe(key)