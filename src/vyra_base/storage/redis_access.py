
import configparser
import redis.asyncio as redis
from typing import Optional, Union

from vyra_base.storage.storage import Storage
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger


class RedisAccess(Storage):
    """Redis access class."""

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, 
            module_name: str,
            redis_config_path: Optional[str] = None,
            redis_config: Optional[dict] = None) -> None:

        self.module_name = module_name

        if redis_config_path is not None and redis_config is None:
            self._config = configparser.ConfigParser()
            self._config.read(redis_config_path)
            # Convert ConfigParser to dict-like access
            config_dict = {section: dict(self._config.items(section)) 
                          for section in self._config.sections()}
            self._config = config_dict
        elif redis_config is not None:
            redis_config = redis_config[0] if isinstance(redis_config, list) else redis_config

            if not isinstance(redis_config, dict):
                raise ValueError("redis_config must be a dictionary.")

            self._config = redis_config
        else:
            raise ValueError("Either redis_config_path or redis_config must be provided.")
        
        if 'redis' not in self._config:
            raise ValueError("redis_config must contain a 'redis' section.")
        
        redis_section = self._config['redis']
        self._host: str = redis_section.get('host', 'localhost')
        self._port: int = int(redis_section.get('port', '6379'))
        self._requirepass: Optional[str] = redis_section.get('requirepass', None)
        self._logfile: Optional[str] = redis_section.get('logfile', None)
        self._maxmemory: Optional[str] = redis_section.get('maxmemory', None)

        # self._redis_engine = redis.Redis(
        #     host=self._host,
        #     port=self._port,
        #     password=self._requirepass,
        #     decode_responses=True
        # )

        self._redis_engine = redis.Redis(
            host=self._host,
            port=self._port,
            decode_responses=True
        )

    @ErrorTraceback.w_check_error_exist
    async def configure_base_settings(self):
        """Configure Redis base settings like maxmemory and logfile."""
        try:
            # Speicherlimit setzen
            if self._maxmemory:
                Logger.log(f"Setting Redis maxmemory to {self._maxmemory}")
                await self._redis_engine.config_set("maxmemory", self._maxmemory)

        except Exception as e:
            err_msg = f"Failed to configure Redis settings: {e}"
            raise RuntimeError(err_msg)

    @ErrorTraceback.w_check_error_exist
    async def close(self):
        """Close the Redis connection."""
        if self._redis_engine:
            await self._redis_engine.close()

    @property
    def redis(self) -> redis.Redis:
        """
        Get the Redis client instance.
        :return: The Redis client instance.
        :rtype: redis.Redis
        """
        return self._redis_engine

    async def __aenter__(self):
        """Async context manager entry."""
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()