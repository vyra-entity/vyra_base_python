from vyra_base.storage.db_access import DBTYPE
from vyra_base.storage.storage import Storage
from vyra_base.helper.error_handler import ErrorTraceback
import configparser
import redis.asyncio as redis


class RedisAccess(Storage):
    """Redis access class."""

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, 
            module_name: str,
            redis_config_path: str = None,
            redis_config: dict = None) -> None:

        self.module_name = module_name

        if redis_config_path is not None and redis_config is None:
            self._config = configparser.ConfigParser()
            self._config.read(redis_config_path)
        elif redis_config is not None:
            redis_config = redis_config[0] if isinstance(redis_config, list) else redis_config

            if not isinstance(redis_config, dict):
                raise ValueError("redis_config must be a dictionary.")

            self._config = redis_config
        else:
            raise ValueError("Either redis_config_path or redis_config must be provided.")
        
        if not 'redis' in self._config:
            raise ValueError("redis_config must contain a 'redis' section.")
        
        self._host: str | None = self._config['redis'].get('host', 'localhost')
        self._port: int | None = int(self._config['redis'].get('port', '6379'))
        self._requirepass: str | None = self._config['redis'].get('requirepass', None)
        self._logfile: str | None = self._config['redis'].get('logfile', None)
        self._maxmemory: str | None = self._config['redis'].get('maxmemory', None)

        self._redis_engine = redis.Redis(
            host=self._host,
            port=self._port,
            password=self._requirepass,
            decode_responses=True
        )

        # Speicherlimit auf 256 MB setzen
        if self._maxmemory:
            self._redis_engine.config_set("maxmemory", self._maxmemory)
        
        # Logfile setzen (falls Redis nicht als daemon läuft, hat das evtl. keine Wirkung)
        if self._logfile:
            self._redis_engine.config_set("logfile", self._logfile)

    @property
    def redis(self) -> redis.Redis:
        """
        Get the Redis client instance.
        :return: The Redis client instance.
        :rtype: redis.Redis
        """
        return self._redis_engine