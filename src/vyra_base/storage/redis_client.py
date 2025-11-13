"""
Unified Redis Client for vyra_base
Combines functionality from redis_access.py and redis_manipulator.py
with TLS support and streaming capabilities for industrial applications
"""

import os
import ssl
import asyncio
from pathlib import Path
from datetime import datetime
from typing import Any, Dict, List, Optional, Union
from enum import Enum

import redis.asyncio as redis
from redis.asyncio.client import PubSub

from vyra_base.storage.storage import Storage
from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback


class REDIS_TYPE(str, Enum):
    """Redis data types supported by the client."""
    STRING = "string"
    HASH = "hash"
    LIST = "list"
    SET = "set"


class RedisClient(Storage):
    """
    Unified Redis Client for vyra_base
    Combines RedisAccess and RedisManipulator functionality
    with TLS support and streaming capabilities for professional communication
    """

    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        module_name: str,
        host: str = None,
        port: int = None,
        username: str = None,
        password: str = None,
        db: int = 0,
        use_tls: bool = None,
        redis_config_path: Optional[str] = None,
        redis_config: Optional[dict] = None
    ):
        """
        Initialize Redis client with TLS support
        
        Args:
            module_name: Name of the module using this client
            host: Redis server hostname (default from env REDIS_HOST)
            port: Redis server port (default from env REDIS_PORT)
            username: Redis ACL username (default from env REDIS_USERNAME)
            password: Redis ACL password (default from env REDIS_PASSWORD)
            db: Redis database number
            use_tls: Enable TLS (default from env REDIS_TLS_ENABLED)
            redis_config_path: Path to Redis config file (legacy support)
            redis_config: Redis configuration dict (legacy support)
        """
        self.module_name = module_name
        
        # Handle legacy configuration format
        if redis_config_path or redis_config:
            self._load_legacy_config(redis_config_path, redis_config)
        else:
            self._load_environment_config(host, port, username, password, db, use_tls)
        
        self._redis_engine: Optional[redis.Redis] = None
        self._pubsub: Optional[PubSub] = None
        self._connected = False

    def _load_legacy_config(self, redis_config_path: Optional[str], redis_config: Optional[dict]):
        """Load configuration from legacy format (INI file or dict)."""
        if redis_config_path is not None:
            import configparser
            config = configparser.ConfigParser()
            config.read(redis_config_path)
            config_dict = {section: dict(config.items(section)) 
                          for section in config.sections()}
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
        self.host = redis_section.get('host', 'localhost')
        self.port = int(redis_section.get('port', '6379'))
        self.username = redis_section.get('username', None)
        self.password = redis_section.get('password', None)
        self.db = 0
        self.use_tls = redis_section.get('use_tls', 'false').lower() == 'true'
        
        # TLS configuration from config
        self.ca_cert = redis_section.get('ca_cert')
        self.client_cert = redis_section.get('client_cert')
        self.client_key = redis_section.get('client_key')

    def _load_environment_config(
        self, 
        host: Optional[str], 
        port: Optional[int], 
        username: Optional[str], 
        password: Optional[str], 
        db: int, 
        use_tls: Optional[bool]
    ):
        """Load configuration from environment variables."""
        # Get configuration from environment with fallbacks
        self.host = host or os.getenv('REDIS_HOST', 'redis')
        self.port = port or int(os.getenv('REDIS_PORT', '6379'))
        self.username = username or os.getenv('REDIS_USERNAME', 'modulemanager')
        self.password = password or os.getenv('REDIS_PASSWORD', 'vyra_modulemanager_secure_2024')
        self.db = db
        
        # TLS configuration
        tls_enabled = use_tls if use_tls is not None else os.getenv('REDIS_TLS_ENABLED', 'true').lower() == 'true'
        self.use_tls = tls_enabled
        
        # Certificate paths
        cert_path = Path(os.getenv('CERTIFICATES_PATH', '/workspace/storage/certificates'))
        redis_cert_path = cert_path / 'redis'
        
        self.ca_cert = str(redis_cert_path / 'ca-cert.pem')
        self.client_cert = str(redis_cert_path / 'client-cert.pem')
        self.client_key = str(redis_cert_path / 'client-key.pem')

    async def _ensure_connected(self) -> redis.Redis:
        """
        Ensure Redis connection is established and return the client
        
        Returns:
            redis.Redis: Connected Redis client
            
        Raises:
            RuntimeError: If connection fails
        """
        if not self._connected or self._redis_engine is None:
            await self.connect()
        
        if self._redis_engine is None:
            raise RuntimeError("Redis connection failed")
        
        return self._redis_engine

    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish connection to Redis server."""
        try:
            # Create Redis client with TLS if enabled
            ssl_params = {}
            if self.use_tls:
                ssl_params.update({
                    'ssl': True,
                    'ssl_ca_certs': self.ca_cert,
                    'ssl_certfile': self.client_cert,
                    'ssl_keyfile': self.client_key,
                    'ssl_cert_reqs': 'required',
                    'ssl_check_hostname': False  # Disable hostname verification for private CAs
                })
            
            client = redis.Redis(
                host=self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                db=self.db,
                decode_responses=True,
                **ssl_params
            )
            
            # Test connection
            await client.ping()
            
            self._redis_engine = client
            self._connected = True
            
            # Initialize PubSub if needed
            if self._pubsub is None:
                self._pubsub = client.pubsub()
            
            Logger.info(f"✅ Redis connected for {self.module_name} at {self.host}:{self.port} (TLS: {self.use_tls})")
            
        except Exception as e:
            self._connected = False
            self._redis_engine = None
            Logger.error(f"❌ Failed to connect to Redis: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def configure_base_settings(self):
        """Configure Redis base settings like maxmemory and logfile."""
        client = await self._ensure_connected()
        
        try:
            # Set memory limit if configured
            if hasattr(self, '_config') and 'redis' in self._config:
                maxmemory = self._config['redis'].get('maxmemory')
                if maxmemory:
                    Logger.info(f"Setting Redis maxmemory to {maxmemory}")
                    await client.config_set("maxmemory", maxmemory)
        except Exception as e:
            err_msg = f"Failed to configure Redis settings: {e}"
            Logger.error(err_msg)
            raise RuntimeError(err_msg)

    @ErrorTraceback.w_check_error_exist
    async def close(self):
        """Close the Redis connection."""
        if self._pubsub:
            await self._pubsub.close()
            self._pubsub = None
        
        if self._redis_engine:
            await self._redis_engine.close()
            self._redis_engine = None
        
        self._connected = False
        Logger.debug(f"Redis connection closed for {self.module_name}")

    # ======================
    # STORAGE INTERFACE
    # ======================

    @ErrorTraceback.w_check_error_exist
    async def get(self, key: str) -> Any:
        """
        Get a value from Redis.
        
        Args:
            key: The key to retrieve
            
        Returns:
            The value associated with the key, or None if not found
        """
        client = await self._ensure_connected()
        
        try:
            # First check what type of data structure this is
            key_type = await client.type(key)
            
            if key_type == "none":
                return None
            elif key_type == "string":
                return await client.get(key)
            elif key_type == "hash":
                return await client.hgetall(key)
            elif key_type == "list":
                return await client.lrange(key, 0, -1)
            elif key_type == "set":
                return list(await client.smembers(key))
            else:
                # Fallback to string
                return await client.get(key)
                
        except Exception as e:
            Logger.error(f"Error getting key {key}: {e}")
            return None

    @ErrorTraceback.w_check_error_exist
    async def set(self, key: str, value: Any) -> bool:
        """
        Set a value in Redis.
        
        Args:
            key: The key to set
            value: The value to set
            
        Returns:
            True if the operation was successful
        """
        client = await self._ensure_connected()
        
        try:
            if isinstance(value, dict):
                # Store as hash
                await client.delete(key)  # Clear existing data
                result = await client.hset(key, mapping=value)
                return result > 0
            elif isinstance(value, list):
                # Store as list
                await client.delete(key)  # Clear existing data
                if value:  # Only push if list is not empty
                    result = await client.rpush(key, *value)
                    return result > 0
                return True
            elif isinstance(value, set):
                # Store as set
                await client.delete(key)  # Clear existing data
                if value:  # Only add if set is not empty
                    result = await client.sadd(key, *value)
                    return result > 0
                return True
            else:
                # Store as string
                result = await client.set(key, value)
                return result is not None
        except Exception as e:
            Logger.error(f"Error setting key {key}: {e}")
            return False

    @ErrorTraceback.w_check_error_exist
    async def delete(self, key: str) -> bool:
        """
        Delete a key from Redis.
        
        Args:
            key: The key to delete
            
        Returns:
            True if the key was deleted, False if it did not exist
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.delete(key)
            return result > 0
        except Exception as e:
            Logger.error(f"Error deleting key {key}: {e}")
            return False

    @ErrorTraceback.w_check_error_exist
    async def exists(self, key: str) -> bool:
        """
        Check if a key exists in Redis.
        
        Args:
            key: The key to check
            
        Returns:
            True if the key exists, False otherwise
        """
        client = await self._ensure_connected()
        return await client.exists(key) > 0

    @ErrorTraceback.w_check_error_exist
    async def clear(self) -> bool:
        """
        Clear all keys in the Redis database.
        
        Returns:
            True if the operation was successful, False otherwise
        """
        client = await self._ensure_connected()
        
        try:
            await client.flushdb()
            return True
        except Exception as e:
            Logger.error(f"Error clearing Redis database: {e}")
            return False

    @ErrorTraceback.w_check_error_exist
    async def get_all_keys(self) -> list[str]:
        """
        Get all keys in the Redis database.
        
        Returns:
            A list of all keys
        """
        client = await self._ensure_connected()
        
        try:
            return await client.keys('*')
        except Exception as e:
            Logger.error(f"Error retrieving keys from Redis: {e}")
            return []

    @ErrorTraceback.w_check_error_exist
    async def get_keys_by_pattern(self, pattern: str) -> list[str]:
        """
        Get keys matching a specific pattern in the Redis database.
        
        Args:
            pattern: The pattern to match keys against
            
        Returns:
            A list of keys matching the pattern
        """
        client = await self._ensure_connected()
        
        try:
            return await client.keys(pattern)
        except Exception as e:
            Logger.error(f"Error retrieving keys with pattern {pattern}: {e}")
            return []

    @ErrorTraceback.w_check_error_exist
    async def get_type(self, key: str) -> Optional[REDIS_TYPE]:
        """
        Get the type of a key in Redis.
        
        Args:
            key: The key to check
            
        Returns:
            The type of the key, or None if it does not exist
        """
        client = await self._ensure_connected()
        
        try:
            type_str = await client.type(key)
            if type_str == "none":
                return None
            return REDIS_TYPE(type_str) if type_str in [e.value for e in REDIS_TYPE] else None
        except Exception as e:
            Logger.error(f"Error retrieving type for key {key}: {e}")
            return None

    @ErrorTraceback.w_check_error_exist
    async def get_length(self, key: str, redis_type: REDIS_TYPE) -> int:
        """
        Get the length of a key in Redis.
        
        Args:
            key: The key to check
            redis_type: The type of the Redis data structure
            
        Returns:
            The length of the key, or -1 if it does not exist
        """
        client = await self._ensure_connected()
        
        try:
            match redis_type:
                case REDIS_TYPE.STRING:
                    return await client.strlen(key)
                case REDIS_TYPE.HASH:
                    return await client.hlen(key)
                case REDIS_TYPE.LIST:
                    return await client.llen(key)
                case REDIS_TYPE.SET:
                    return await client.scard(key)
                case _:
                    Logger.error(f"Unsupported Redis type: {redis_type}")
                    return -1
        except Exception as e:
            Logger.error(f"Error getting length for key {key}: {e}")
            return -1

    # ======================
    # PUB/SUB FUNCTIONALITY
    # ======================

    @ErrorTraceback.w_check_error_exist
    async def publish_message(self, channel: str, message: Union[str, Dict[str, Any]]) -> int:
        """
        Publish message to Redis channel
        
        Args:
            channel: Channel name
            message: Message content (string or dict)
            
        Returns:
            Number of clients that received the message
        """
        client = await self._ensure_connected()
        
        try:
            if isinstance(message, dict):
                import json
                message = json.dumps(message)
            
            result = await client.publish(channel, message)
            Logger.debug(f"📤 Published to {channel}: {message}")
            return result
        except Exception as e:
            Logger.error(f"❌ Failed to publish message to {channel}: {e}")
            return 0

    @ErrorTraceback.w_check_error_exist
    async def subscribe_channel(self, channel: str):
        """
        Subscribe to Redis channel
        
        Args:
            channel: Channel name to subscribe to
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        try:
            await self._pubsub.subscribe(channel)
            Logger.debug(f"📥 Subscribed to channel: {channel}")
        except Exception as e:
            Logger.error(f"❌ Failed to subscribe to {channel}: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def subscribe_to_key(self, key: str) -> None:
        """
        Subscribe to changes on a specific key in Redis.
        
        Args:
            key: The key to subscribe to
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        try:
            await self._pubsub.subscribe(key)
        except Exception as e:
            Logger.error(f"Error subscribing to key {key}: {e}")

    @ErrorTraceback.w_check_error_exist
    async def unsubscribe_from_key(self, key: str) -> None:
        """
        Unsubscribe from changes on a specific key in Redis.
        
        Args:
            key: The key to unsubscribe from
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        try:
            await self._pubsub.unsubscribe(key)
        except Exception as e:
            Logger.error(f"Error unsubscribing from key {key}: {e}")

    @ErrorTraceback.w_check_error_exist
    async def health_check(self) -> bool:
        """
        Check Redis connection health
        
        Returns:
            True if healthy, False otherwise
        """
        try:
            if not self._connected:
                await self.connect()
            client = await self._ensure_connected()
            await client.ping()
            return True
        except Exception:
            return False

    # ======================
    # STREAMING & INDUSTRIAL COMMUNICATION
    # ======================
    
    @ErrorTraceback.w_check_error_exist
    async def subscribe_pattern(self, pattern: str) -> PubSub:
        """
        Subscribe to Redis channel pattern for backend communication
        
        Args:
            pattern: Channel pattern (e.g., "modulemanager_*", "ros2_*")
            
        Returns:
            PubSub object for listening to messages
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        try:
            await self._pubsub.psubscribe(pattern)
            Logger.debug(f"📥 Subscribed to pattern: {pattern}")
            return self._pubsub
        except Exception as e:
            Logger.error(f"❌ Failed to subscribe to pattern {pattern}: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def publish_event(self, 
                          channel: str, 
                          data: Dict[str, Any],
                          add_metadata: bool = True) -> int:
        """
        Publish event to a Redis channel with optional metadata
        
        Args:
            channel: Channel name (e.g., "modulemanager_register", "ros2_node_state")
            data: Event data
            add_metadata: Whether to add timestamp and source metadata
            
        Returns:
            Number of clients that received the message
        """
        if add_metadata:
            enhanced_data = {
                "timestamp": datetime.now().isoformat(),
                "source_module": self.module_name,
                "data": data
            }
            return await self.publish_message(channel, enhanced_data)
        else:
            return await self.publish_message(channel, data)

    @ErrorTraceback.w_check_error_exist
    async def create_stream_listener(self, 
                                   channels: list[str], 
                                   callback_handler,
                                   callback_context=None) -> None:
        """
        Create a persistent stream listener for industrial communication
        
        Args:
            channels: List of channels/patterns to listen to
            callback_handler: Async function to handle incoming messages (signature: async def handler(message, context))
            callback_context: Optional context object passed to callback (e.g., PermissionManager instance)
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        try:
            # Subscribe to channels
            for channel in channels:
                if "*" in channel:
                    await self._pubsub.psubscribe(channel)
                    Logger.debug(f"📥 Pattern subscribed: {channel}")
                else:
                    await self._pubsub.subscribe(channel)
                    Logger.debug(f"📥 Channel subscribed: {channel}")
            
            Logger.info(f"🎧 Stream listener created for channels: {channels}")
            
            # Start listening loop
            if callback_handler:
                await self._start_message_loop(callback_handler, callback_context)
            else:
                Logger.warning("⚠️ No callback handler provided for stream listener")
                
        except Exception as e:
            Logger.error(f"❌ Failed to create stream listener: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def _start_message_loop(self, callback_handler, callback_context):
        """
        Start the message processing loop for industrial-grade communication
        
        Args:
            callback_handler: Async function to process messages
            callback_context: Optional context object for message handling
        """
        if self._pubsub is None:
            raise RuntimeError("PubSub not available")
        
        try:
            Logger.info("🔄 Starting industrial Redis message loop...")
            
            async for message in self._pubsub.listen():
                if message["type"] in ["message", "pmessage"]:
                    try:
                        # Process message through callback
                        if asyncio.iscoroutinefunction(callback_handler):
                            await callback_handler(message, callback_context)
                        else:
                            callback_handler(message, callback_context)
                            
                        Logger.debug(f"📨 Processed message from {message.get('channel', 'unknown')}")
                        
                    except Exception as e:
                        Logger.error(f"❌ Error processing message: {e}")
                        # Continue processing other messages
                        
        except Exception as e:
            Logger.error(f"❌ Message loop error: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def parse_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse and normalize a Redis message for easier processing
        
        Args:
            message: Raw Redis message
            
        Returns:
            Normalized message dictionary with channel, data, and pattern info
        """
        try:
            channel = message.get("channel", "")
            if isinstance(channel, bytes):
                channel = channel.decode('utf-8')
            
            data = message.get("data", "")
            if isinstance(data, bytes):
                data = data.decode('utf-8')
            
            # Try to parse JSON
            import json
            try:
                parsed_data = json.loads(data) if isinstance(data, str) and data.startswith('{') else {"raw_data": data}
            except json.JSONDecodeError:
                parsed_data = {"raw_data": data}
            
            # Ensure parsed_data is a dict
            if not isinstance(parsed_data, dict):
                parsed_data = {"data": parsed_data}
            
            return {
                "channel": channel,
                "pattern": message.get("pattern", "").decode('utf-8') if isinstance(message.get("pattern"), bytes) else message.get("pattern", ""),
                "type": message.get("type", ""),
                "data": parsed_data
            }
            
        except Exception as e:
            Logger.error(f"❌ Error parsing message: {e}")
            return {"channel": "unknown", "data": {}, "error": str(e)}


# Legacy compatibility wrappers
RedisAccess = RedisClient
RedisManipulator = RedisClient