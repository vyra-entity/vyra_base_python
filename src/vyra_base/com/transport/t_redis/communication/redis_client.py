"""
Unified Redis Client for vyra_base
Combines functionality from redis_access.py and redis_manipulator.py
with TLS support and streaming capabilities for industrial applications
"""

import json
import os
import ssl
import asyncio
from pathlib import Path
from datetime import datetime
from typing import Any, Callable, Coroutine, Dict, List, Optional, Union
from enum import Enum

import redis.asyncio as redis
from redis.asyncio.client import PubSub

import logging
logger = logging.getLogger(__name__)
from vyra_base.helper.error_handler import ErrorTraceback


class REDIS_TYPE(str, Enum):
    """Redis data types supported by the client."""
    STRING = "string"
    HASH = "hash"
    LIST = "list"
    SET = "set"


class RedisClient():
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
        
        # Multi-listener management
        self._active_channels: set[str] = set()  # Track all subscribed channels/patterns
        self._listener_task: Optional[asyncio.Task] = None  # Single background task
        self._listener_running = False
        self._channel_callbacks: dict = {}  # Per-channel callback routing: channel -> (handler, context)

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
            
            logger.info(f"✅ Redis connected for {self.module_name} at {self.host}:{self.port} (TLS: {self.use_tls})")
            
        except Exception as e:
            self._connected = False
            self._redis_engine = None
            logger.error(f"❌ Failed to connect to Redis: {e}")
            raise

    async def ping(self) -> bool:
        """Ping Redis server to check connectivity."""
        if self._redis_engine is None:
            logger.warning("Redis client not initialized for ping")
            return False
        
        try:
            return await self._redis_engine.ping()
        except Exception as e:
            logger.error(f"Redis ping failed: {e}")
            return False

    @ErrorTraceback.w_check_error_exist
    async def configure_base_settings(self):
        """Configure Redis base settings like maxmemory and logfile."""
        client = await self._ensure_connected()
        
        try:
            # Set memory limit if configured
            if hasattr(self, '_config') and 'redis' in self._config:
                maxmemory = self._config['redis'].get('maxmemory')
                if maxmemory:
                    logger.info(f"Setting Redis maxmemory to {maxmemory}")
                    await client.config_set("maxmemory", maxmemory)
        except Exception as e:
            err_msg = f"Failed to configure Redis settings: {e}"
            logger.error(err_msg)
            raise RuntimeError(err_msg)

    @ErrorTraceback.w_check_error_exist
    async def close(self):
        """Close the Redis connection."""
        for ac in self._active_channels:
            try:
                await self.remove_listener_channels(ac)
            except Exception as e:
                logger.warning(f"Error unsubscribing from {ac} during close: {e}")

        if self._pubsub:
            await self._pubsub.close()
            self._pubsub = None
        
        if self._redis_engine:
            await self._redis_engine.close()
            self._redis_engine = None
        
        self._connected = False
        logger.debug(f"Redis connection closed for {self.module_name}")

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
            logger.error(f"Error getting key {key}: {e}")
            return None

    @ErrorTraceback.w_check_error_exist
    async def set(self, key: str, value: Any, ex: int = None) -> bool:
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
                result = await client.set(key, value, ex=ex)
                return result is not None
        except Exception as e:
            logger.error(f"Error setting key {key}: {e}")
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
            logger.error(f"Error deleting key {key}: {e}")
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
            logger.error(f"Error clearing Redis database: {e}")
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
            logger.error(f"Error retrieving keys from Redis: {e}")
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
            logger.error(f"Error retrieving keys with pattern {pattern}: {e}")
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
            logger.error(f"Error retrieving type for key {key}: {e}")
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
                    logger.error(f"Unsupported Redis type: {redis_type}")
                    return -1
        except Exception as e:
            logger.error(f"Error getting length for key {key}: {e}")
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
                message = json.dumps(message)
            
            result = await client.publish(channel, message)
            logger.debug(f"📤 Published to {channel}: {message}")
            return result
        except Exception as e:
            logger.error(f"❌ Failed to publish message to {channel}: {e}")
            return 0

    @ErrorTraceback.w_check_error_exist
    async def subscribe_channel(self, *channel: str) -> None:
        """
        Subscribe to Redis channel
        
        Args:
            channel: Channel name to subscribe to
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        for ch in channel:
            try:
                await self._pubsub.subscribe(ch)
                logger.debug(f"📥 Subscribed to channel: {ch}")
            except Exception as e:
                logger.error(f"❌ Failed to subscribe to {ch}: {e}")
                continue

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
            logger.error(f"Error subscribing to key {key}: {e}")

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
            logger.error(f"Error unsubscribing from key {key}: {e}")

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
    # PUB/SUB EXTENDED
    # ======================
    
    @ErrorTraceback.w_check_error_exist
    async def subscribe_pattern(self, pattern: str) -> PubSub:
        """
        Subscribe to Redis channel pattern (wildcard support)
        
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
            logger.debug(f"📥 Subscribed to pattern: {pattern}")
            return self._pubsub
        except Exception as e:
            logger.error(f"❌ Failed to subscribe to pattern {pattern}: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def publish_with_metadata(self, 
                                    channel: str, 
                                    data: Dict[str, Any],
                                    add_metadata: bool = True) -> int:
        """
        Publish message to a Redis channel with optional metadata
        
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
    async def create_pubsub_listener(self, 
                                     channels: str|list[str], 
                                     callback_handler: Callable|Coroutine,
                                     callback_context: Any=None,
                                     start_loop: bool = True) -> None:
        """
        Create a persistent PubSub listener with callback (supports multiple calls)
        
        Args:
            channels: List of channels/patterns to listen to (use * for wildcards)
            callback_handler: Async function to handle incoming messages (signature: async def handler(message, context))
            callback_context: Optional context object passed to callback (e.g., PermissionManager instance)
            start_loop: Whether to start the listener loop (default True)
                       Set to False if you want to add channels first and start loop manually
        
        Note:
            - Can be called multiple times to add more channels
            - If loop is already running, new channels are automatically subscribed
            - Only one background task is created regardless of how many times this is called
        """
        client = await self._ensure_connected()
        
        if self._pubsub is None:
            self._pubsub = client.pubsub()
        
        if isinstance(channels, str):
            channels = [channels]
        try:
            # Subscribe to new channels
            new_channels = []
            for channel in channels:
                if channel not in self._active_channels:
                    if "*" in channel:
                        await self._pubsub.psubscribe(channel)
                        logger.debug(f"📥 Pattern subscribed: {channel}")
                    else:
                        await self._pubsub.subscribe(channel)
                        logger.debug(f"📥 Channel subscribed: {channel}")
                    
                    self._active_channels.add(channel)
                    new_channels.append(channel)
                else:
                    logger.debug(f"⏭️  Channel already subscribed: {channel}")
            
            if new_channels:
                logger.info(f"🎧 PubSub listener added channels: {new_channels}")
            
            # Register per-channel callbacks for routing
            if callback_handler:
                for channel in channels:
                    self._channel_callbacks[channel] = (callback_handler, callback_context)
            
            # Start listening loop if not already running and start_loop is True
            if start_loop and callback_handler and not self._listener_running:
                self._listener_running = True
                self._listener_task = asyncio.create_task(
                    self._start_pubsub_loop(callback_handler, callback_context)
                )
                logger.info(f"🔄 PubSub listener loop started for {len(self._active_channels)} channels")
            elif self._listener_running:
                logger.debug(f"🔄 PubSub loop already running, new channels automatically active")
            elif not callback_handler:
                logger.warning("⚠️ No callback handler provided for PubSub listener")
                
        except Exception as e:
            logger.error(f"❌ Failed to create PubSub listener: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def _start_pubsub_loop(self, callback_handler, callback_context):
        """
        Start the PubSub message processing loop (runs as background task)
        
        Args:
            callback_handler: Async function to process messages
            callback_context: Optional context object for message handling
        """
        if self._pubsub is None:
            raise RuntimeError("PubSub not available")
        
        try:
            logger.info(f"🔄 Starting Redis PubSub message loop for {len(self._active_channels)} channels...")
            
            async for message in self._pubsub.listen():
                if not self._listener_running:
                    logger.info("⏹️  PubSub loop stopped by flag")
                    break
                
                if message["type"] in ["message", "pmessage"]:
                    try:
                        # Route to per-channel callback if registered, else use default
                        channel = message.get('channel', b'')
                        if isinstance(channel, bytes):
                            channel = channel.decode('utf-8')
                        
                        if channel in self._channel_callbacks:
                            ch_handler, ch_context = self._channel_callbacks[channel]
                        else:
                            ch_handler = callback_handler
                            ch_context = callback_context

                        # Process message through callback
                        if asyncio.iscoroutinefunction(ch_handler):
                            await ch_handler(message, ch_context)
                        else:
                            ch_handler(message, ch_context)
                            
                        logger.debug(f"📨 Processed message from {channel}")
                        
                    except Exception as e:
                        logger.error(f"❌ Error processing message: {e}")
                        # Continue processing other messages
                        
        except asyncio.CancelledError:
            logger.info("⏹️  PubSub loop cancelled")
            raise
        except Exception as e:
            logger.error(f"❌ PubSub loop error: {e}")
            raise
        finally:
            self._listener_running = False
            logger.info("🛑 PubSub message loop stopped")

    @ErrorTraceback.w_check_error_exist
    async def parse_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse and normalize a Redis PubSub message for easier processing
        
        Args:
            message: Raw Redis PubSub message
            
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
            logger.error(f"❌ Error parsing message: {e}")
            return {"channel": "unknown", "data": {}, "error": str(e)}

    @ErrorTraceback.w_check_error_exist
    async def get_active_listeners(self) -> Dict[str, Any]:
        """
        Get information about active PubSub listeners
        
        Returns:
            Dictionary with listener status and active channels
        """
        return {
            "listener_running": self._listener_running,
            "active_channels": list(self._active_channels),
            "total_channels": len(self._active_channels),
            "task_status": "running" if self._listener_task and not self._listener_task.done() else "stopped"
        }
    
    @ErrorTraceback.w_check_error_exist
    async def remove_listener_channels(self, channels: str | list[str]) -> Dict[str, Any]:
        """
        Remove specific channels from the active listeners
        
        Args:
            channels: List of channel names/patterns to unsubscribe
            
        Returns:
            Dictionary with removal status and remaining channels
        """
        if isinstance(channels, str):
            channels = [channels]

        if self._pubsub is None:
            logger.warning("⚠️ No PubSub connection available")
            return {
                "success": False,
                "removed": [],
                "remaining": [],
                "error": "No PubSub connection"
            }
        
        removed = []
        not_found = []
        
        try:
            for channel in channels:
                if channel in self._active_channels:
                    # Unsubscribe based on pattern or regular channel
                    if "*" in channel:
                        await self._pubsub.punsubscribe(channel)
                        logger.debug(f"📤 Pattern unsubscribed: {channel}")
                    else:
                        await self._pubsub.unsubscribe(channel)
                        logger.debug(f"📤 Channel unsubscribed: {channel}")
                    
                    self._active_channels.remove(channel)
                    removed.append(channel)
                else:
                    not_found.append(channel)
                    logger.debug(f"⚠️  Channel not found in active listeners: {channel}")
            
            remaining = list(self._active_channels)
            
            logger.info(f"🗑️  Removed {len(removed)} channels, {len(remaining)} remaining")
            
            return {
                "success": True,
                "removed": removed,
                "not_found": not_found,
                "remaining": remaining,
                "total_remaining": len(remaining)
            }
            
        except Exception as e:
            logger.error(f"❌ Failed to remove listener channels: {e}")
            return {
                "success": False,
                "removed": removed,
                "error": str(e)
            }
    
    @ErrorTraceback.w_check_error_exist
    async def stop_pubsub_listener(self) -> bool:
        """
        Stop the PubSub listener loop and unsubscribe from all channels
        
        Returns:
            True if stopped successfully
        """
        try:
            self._listener_running = False
            
            # Cancel the listener task
            if self._listener_task and not self._listener_task.done():
                self._listener_task.cancel()
                try:
                    await self._listener_task
                except asyncio.CancelledError:
                    pass
            
            # Unsubscribe from all channels
            if self._pubsub and self._active_channels:
                all_channels = list(self._active_channels)
                result = await self.remove_listener_channels(all_channels)
                logger.info(f"🛑 Stopped PubSub listener and unsubscribed from {len(result['removed'])} channels")
            
            self._listener_task = None
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to stop PubSub listener: {e}")
            return False
    
    # ======================
    # BACKWARD COMPATIBILITY - PubSub Aliases
    # ======================
    
    async def publish_event(self, channel: str, data: Dict[str, Any], add_metadata: bool = True) -> int:
        """Alias for publish_with_metadata (backward compatibility)"""
        return await self.publish_with_metadata(channel, data, add_metadata)
    
    async def create_stream_listener(self, channels: list[str], callback_handler, callback_context=None) -> None:
        """Alias for create_pubsub_listener (backward compatibility)"""
        return await self.create_pubsub_listener(channels, callback_handler, callback_context)

    # ======================
    # REDIS STREAMS
    # ======================
    
    @ErrorTraceback.w_check_error_exist
    async def xadd(self, 
                   stream: str, 
                   fields: Dict[str, Any],
                   message_id: str = "*",
                   maxlen: Optional[int] = None,
                   approximate: bool = True) -> str:
        """
        Add entry to a Redis Stream
        
        Args:
            stream: Stream name
            fields: Dictionary of field-value pairs
            message_id: Message ID (* for auto-generated)
            maxlen: Optional max stream length (for trimming)
            approximate: Use approximate trimming (~) for better performance
            
        Returns:
            Message ID of the added entry
        """
        client = await self._ensure_connected()
        
        try:
            # Convert all values to strings
            str_fields = {k: json.dumps(v) if isinstance(v, (dict, list)) else str(v) 
                         for k, v in fields.items()}
            
            if maxlen:
                result = await client.xadd(
                    stream, 
                    str_fields, 
                    id=message_id,
                    maxlen=maxlen,
                    approximate=approximate
                )
            else:
                result = await client.xadd(stream, str_fields, id=message_id)
            
            logger.debug(f"📝 Added entry to stream {stream}: {result}")
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to add entry to stream {stream}: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xread(self,
                    streams: Dict[str, str],
                    count: Optional[int] = None,
                    block: Optional[int] = None) -> List[tuple]:
        """
        Read entries from Redis Streams
        
        Args:
            streams: Dictionary of {stream_name: last_id} (use '0' for beginning, '$' for new)
            count: Max number of entries per stream
            block: Block for N milliseconds if no data (None = don't block)
            
        Returns:
            List of (stream_name, messages) tuples
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xread(
                streams=streams,
                count=count,
                block=block
            )
            
            logger.debug(f"📖 Read from streams: {list(streams.keys())}")
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to read from streams: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xreadgroup(self,
                        groupname: str,
                        consumername: str,
                        streams: Dict[str, str],
                        count: Optional[int] = None,
                        block: Optional[int] = None,
                        noack: bool = False) -> List[tuple]:
        """
        Read from streams as a consumer group member
        
        Args:
            groupname: Consumer group name
            consumername: Consumer identifier
            streams: Dictionary of {stream_name: last_id} (use '>' for new messages)
            count: Max number of entries
            block: Block for N milliseconds if no data
            noack: Don't require ACK (fire-and-forget)
            
        Returns:
            List of (stream_name, messages) tuples
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xreadgroup(
                groupname=groupname,
                consumername=consumername,
                streams=streams,
                count=count,
                block=block,
                noack=noack
            )
            
            logger.debug(f"📖 Consumer {consumername} read from group {groupname}")
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to read as consumer group: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xack(self,
                   stream: str,
                   groupname: str,
                   *message_ids: str) -> int:
        """
        Acknowledge processed messages in a consumer group
        
        Args:
            stream: Stream name
            groupname: Consumer group name
            message_ids: Message IDs to acknowledge
            
        Returns:
            Number of messages acknowledged
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xack(stream, groupname, *message_ids)
            logger.debug(f"✅ Acknowledged {result} messages in {stream}/{groupname}")
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to acknowledge messages: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xgroup_create(self,
                           stream: str,
                           groupname: str,
                           id: str = "$",
                           mkstream: bool = True) -> bool:
        """
        Create a consumer group
        
        Args:
            stream: Stream name
            groupname: Consumer group name
            id: Start reading from this ID ('$' = end, '0' = beginning)
            mkstream: Create stream if it doesn't exist
            
        Returns:
            True if successful
        """
        client = await self._ensure_connected()
        
        try:
            await client.xgroup_create(
                stream,
                groupname,
                id=id,
                mkstream=mkstream
            )
            logger.info(f"👥 Created consumer group {groupname} on stream {stream}")
            return True
            
        except Exception as e:
            # Group might already exist
            if "BUSYGROUP" in str(e):
                logger.debug(f"Consumer group {groupname} already exists on {stream}")
                return True
            logger.error(f"❌ Failed to create consumer group: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xgroup_destroy(self, stream: str, groupname: str) -> bool:
        """
        Destroy a consumer group
        
        Args:
            stream: Stream name
            groupname: Consumer group name
            
        Returns:
            True if successful
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xgroup_destroy(stream, groupname)
            logger.info(f"🗑️ Destroyed consumer group {groupname} on stream {stream}")
            return bool(result)
            
        except Exception as e:
            logger.error(f"❌ Failed to destroy consumer group: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xlen(self, stream: str) -> int:
        """
        Get the number of entries in a stream
        
        Args:
            stream: Stream name
            
        Returns:
            Number of entries
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xlen(stream)
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to get stream length: {e}")
            return 0

    @ErrorTraceback.w_check_error_exist
    async def xtrim(self,
                    stream: str,
                    maxlen: int,
                    approximate: bool = True) -> int:
        """
        Trim stream to a maximum length
        
        Args:
            stream: Stream name
            maxlen: Maximum number of entries to keep
            approximate: Use approximate trimming (~) for better performance
            
        Returns:
            Number of entries deleted
        """
        client = await self._ensure_connected()
        
        try:
            result = await client.xtrim(
                stream,
                maxlen=maxlen,
                approximate=approximate
            )
            logger.debug(f"✂️ Trimmed stream {stream}: removed {result} entries")
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to trim stream: {e}")
            raise

    @ErrorTraceback.w_check_error_exist
    async def xpending(self,
                      stream: str,
                      groupname: str,
                      start: str = "-",
                      end: str = "+",
                      count: int = 10,
                      consumername: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Get pending messages in a consumer group
        
        Args:
            stream: Stream name
            groupname: Consumer group name
            start: Start ID (- for minimum)
            end: End ID (+ for maximum)
            count: Max messages to return
            consumername: Filter by specific consumer
            
        Returns:
            List of pending message info
        """
        client = await self._ensure_connected()
        
        try:
            if consumername:
                result = await client.xpending_range(
                    stream,
                    groupname,
                    min=start,
                    max=end,
                    count=count,
                    consumername=consumername
                )
            else:
                result = await client.xpending_range(
                    stream,
                    groupname,
                    min=start,
                    max=end,
                    count=count
                )
            
            return result
            
        except Exception as e:
            logger.error(f"❌ Failed to get pending messages: {e}")
            raise