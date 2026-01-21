import asyncio
from typing import Any, Type

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.speaker import VyraSpeaker
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.storage.redis_client import RedisClient, REDIS_TYPE
from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
from vyra_base.com.datalayer.interface_factory import remove_vyra_speaker


class Volatile:
    """
    A class to manage volatile parameters. Volatile parameters are
    temporary parameters that are not persisted in the database.
    They are stored in Redis and can be used for temporary data storage.
    This class provides methods to read, write, and manage volatile parameters.
    It also provides methods to subscribe to changes on volatile parameters.
    The volatile parameters are identified by their keys, which are strings.
    The class uses Redis as the storage backend for volatile parameters.
    API:
        - Volatiles could only be written by the module itself.
        - Volatiles could be subscribed by other modules to get notified on changes.
        - Volatiles are not persisted and will be lost on system restart.
        - Volatiles are not shared between modules, each module has its own set of volatiles.
        - Volatiles are identified by their keys, which are strings.
        - Volatiles could be of different types, such as string, hash, list, set.
    Example usage:
        - current robot state
        - io states
        - status information
        - temporary data storage
        - any custom data defined by the module.
    """
    EVENT_TOPIC_PREFIX = "volatile/"

    def __init__(
            self, 
            storage_access_transient: RedisClient, 
            module_id: str,
            node: VyraNode,
            transient_base_types: dict[str, Any]):
        """
        Initialize the Volatile class.
        """
        self.module_id: str = module_id
        self.communication_node = node

        self.REDIS_TYPE_MAP: dict[REDIS_TYPE, type] = {
            REDIS_TYPE.STRING: transient_base_types['VolatileString'],
            REDIS_TYPE.HASH: transient_base_types['VolatileHash'],
            REDIS_TYPE.LIST: transient_base_types['VolatileList'],
            REDIS_TYPE.SET: transient_base_types['VolatileSet']
        }
        
        self.redis: RedisClient = storage_access_transient
        self._active_shouter: dict[str, VyraSpeaker] = {}
        self._listener: asyncio.Task | None = None



    def __del__(self):
        """
        Clean up the Volatile instance.
        This will unsubscribe from all active shouters and stop the listener if active.
        """
        if self._listener:
            self._listener.cancel()
            self._listener = None

        for key, speaker in self._active_shouter.items():
            remove_vyra_speaker(key)

        self._active_shouter.clear()

    @ErrorTraceback.w_check_error_exist
    async def activate_listener(self, channel):
        """
        Activate the Redis pub/sub listener for monitoring volatile parameter changes.
        
        This method starts listening for changes on registered volatile parameters.
        When a volatile value changes in Redis, the listener will receive a message
        and trigger the appropriate ROS2 speaker to publish the change.
        
        **Workflow:**
        1. Get currently active Redis pub/sub channels
        2. Identify new channels that need listeners
        3. Create Redis pub/sub listener with change notification callback
        
        :param channel: The channel name to activate (currently unused, for future extension).
        :type channel: str
        :raises KeyError: If the channel does not exist in Redis.
        :raises RuntimeError: If the listener creation fails.
        """
        # Get list of channels that are already being listened to
        active_listener = (await self.redis.get_active_listeners())['active_channels']
        
        # Find channels in our speaker registry that don't have active listeners yet
        new_listener = [li for li in self._active_shouter.keys() if li not in active_listener]
        
        # Create pub/sub listener for new channels
        await self.redis.create_pubsub_listener(
            channels=new_listener,
            callback_handler=self.on_volatile_change_received
        )
    
    @ErrorTraceback.w_check_error_exist
    async def on_volatile_change_received(self, message: dict, callback_context):
        """
        Callback handler that processes Redis pub/sub messages when volatile values change.
        
        This method is called automatically by the Redis pub/sub listener whenever
        a volatile parameter changes. It publishes the change to the corresponding
        ROS2 topic via the registered VyraSpeaker.
        
        **Message flow:**
        1. Redis detects value change on subscribed key
        2. Redis pub/sub sends message to this callback
        3. Callback extracts channel and message type
        4. If valid, publishes change to ROS2 topic via VyraSpeaker
        
        :param message: Redis pub/sub message containing type, channel, and data.
        :type message: dict
        :param callback_context: Context information from the Redis listener.
        :type callback_context: Any
        
        **Expected message format:**
        {
            "type": "message",
            "channel": "volatile_key_name",
            "data": <value>
        }
        """
        message_type = message.get("type", None)
        channel = message.get("channel", None)

        # Only process actual messages (not subscribe/unsubscribe events)
        if message_type == "message" and channel in self._active_shouter:
            # Publish the changed value to the ROS2 topic
            self._active_shouter[channel].shout(message["data"])

    @ErrorTraceback.w_check_error_exist
    async def read_all_volatile_names(self) -> list:
        """
        Retrieve all volatile parameter names (keys) stored in Redis.
        
        This method queries Redis for all keys in the module's namespace
        and returns them as a list. Useful for discovering available
        volatile parameters or iterating over all stored values.
        
        :return: List of volatile parameter key names.
        :rtype: list[str]
        
        **Example:**
        
        .. code-block:: python
        
            keys = await volatile.read_all_volatile_names()
            print(f"Available volatiles: {keys}")
            # Output: ['temperature', 'pressure', 'humidity']
        """
        return list(await self.redis.get_all_keys())

    @ErrorTraceback.w_check_error_exist
    async def set_volatile_value(self, key: Any, value: Type[REDIS_TYPE]):
        """
        Set or update the value of a volatile parameter in Redis.
        
        This method stores a value under the specified key. If the key already
        exists, its value will be overwritten. The value is stored in Redis
        using the appropriate data structure (string, hash, list, or set).
        
        :param key: The unique identifier for the volatile parameter.
        :type key: str
        :param value: The value to store (can be string, dict, list, or set).
        :type value: Any
        
        **Example:**
        
        .. code-block:: python
        
            # Store simple value
            await volatile.set_volatile_value("temperature", 23.5)
            
            # Store complex data
            await volatile.set_volatile_value("sensor_data", {
                "temp": 23.5,
                "humidity": 60.2
            })
        """
        await self.redis.set(key, value)


    @ErrorTraceback.w_check_error_exist
    async def get_volatile_value(self, key: str) -> Any:
        """
        Retrieve the current value of a volatile parameter from Redis.
        
        This method fetches the value stored under the specified key.
        If the key does not exist, None is returned.
        
        :param key: The unique identifier of the volatile parameter.
        :type key: str
        :return: The stored value, or None if the key does not exist.
        :rtype: Any
        
        **Example:**
        
        .. code-block:: python
        
            value = await volatile.get_volatile_value("temperature")
            if value is not None:
                print(f"Current temperature: {value}°C")
        """
        return await self.redis.get(key)

    @ErrorTraceback.w_check_error_exist
    async def subscribe_to_changes(self, volatile_key: str):
        """
        Subscribe to change notifications for a specific volatile parameter.
        
        This method sets up automatic ROS2 topic publishing whenever the specified
        volatile parameter changes in Redis. It creates a ROS2 speaker that will
        publish change events to a topic that other modules can subscribe to.
        
        **Setup workflow:**
        1. Verify that the volatile key exists in Redis
        2. Determine the Redis data type (string, hash, list, set)
        3. Create appropriate ROS2 speaker for that data type
        4. Subscribe to Redis key-space notifications for that key
        5. When key changes, Redis notifies this module
        6. Notification triggers ROS2 topic publication via speaker
        
        :param volatile_key: The name of the volatile parameter to monitor.
        :type volatile_key: str
        :raises KeyError: If the volatile key does not exist in Redis.
        :raises ValueError: If the Redis data type is not supported.
        
        **Example usage:**
        
        .. code-block:: python
        
            # Step 1: Create volatile value
            await volatile.set_volatile_value("temperature", 23.5)
            
            # Step 2: Subscribe to changes (creates ROS2 topic)
            await volatile.subscribe_to_changes("temperature")
            
            # Step 3: Activate listener to receive notifications
            await volatile.activate_listener("temperature")
            
            # Step 4: Any change now publishes to ROS2 topic
            await volatile.set_volatile_value("temperature", 24.1)
            # -> Automatically published to ROS2 topic!
        
        **Resulting ROS2 topic:**
        The created topic name follows the pattern:
        ``/module_name/volatile/<volatile_key>``
        """
        # Step 1: Verify volatile key exists
        if not await self.redis.exists(volatile_key):
            raise KeyError(
                f"Volatile key '{volatile_key}' does not exist in Redis. "
                f"Create it first with set_volatile_value().")

        # Step 2: Get Redis data type
        redis_type: REDIS_TYPE | None = await self.redis.get_type(volatile_key)

        if redis_type is None:
            raise KeyError(f"Could not determine type for key '{volatile_key}'.")

        if redis_type not in self.REDIS_TYPE_MAP:
            raise ValueError(
                f"Unsupported Redis type: {redis_type}. "
                f"Supported types: {list(self.REDIS_TYPE_MAP.keys())}")

        # Step 3: Create ROS2 speaker for this volatile type
        speaker: VyraSpeaker = create_vyra_speaker(
            type=self.REDIS_TYPE_MAP[redis_type],
            node=self.communication_node,
            description=f"Volatile change events for: {volatile_key}",
            ident_name="volatile"
        )
        self._active_shouter[volatile_key] = speaker
        
        # Step 4: Subscribe to Redis key-space notifications
        await self.redis.subscribe_to_key(volatile_key)

    @ErrorTraceback.w_check_error_exist
    async def unsubscribe_from_changes(self, volatile_key: str):
        """
        Unsubscribe from change notifications for a specific volatile parameter.
        
        This method stops monitoring the specified volatile parameter and removes
        the ROS2 topic publication. After calling this, changes to the volatile
        will no longer trigger ROS2 topic messages.
        
        :param volatile_key: The name of the volatile parameter to stop monitoring.
        :type volatile_key: str
        :raises KeyError: If the volatile key does not exist in Redis.
        
        **Example:**
        
        .. code-block:: python
        
            # Stop monitoring temperature changes
            await volatile.unsubscribe_from_changes("temperature")
        """
        if await self.redis.exists(volatile_key):
            await self.redis.unsubscribe_from_key(volatile_key)
            # Remove speaker from registry if it exists
            if volatile_key in self._active_shouter:
                remove_vyra_speaker(volatile_key)
                del self._active_shouter[volatile_key]
        else:
            raise KeyError(
                f"Volatile key '{volatile_key}' does not exist in Redis.")
