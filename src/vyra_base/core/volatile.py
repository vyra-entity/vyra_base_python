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
    async def activate_listener(self):
        """
        Activate the listener for transient events.
        This method starts listening for changes on volatile parameters.
        It will create a speaker for each volatile parameter and subscribe to it.
        """
        # Start listening for transient events
        self._listener = asyncio.create_task(
            self.transient_event_listener())
    
    @ErrorTraceback.w_check_error_exist
    async def transient_event_listener(self):
        self._listener_active = True
        async for message in self.redis.pubsub.listen():
            if message["type"] == "message":
                channel = message["channel"]
                if channel in self._active_shouter:
                    self._active_shouter[channel].shout(message["data"])

    @ErrorTraceback.w_check_error_exist
    async def read_all_volatile_names(self) -> list:
        """
        Read all volatile parameter names.
        """
        return list(await self.redis.get_all_keys())

    @ErrorTraceback.w_check_error_exist
    async def set_volatile_value(self, key: Any, value: Type[REDIS_TYPE]):
        """
        Set the value of the volatile parameter.
        """
        await self.redis.set(key, value)


    @ErrorTraceback.w_check_error_exist
    async def get_volatile_value(self, key: str) -> Any:
        """
        Get the value of the volatile parameter.
        """
        return await self.redis.get(key)

    @ErrorTraceback.w_check_error_exist
    async def add_change_event(self, key: str):
        """
        Publish a change event for the volatile parameter. 
        :param key: The key of the volatile parameter. Used to identify the parameter.
        """
        if not await self.redis.exists(key):
            raise KeyError(f"Key '{key}' does not exist in the transient storage.")

        get_type: REDIS_TYPE | None = await self.redis.get_type(key)

        if get_type is None:
            raise KeyError(f"Key '{key}' does not exist in the transient storage.")

        if get_type not in self.REDIS_TYPE_MAP:
            raise ValueError(
                f"Unsupported Redis type: {get_type}. "
                f"Must be one of {list(self.REDIS_TYPE_MAP)}.")

        speaker: VyraSpeaker = create_vyra_speaker(
            type=self.REDIS_TYPE_MAP[get_type],
            node=self.communication_node,
            description="Volatile parameter changes: " + key,
            ident_name="volatile"
        )
        self._active_shouter[key] = speaker
        
        await self.redis.subscribe_to_key(key)

    @ErrorTraceback.w_check_error_exist
    async def remove_change_event(self, key: str):
        """
        Remove the change event for the volatile parameter.
        :param key: The key of the volatile parameter.
        """
        if await self.redis.exists(key):
            await self.redis.unsubscribe_from_key(key)
        else:
            raise KeyError(f"Key '{key}' does not exist in the transient storage.")
