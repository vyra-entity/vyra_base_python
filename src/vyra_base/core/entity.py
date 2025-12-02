from __future__ import annotations

import asyncio
import json
from asyncio import AbstractEventLoop
from dataclasses import asdict
from pathlib import Path
from typing import Any, Optional, Union

from vyra_base.com.datalayer.callable import VyraCallable
from vyra_base.com.datalayer.interface_factory import (
    DataSpace,
    create_vyra_callable,
    create_vyra_job,
    create_vyra_speaker,
    remote_callable,
)
from vyra_base.com.datalayer.node import CheckerNode, NodeSettings, VyraNode
from vyra_base.com.feeder.error_feeder import ErrorFeeder
from vyra_base.com.feeder.news_feeder import NewsFeeder
from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.defaults.entries import (
    ErrorEntry,
    FunctionConfigBaseTypes,
    FunctionConfigEntry,
    ModuleEntry,
    NewsEntry,
    StateEntry,
)
from vyra_base.helper.logger import Logger
from vyra_base.state.state_machine import StateMachine
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_access import DBTYPE
from vyra_base.storage.redis_client import RedisClient
from vyra_base.storage.storage import Storage
from vyra_base.core.parameter import Parameter
from vyra_base.core.volatile import Volatile
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.security.access import AccessManager


class VyraEntity:
    """
    Base class for all V.Y.R.A. entities .

    This class initializes the entity with a ROS2 node, state, news, and error feeders.
    It also provides methods to register remote callables and manage interfaces.
    It is designed to be extended by specific entities that require additional functionality.

    :ivar node: The ROS2 node for the entity.
    :vartype node: VyraNode
    :ivar state_feeder: Feeder for state updates.
    :vartype state_feeder: StateFeeder
    :ivar news_feeder: Feeder for news updates.
    :vartype news_feeder: NewsFeeder
    :ivar error_feeder: Feeder for error updates.
    :vartype error_feeder: ErrorFeeder
    :ivar state_machine: State machine for managing entity states.
    :vartype state_machine: StateMachine

    :cvar _interface_list: List of interface configurations.
    :vartype _interface_list: list[FunctionConfigEntry]
    :cvar _storage_list: List of registered storage objects.
    :vartype _storage_list: list[Storage]

    :raises RuntimeError: If the node name is already available in the ROS2 system.
    """

    _interface_list: list[FunctionConfigEntry] = []
    _storage_list: list[Storage] = []

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, 
            state_entry: StateEntry,
            news_entry: NewsEntry,
            error_entry: ErrorEntry,
            module_entry: ModuleEntry,
            module_config: dict[str, Any],
            log_config: Optional[dict[str, Any]] = None) -> None:
        """
        Initialize the VyraEntity.

        :param state_entry: State entry configuration.
        :type state_entry: StateEntry
        :param news_entry: News entry configuration.
        :type news_entry: NewsEntry
        :param error_entry: Error entry configuration.
        :type error_entry: ErrorEntry
        :param module_entry: Module entry configuration.
        :type module_entry: ModuleEntry
        :param module_config: Module configuration. Containing settings that are used during runtime. 
                              For example simulation settings, livecycle settings
        :type module_config: dict[str, Any]
        :param log_config: Optional configuration for the logger. Using python logging config to configurate.
        :type log_config: dict[str, Any], optional
        :raises TypeError: If the provided entries are not of the correct type.
        :raises ValueError: If the module entry is not valid.
        :raises RuntimeError: If the node name is already available in the ROS2 system.
        """
        self._init_logger(log_config)

        if VyraEntity._check_node_availability(module_entry.name):
            raise RuntimeError(
                f"Node {module_entry.name} is available in the ROS2 system."
                " Please choose a different name."
            )

        self.module_entry: ModuleEntry = module_entry
        self.module_config: dict[str, Any] = module_config

        node_settings = NodeSettings(
            name=f"{self.module_entry.name}_{self.module_entry.uuid}"
        )

        self._node = VyraNode(node_settings)

        feeder: tuple = self.__init_feeder(state_entry, news_entry, error_entry)
        self.state_feeder: StateFeeder = feeder[0]
        self.news_feeder: NewsFeeder = feeder[1]
        self.error_feeder: ErrorFeeder = feeder[2]

        self.state_machine: StateMachine = self._init_state_machine(state_entry)

        self.access_manager: AccessManager = self._init_security_access()

        VyraEntity.register_callables_callbacks(self)
        VyraEntity.register_callables_callbacks(self.access_manager)

        self.news_feeder.feed("...V.Y.R.A. entity initialized")

    @property
    def node(self) -> Optional[VyraNode]:
        """
        Get the ROS2 node of the entity.

        :returns: The ROS2 node.
        :rtype: VyraNode
        """
        if not hasattr(self, '_node'):
            return None
        return self._node
    
    @property
    def namespace(self) -> str:
        """
        Get the namespace of the ROS2 node.

        :returns: The namespace of the node.
        :rtype: str
        """
        if not hasattr(self, '_node'):
            return ''
        return self._node.get_namespace()

    def _init_logger(self, log_config: Optional[dict[str, Any]]) -> None:
        """
        Initialize the logger for the entity.
        
        This method sets up the logger configuration based on the provided log configuration path.
        It should be called during the initialization of the entity.
        """
        log_config_path = Path(__file__).resolve().parent.parent
        log_config_path: Path = log_config_path / "helper" / "logger_config.json"
        Logger.initialize(log_config_path=log_config_path, log_config=log_config)

    def __init_feeder(
            self, 
            state_entry: StateEntry, 
            news_entry: NewsEntry, 
            error_entry: ErrorEntry) -> tuple[StateFeeder, NewsFeeder, ErrorFeeder]:
        """
        Initialize the feeders for the entity.

        This method sets up the state, news, and error feeders for the entity.
        It should be called during the initialization of the entity.
        """
        state_feeder = StateFeeder(
            type=state_entry._type, 
            node=self._node, 
            module_config=self.module_entry
        )

        news_feeder = NewsFeeder(
            type=news_entry._type, 
            node=self._node,
            module_config=self.module_entry,
            loggingOn=True
        )
        
        error_feeder = ErrorFeeder(
            type=error_entry._type, 
            node=self._node,
            module_config=self.module_entry,
            loggingOn=True
        )
        return state_feeder, news_feeder, error_feeder

    async def _init_storages_accesses(
            self, persistent_config: dict[str, Any], 
            transient_config: dict[str, Any]) -> None:
        """
        Initialize storages for the entity.

        This method sets up the storage access for the entity, including persistent and transient storage.
        It should be called during the initialization of the entity.
        """
        self.database_access = DbAccess(
            module_name=self.module_entry.name,
            db_config=persistent_config
        )

        await self.database_access.create_all_tables()

        self.redis_access = RedisClient(
            module_name=self.module_entry.name,
            redis_config=transient_config,
        )

        await self.redis_access.configure_base_settings()

    async def _init_params(
            self, 
            default_config: Any, 
            parameter_base_types: dict[str, Any]) -> None:
        """
        Initialize parameters for the entity.

        This method should be implemented to set up initial parameters.
        It is called during the initialization of the entity.
        :param default_config: Configuration for the database containing default parameters.
        :type default_config: Any
        """
        Logger.debug("Initializing parameters for the entity.")
        
        self.param_manager = Parameter(
            parameter_base_types=parameter_base_types,
            node=self._node,
            storage_access_persistant=self.database_access,
            storage_access_transient=self.redis_access
        )

        VyraEntity.register_callables_callbacks(self.param_manager)

        self.default_database_access = DbAccess(
            module_name=self.module_entry.name,
            db_config=default_config
        )

        await self.param_manager.load_defaults(self.default_database_access)

    def _init_volatiles(self, transient_base_types: dict[str, Any]) -> None:
        """
        Initialize volatile parameters for the entity. Volatile parameters are stored in Redis.
        Thus they could only be used during runtime and are not persisted.

        This method sets up the volatile parameters using Redis for transient storage.
        It should be called during the initialization of the entity.
        
        :param transient_base_types: Dictionary containing base types for volatile parameters.
        :type transient_base_types: dict[str, Any]
        """
        Logger.debug("Initializing volatile parameters for the entity.")
        
        self.volatile = Volatile(
            storage_access_transient=self.redis_access,
            module_id=self.module_entry.uuid,
            node=self._node,
            transient_base_types=transient_base_types
        )

        VyraEntity.register_callables_callbacks(self.volatile)

    def _init_state_machine(self, state_entry: StateEntry) -> StateMachine:
        """
        Initialize the state machine for the entity.

        This method sets up the state machine with the provided state entry.
        It should be called during the initialization of the entity.
        
        :param state_entry: The state entry configuration.
        :type state_entry: StateEntry
        """
        state_machine = StateMachine(
            self.state_feeder,
            state_entry._type,
            module_config=self.module_entry
        )
        state_machine.initialize()
        return state_machine

    def _init_security_access(self) -> AccessManager:
        """
        Initialize the security access for the entity.

        This method sets up the security access manager for the entity.
        It should be called during the initialization of the entity.
        """
        Logger.debug("Initializing security access for the entity.")
        
        return AccessManager()

    async def setup_storage(
            self, config: dict[str, Any], 
            transient_base_types: dict[str, Any],
            parameter_base_types: dict[str, Any]) -> None:
        """
        Set up the storage for the entity.

        This method initializes the storage access for the entity, including persistent and transient storage.
        It should be called during the initialization of the entity.
        
        :param config: Optional configuration for the storage setup.
        :type config: dict[str, Any]
        :param transient_base_types: Dictionary containing base types for transient storage. 
                                     Must be defined in the vyra module interfaces.
        :type transient_base_types: dict[str, Any]
        :raises ValueError: If the configuration is invalid or incomplete.
        :raises RuntimeError: If the storage setup fails.
        :returns: None
        """
        if not isinstance(config, dict) or config == {}:
            Logger.warn("No storage configuration provided. Skipping storage setup.")
            return

        persistent_config: dict[str, Any] = {}
        transient_config: dict[str, Any] = {}

        for config_key in config.keys():
            if config_key in (item.value for item in DBTYPE):
                Logger.debug(f"Configuring {config_key} for persistent storage.")
                persistent_config[config_key] = config[config_key]
            elif config_key == "redis":
                Logger.debug(f"Configuring {config_key} for transient storage.")
                transient_config[config_key] = config[config_key]

        if not persistent_config or not transient_config:
            Logger.warn(
                "Incomplete storage configuration provided. "
                "Skipping storage setup.")
            raise ValueError(
                "Both persistent and transient storage configurations must be provided."
            )

        await self._init_storages_accesses(
            persistent_config=persistent_config,
            transient_config=transient_config
        )

        if 'default_database' not in persistent_config[self.database_access.db_type]:
            Logger.warn(
                "No default database configuration provided. "
                "Skipping parameter initialization.")
            return
        else:
            dtype = self.database_access.db_type
            persistent_config[dtype]['database'] = persistent_config[dtype]['default_database']
            
            await self._init_params(persistent_config, parameter_base_types)

        self._init_volatiles(transient_base_types=transient_base_types)

        Logger.log("Storage access initialized.")

    async def set_interfaces(
            self, 
            settings: list[FunctionConfigEntry],
            permission_xml: str=None) -> None:
        """
        Add a DDS communication interface to this module.

        Interfaces are used to communicate with other modules or external systems.
        Interface types can be:

        - ``!vyra-callable``: Callable function that can be invoked remotely.
        - ``!vyra-job``: Job that can be executed in the background.
        - ``!vyra-speaker``: Speaker that can publish messages periodically.

        :param settings: Settings for the module functions.
        :type settings: list[FunctionConfigEntry]
        :param permission_file: Path to the permission file for the interface.
        :type permission_file: str
        :returns: None
        :rtype: None
        """

        if not permission_xml:
            Logger.warn(
                f"No permission XML provided for provided settings: {settings}. "
                "Skip loading interfaces in permission.xml"
            )

        async_loop: AbstractEventLoop = asyncio.get_event_loop()

        for setting in settings:
            if setting.functionname in [i.functionname for i in self._interface_list]:
                Logger.warn(
                    f"Interface {setting.functionname} already "
                    "exists. Skipping creation.")
                continue
            
            self._interface_list.append(setting)

            if setting.type == FunctionConfigBaseTypes.callable.value:
                Logger.info(f"Creating callable: {setting.functionname}")
                create_vyra_callable(
                    type=setting.ros2type,
                    node=self._node,
                    callback=setting.callback,
                    async_loop=async_loop,
                    ident_name=setting.functionname
                )
            elif setting.type == FunctionConfigBaseTypes.job.value:
                Logger.info(f"Creating job: {setting.functionname}")
                # TBD: vyra job implementation
                # create_vyra_job(...)
                Logger.warn(f"Vyra Jobs are not implemented yet.")

            elif setting.type == FunctionConfigBaseTypes.speaker.value:
                Logger.info(f"Creating speaker: {setting.functionname}")
                periodic: bool = False
                periodic_caller: Any = None
                periodic_interval: Union[float, None] = None

                if setting.periodic != None:
                    periodic: bool = True
                    periodic_caller = setting.periodic.caller if periodic else None
                    periodic_interval = setting.periodic.interval if periodic else None
                
                create_vyra_speaker(
                    type=setting.ros2type,
                    node=self._node,
                    description=setting.description,
                    periodic=periodic,
                    interval_time=periodic_interval,
                    periodic_caller= periodic_caller,
                    qos_profile=setting.qosprofile,
                    async_loop=async_loop,
                    ident_name=setting.functionname
                )
            else:
                fail_msg = (
                    f"Unsupported interface type: {setting.type}. "
                    "Supported types are callable, job, speaker."
                )
                Logger.error(fail_msg)
                raise ValueError(fail_msg)
        
            

    def register_storage(self, storage: Storage) -> None:
        """
        Register a storage object to the entity.

        :param storage: The storage object to register.
        :type storage: Storage
        :raises TypeError: If storage is not an instance of Storage.
        """
        if not isinstance(storage, Storage):
            raise TypeError("storage must be an instance of Storage.")
        
        self._storage_list.append(storage)
        Logger.info(f"Storage {storage} registered successfully.")

    @classmethod
    def _check_node_availability(cls, node_name: str) -> bool:
        """
        Check if a node with the given name is available in the ROS2 system.

        :param node_name: The name of the node to check.
        :type node_name: str
        :returns: True if the node is available, False otherwise.
        :rtype: bool
        """
        checker_node = CheckerNode()
        return checker_node.is_node_available(node_name)

    # @remote_callable
    # async def trigger_transition(self, request: Any, response: Any) -> None:
    #     """
    #     Trigger a state transition for the entity from internal or remote.

    #     :param request: The request containing the transition name.
    #     :type request: Any
    #     :param response: The response object to update with the result.
    #     :type response: Any
    #     :raises NotImplementedError: If the method is not implemented in the subclass.
    #     :returns: None
    #     :rtype: None
    #     """
    #     # TEST -----------------------------------------------------
    #     # from vyra_base.storage.db_manipulator import DbManipulator
    #     # from vyra_base.storage.tb_params import Parameter
    #     # self.mr_manipulator = DbManipulator(
    #     #     self.database_access, 
    #     #     Parameter
    #     # )
    #     # Logger.log(await self.mr_manipulator.get_all())
    #     # ----------------------------------------------------------

    #     trigger_list = [t['trigger'] for t in self.state_machine.all_transitions]
    #     if request.trigger_name not in trigger_list:
    #         fail_msg = (
    #             f"Transition {request.trigger_name} not found in "
    #             f"available transitions: {trigger_list}."
    #         )
            
    #         response.success = False
    #         response.message = fail_msg
    #         Logger.warn(fail_msg)
    #         return None
        
    #     can_trigger, possible_trigger = self.state_machine.is_transition_possible(
    #         request.trigger_name)

    #     if not can_trigger:
    #         fail_msg = (
    #             f"Transition {request.trigger_name} not possible in "
    #             f"current state {self.state_machine.model.state}."
    #             f" Possible transitions are: {possible_trigger}."
    #         )
            
    #         response.success = False
    #         response.message = fail_msg
    #         Logger.warn(fail_msg)
    #         return None
        
    #     getattr(self.state_machine.model, f"{request.trigger_name}")()

    #     response.success = True
    #     response.message = f"Transition {request.trigger_name} triggered successfully."

    #     self.news_feeder.feed(
    #         f"Transition {request.trigger_name} triggered successfully."
    #     )

    # @remote_callable
    # async def startup(self, request: Any, response: Any) -> None:
    #     """
    #     Start up the entity and initialize its components.

    #     :param request: The request object containing startup parameters.
    #     :type request: Any
    #     :param response: The response object to update with the result.
    #     :type response: Any
    #     :raises NotImplementedError: If the method is not implemented in the subclass.
    #     :returns: None
    #     :rtype: None
    #     """
        
    #     can_trigger, possible_trigger = self.state_machine.is_transition_possible(
    #         'StartUp')

    #     if not can_trigger:
    #         fail_msg = (
    #             f"<StartUp> not possible in "
    #             f"current state {self.state_machine.model.state}."
    #             f" Possible transitions are: {possible_trigger}."
    #         )
            
    #         response.success = False
    #         response.message = fail_msg
    #         Logger.warn(fail_msg)
    #         return None
        
    #     getattr(self.state_machine.model, f"StartUp")()

    #     response.success = True
    #     response.message = f"<StartUp> triggered successfully."

    #     self.news_feeder.feed(
    #         f"<StartUp> triggered successfully."
    #     )
    
    @remote_callable
    async def get_interface_list(self, request: Any, response: Any) -> Any:
        """
        Retrieves all capabilities (speaker, callable, job) of the entity that are set to
        visible from the module to external access.

        :param request: The request object.
        :type request: Any
        :param response: The response object to update with the result.
        :type response: Any
        :returns: None
        :rtype: Any
        """
        response_interface_list = []
        for interface in self._interface_list:
            if interface.displaystyle.visible:
                response_interface_list.append(json.dumps(interface.asdict()))

        response.interface_list = response_interface_list
        return None

    @staticmethod
    def register_callables_callbacks(callback_parent: object):
        """
        Registers all remote callables defined in the callback_parent. 
        
        Remote callables must be decorated with ``@vyra_base.com.datalayer.callable.remote_callable``.

        Example::

            from vyra_base.com.datalayer.interface_factory import remote_callable

            class MyParentClass:
                @remote_callable
                async def my_remote_function(self, request: Any, response: Any):
                    pass

            instance_my_parent = MyParentClass()

        To register the remote callable in this example, the instance_my_parent
        object must be passed to this function:
        
        - ``register_callables_callbacks(instance_my_parent)``

        Inside your MyParentClass in a method you can call the same function and
        set the callback_parent to self to register the callables of the
        instance itself:
        
        - ``register_callables_callbacks(self)``

        This function will iterate over all attributes of the instance and
        register those marked as remote callable with the DataSpace.

        .. warning::
           This function will only register the callbacks. Run 
           ``entity.set_interfaces(your_config)`` afterwards to load the interfaces 
           in vyra.

        :param callback_parent: The class or instance containing the remote callables.
        :type callback_parent: Type[object]
        :raises TypeError: If callback_parent is not an instance of object.
        :raises ValueError: If callback_parent does not have any remote callables.
        :raises RuntimeError: If the callable registration fails.
        :returns: None
        :rtype: None
        """
        for attr_name in dir(callback_parent):
            attr = getattr(callback_parent, attr_name)
            rc_active = getattr(attr, "_remote_callable", False)

            if callable(attr) and rc_active:
                callable_obj = VyraCallable(
                    name=attr.__name__,
                    connected_callback=attr
                )
                
                Logger.debug(
                    f"Registering callable callback <{callable_obj.name}> from method {attr}")
                
                DataSpace.add_callable(callable_obj)

                # Set on the underlying function object
                if hasattr(attr, "__func__"):
                    setattr(attr.__func__, "_remote_callable", False)
                else:
                    setattr(attr, "_remote_callable", False)