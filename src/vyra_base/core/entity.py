from __future__ import annotations

import asyncio
import importlib.util
import json
import logging

from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Union, TYPE_CHECKING

# NEW: Import from new multi-protocol architecture
from vyra_base.com import InterfaceFactory, remote_service, ProtocolType
from vyra_base.com.core.interface_path_registry import InterfacePathRegistry
from vyra_base.com.feeder.error_feeder import ErrorFeeder
from vyra_base.com.feeder.news_feeder import NewsFeeder
from vyra_base.com.feeder.state_feeder import StateFeeder

from vyra_base.com.transport.t_zenoh.provider import ZenohProvider, ZENOH_AVAILABLE

# Check ROS2 availability
_ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None

if TYPE_CHECKING or _ROS2_AVAILABLE:
    from vyra_base.com.transport.t_ros2.node import CheckerNode, NodeSettings, VyraNode
else:
    CheckerNode = None
    NodeSettings = None
    VyraNode = None
from vyra_base.defaults.entries import (
    ErrorEntry,
    FunctionConfigBaseTypes,
    FunctionConfigEntry,
    ModuleEntry,
    NewsEntry,
    StateEntry,
)
from vyra_base.state import (
    UnifiedStateMachine,
    StateMachineConfig,
    LifecycleState,
    OperationalState,
    HealthState,
    StateEvent,
    EventType,
)
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_access import DBTYPE
from vyra_base.com.transport.t_redis import RedisClient
from vyra_base.storage.storage import Storage
from vyra_base.core.parameter import Parameter
from vyra_base.core.volatile import Volatile
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.security import SecurityManager, SecurityLevel
from vyra_base.helper.logging_config import VyraLoggingConfig

logger = logging.getLogger(__name__)


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
    :vartype state_machine: UnifiedStateMachine
    :ivar security_manager: Security manager for authentication and session management (optional).
    :vartype security_manager: Optional[SecurityManager]

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
            log_config: Optional[dict[str, Any]] = None,
            register_protocols: Optional[list[ProtocolType]] = None) -> None:
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

        # Check ROS2 availability
        self._ros2_available = _ROS2_AVAILABLE
        
        if not self._ros2_available:
            logger.warning(
                "⚠️ ROS2 not available. "
                "Entity will run in slim mode with SharedMemory/UDS/Redis communication. "
                "Install ROS2 and source setup.bash to enable ROS2 features."
            )

        # Only check node availability if ROS2 is available
        if self._ros2_available and VyraEntity._check_node_availability(module_entry.name):
            raise RuntimeError(
                f"Node {module_entry.name} is available in the ROS2 system."
                " Please choose a different name."
            )

        self.module_entry: ModuleEntry = module_entry
        self.module_config: dict[str, Any] = module_config
        
        # Store ROS2 message types for creating entries
        self._error_entry_type = error_entry._type

        # Create ROS2 node only if ROS2 is available
        if self._ros2_available and VyraNode and NodeSettings:
            node_settings = NodeSettings(
                name=f"{self.module_entry.name}_{self.module_entry.uuid}"
            )
            self._node = VyraNode(node_settings)
            logger.info(f"✅ ROS2 node created: {self._node.get_name()}")
        else:
            self._node = None
            logger.info("🔵 No ROS2 node created (slim mode)")

        self.registered_protocols = register_protocols or [
            ProtocolType.ROS2,
            ProtocolType.ZENOH,
            ProtocolType.REDIS,
            ProtocolType.UDS
        ]

        feeder: tuple = self.__init_feeder(state_entry, news_entry, error_entry)
        self.state_feeder: StateFeeder = feeder[0]
        self.news_feeder: NewsFeeder = feeder[1]
        self.error_feeder: ErrorFeeder = feeder[2]

        self.state_machine: UnifiedStateMachine = self._init_state_machine(state_entry)

        self._init_security_manager(module_config)

        VyraEntity.register_service_callbacks(self)

        self.news_feeder.feed("...V.Y.R.A. entity initialized")

    @property
    def node(self) -> Optional['VyraNode']:
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
        if not hasattr(self, '_node') or self._node is None:
            return ''
        else:
            return self._node.get_namespace()

    async def _register_transport_provider(self, register_types: list[ProtocolType]) -> None:
        """
        Register the transport protocol providers based on availability.

        This method registers the appropriate protocol providers (ROS2, Zenoh, Redis, etc.)
        with the InterfaceFactory to enable communication for the entity.
        """
        providers = []
        
        if self._ros2_available and self._node is not None and ProtocolType.ROS2 in register_types:
            node_name = self._node.node_settings.name
            from vyra_base.com.transport.t_ros2.provider import ROS2Provider

            ros2_provider = ROS2Provider(
                module_name=self.module_entry.name, 
                module_id=self.module_entry.uuid
            )
            
            await ros2_provider.initialize({
                "node_name": node_name,
                "namespace": self._node.get_namespace(),
                "use_simulation_time": self.module_config.get("use_simulation_time", False)
            })
            providers.append(ros2_provider)
            logger.info("✅ Registered ROS2 protocol provider")
        
        if ProtocolType.ZENOH in register_types:
            try:
                if ZENOH_AVAILABLE:
                    zenoh_provider = ZenohProvider(
                        module_name=self.module_entry.name, 
                        module_id=self.module_entry.uuid
                    )
                    
                    zenoh_config = self.module_config.get("zenoh", {
                        "mode": "client",
                        "connect": ["tcp/zenoh-router:7447"]
                    })
                    await zenoh_provider.initialize(zenoh_config)
                    providers.append(zenoh_provider)
                    logger.info("✅ Registered Zenoh protocol provider")
                else:
                    logger.warning("⚠️ Zenoh not available, skipping provider registration")
            except Exception as e:
                logger.warning(f"⚠️ Failed to register Zenoh provider: {e}")
        
        if ProtocolType.REDIS in register_types:
            from vyra_base.com.transport.t_redis.provider import RedisProvider

            redis_provider = RedisProvider(
                module_name=self.module_entry.name, 
                module_id=self.module_entry.uuid
            )
            
            providers.append(redis_provider)
            logger.info("✅ Registered Redis protocol provider")

        if ProtocolType.UDS in register_types:
            from vyra_base.com.transport.t_uds.provider import UDSProvider

            uds_provider = UDSProvider(
                module_name=self.module_entry.name, 
                module_id=self.module_entry.uuid
            )
            
            providers.append(uds_provider)
            logger.info("✅ Registered UDS protocol provider")
        
        # Register all available providers
        InterfaceFactory.register_provider(providers)

    def _unregister_transport_providers(self) -> None:
        """
        Unregister all transport protocol providers.

        This method removes all registered protocol providers from the InterfaceFactory.
        """
        for protocol in self.registered_protocols:
            InterfaceFactory.unregister_provider(protocol)
            logger.info(f"✅ Unregistered {protocol.value} protocol provider")
    
    def set_interface_paths(self, interface_paths: list[str | Path]) -> None:
        """
        Set interface base paths for dynamic interface discovery.
        
        Updates the global InterfacePathRegistry used by all TopicBuilders
        and InterfaceLoaders in this entity's protocol providers.
        
        Must be called BEFORE set_interfaces() to affect interface loading.
        
        Args:
            interface_paths: List of base interface directory paths.
                Each path should point to a directory containing:
                - /config/*.json - Interface metadata
                - /service/*.srv - ROS2 service definitions
                - /publisher/*.msg - ROS2 message definitions
                - /actionServer/*.action - ROS2 action definitions
                - /proto/*.proto - Protocol Buffer definitions
                - /proto/*_pb2.py - Generated Python protobuf modules
        
        Raises:
            ValueError: If no valid paths provided
        
        Examples:
            >>> from ament_index_python.packages import get_package_share_directory
            >>> entity = VyraEntity(...)
            >>> 
            >>> # Set custom interface paths
            >>> module_interfaces = Path(get_package_share_directory("v2_modulemanager_interfaces"))
            >>> vyra_interfaces = Path(get_package_share_directory("vyra_module_template_interfaces"))
            >>> entity.set_interface_paths([module_interfaces, vyra_interfaces])
            >>> 
            >>> # Now set_interfaces() will use dynamic loading from these paths
            >>> await entity.set_interfaces(base_interfaces)
        """
        
        # Validate and convert paths
        validated_paths = []
        for path in interface_paths:
            path_obj = Path(path).resolve()
            if not path_obj.exists():
                logger.warning(f"⚠️ Interface path does not exist: {path_obj}")
                continue
            if not path_obj.is_dir():
                logger.warning(f"⚠️ Interface path is not a directory: {path_obj}")
                continue
            validated_paths.append(path_obj)
        
        if not validated_paths:
            raise ValueError(
                "No valid interface paths provided. At least one valid path required."
            )
        
        # Update global registry
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(p) for p in validated_paths])
        
        logger.info(
            f"✅ Interface paths configured: {len(validated_paths)} path(s)"
        )
        for idx, path in enumerate(validated_paths, 1):
            logger.info(f"  [{idx}] {path}")
        
        # Optionally update environment for ROS2 discovery
        from vyra_base.helper.ros2_env_helper import ensure_workspace_discoverable
        
        for path in validated_paths:
            # Try to find workspace install directory (typically 3 levels up from share/<package>)
            if "share" in path.parts:
                share_idx = path.parts.index("share")
                if share_idx >= 2:
                    install_dir = Path(*path.parts[:share_idx])
                    if install_dir.name == "install":
                        count = ensure_workspace_discoverable(install_dir)
                        if count > 0:
                            logger.debug(
                                f"✓ Made {count} package(s) discoverable from: {install_dir}"
                            )

    def _init_logger(self, log_config: Optional[dict[str, Any]]) -> None:
        """
        Initialize the logger for the entity.
        
        This method sets up the logger configuration based on the provided log configuration.
        Uses the centralized VyraLoggingConfig for professional logging setup.
        
        Args:
            log_config: Optional custom logging configuration dict.
                       Can include: log_level, log_directory, enable_console, enable_file
        """
        # Extract logging parameters from config
        log_level = log_config.get("log_level") if log_config else None
        log_directory = log_config.get("log_directory") if log_config else None
        enable_console = log_config.get("enable_console", True) if log_config else True
        enable_file = log_config.get("enable_file", True) if log_config else True
        
        # Initialize centralized logging configuration
        VyraLoggingConfig.initialize(
            log_level=log_level,
            log_directory=Path(log_directory) if log_directory else None,
            enable_console=enable_console,
            enable_file=enable_file,
        )

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
            module_entity=self.module_entry
        )

        news_feeder = NewsFeeder(
            type=news_entry._type, 
            node=self._node,
            module_entity=self.module_entry,
            loggingOn=True
        )
        
        error_feeder = ErrorFeeder(
            type=error_entry._type, 
            node=self._node,
            module_entity=self.module_entry,
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
        logger.debug("Initializing parameters for the entity.")
        
        self.default_database_access = DbAccess(
            module_name=self.module_entry.name,
            db_config=default_config
        )

        self.param_manager = Parameter(
            parameter_base_types=parameter_base_types,
            node=self._node,
            storage_access_persistant=self.database_access,
            storage_access_persistant_default=self.default_database_access,
            storage_access_transient=self.redis_access
        )

        VyraEntity.register_service_callbacks(self.param_manager)

        await self.param_manager.load_defaults()

    def _init_volatiles(self, transient_base_types: dict[str, Any]) -> None:
        """
        Initialize volatile parameters for the entity. Volatile parameters are stored in Redis.
        Thus they could only be used during runtime and are not persisted.

        This method sets up the volatile parameters using Redis for transient storage.
        It should be called during the initialization of the entity.
        
        :param transient_base_types: Dictionary containing base types for volatile parameters.
        :type transient_base_types: dict[str, Any]
        """
        logger.debug("Initializing volatile parameters for the entity.")
        
        self.volatile = Volatile(
            storage_access_transient=self.redis_access,
            module_id=self.module_entry.uuid,
            node=self._node,
            transient_base_types=transient_base_types
        )

        VyraEntity.register_service_callbacks(self.volatile)

    def _init_state_machine(self, state_entry: StateEntry) -> UnifiedStateMachine:
        """
        Initialize the 3-layer state machine for the entity.

        This method sets up the unified state machine with three layers:
        - Lifecycle Layer: Controls module existence (startup, shutdown, recovery)
        - Operational Layer: Manages runtime activity (tasks, processing)
        - Health Layer: Monitors system integrity (warnings, faults)

        The state machine follows industrial standards (IEC 61508, ISO 13849) and
        provides thread-safe, event-driven state management with complete audit trail.
        
        :param state_entry: The state entry configuration (currently for backward compatibility).
        :type state_entry: StateEntry
        :return: Initialized unified state machine.
        :rtype: UnifiedStateMachine
        """
        logger.info(f"Initializing 3-layer state machine for entity '{self.module_entry.name}'")
        
        # Create configuration for state machine
        config = StateMachineConfig(
            # Initial states (module starts uninitialized)
            initial_lifecycle=LifecycleState.INITIALIZING,
            initial_operational=OperationalState.IDLE,
            initial_health=HealthState.HEALTHY,
            
            # Enable detailed transition logging
            enable_transition_log=True,
            
            # Maximum history size (for debugging and audit)
            max_history_size=1000,
            
            # Strict mode for production (raises exceptions on invalid transitions)
            strict_mode=True,
        )
        
        # Create unified state machine
        state_machine = UnifiedStateMachine(config)
        
        # Register state change callbacks for integration with feeders
        self._register_state_callbacks(state_machine)
        
        logger.info(
            f"State machine initialized: "
            f"lifecycle={state_machine.get_lifecycle_state().value}, "
            f"operational={state_machine.get_operational_state().value}, "
            f"health={state_machine.get_health_state().value}"
        )
        
        return state_machine
    
    def _register_state_callbacks(self, state_machine: UnifiedStateMachine) -> None:
        """
        Register callbacks for state changes to integrate with feeders.
        
        This method connects the state machine to the entity's communication infrastructure
        by registering callbacks that feed state changes to the ROS2 system.
        
        :param state_machine: The unified state machine instance.
        :type state_machine: UnifiedStateMachine
        """
        def on_lifecycle_change(layer: str, old_state: str, new_state: str):
            """Callback for lifecycle state changes."""
            logger.info(f"Lifecycle transition: {old_state} → {new_state}")
            
            # Feed state change to ROS2
            state_data = StateEntry(
                previous=old_state,
                trigger="lifecycle_event",
                current=new_state,
                module_id=self.module_entry.uuid,
                module_name=self.module_entry.name,
                timestamp=datetime.now(),
                _type="lifecycle"
            )
            self.state_feeder.feed(state_data)
            
            # Log important lifecycle transitions
            if new_state == "Active":
                self.news_feeder.feed(f"Module '{self.module_entry.name}' is now active")
            elif new_state == "Recovering":
                self.news_feeder.feed(f"Module '{self.module_entry.name}' entered recovery mode")
            elif new_state == "Deactivated":
                self.news_feeder.feed(f"Module '{self.module_entry.name}' has been deactivated")
        
        def on_operational_change(layer: str, old_state: str, new_state: str):
            """Callback for operational state changes."""
            logger.debug(f"Operational transition: {old_state} → {new_state}")
            
            # Feed state change to ROS2
            state_data = StateEntry(
                previous=old_state,
                trigger="operational_event",
                current=new_state,
                module_id=self.module_entry.uuid,
                module_name=self.module_entry.name,
                timestamp=datetime.now(),
                _type="operational"
            )
            self.state_feeder.feed(state_data)
        
        def on_health_change(layer: str, old_state: str, new_state: str):
            """Callback for health state changes."""
            logger.info(f"Health transition: {old_state} → {new_state}")
            
            # Feed state change to ROS2
            state_data = StateEntry(
                previous=old_state,
                trigger="health_event",
                current=new_state,
                module_id=self.module_entry.uuid,
                module_name=self.module_entry.name,
                timestamp=datetime.now(),
                _type="health"
            )
            self.state_feeder.feed(state_data)
            
            # Report health issues
            if new_state == "Warning":
                self.news_feeder.feed(f"Module '{self.module_entry.name}' health warning")
            elif new_state == "Faulted":
                error_entry = ErrorEntry(
                    _type=self._error_entry_type,
                    level=ErrorEntry.ERROR_LEVEL.MAJOR_FAULT,
                    description=f"Module '{self.module_entry.name}' has faulted",
                    module_id=self.module_entry.uuid,
                    module_name=self.module_entry.name,
                    timestamp=datetime.now()
                )
                self.error_feeder.feed(error_entry)
            elif new_state == "Critical":
                error_entry = ErrorEntry(
                    _type=self._error_entry_type,
                    level=ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT,
                    description=f"CRITICAL: Module '{self.module_entry.name}' in critical state",
                    module_id=self.module_entry.uuid,
                    module_name=self.module_entry.name,
                    timestamp=datetime.now()
                )
                self.error_feeder.feed(error_entry)
        
        # Register callbacks with priorities (lifecycle highest, then health, then operational)
        state_machine.on_lifecycle_change(on_lifecycle_change, priority=10)
        state_machine.on_operational_change(on_operational_change, priority=5)
        state_machine.on_health_change(on_health_change, priority=8)
        
        logger.debug("State machine callbacks registered")
    
    async def startup_entity(self) -> bool:
        """
        Execute the entity startup sequence using the state machine.
        
        This method performs the complete initialization sequence:
        1. Start initialization (Lifecycle: Uninitialized → Initializing)
        2. Initialize resources (storages, parameters, etc.)
        3. Complete initialization (Lifecycle: Initializing → Active)
        4. Set operational ready (Operational: Idle → Ready)
        
        :return: True if startup successful, False otherwise.
        :rtype: bool
        """
        try:
            logger.info(f"Starting entity '{self.module_entry.name}' startup sequence")
            
            # Step 1: Begin initialization
            self.state_machine.start(metadata={
                "entity": self.module_entry.name,
                "uuid": self.module_entry.uuid,
                "timestamp": "startup_initiated"
            })
            
            # Step 2: Initialize transport providers
            await self._register_transport_provider(self.registered_protocols)

            # Step 3: Initialize resources (storages would be initialized here)
            await self.state_feeder.start()
            await self.news_feeder.start()
            await self.error_feeder.start()

            # This is where setup_storage and other initialization would happen
            
            # Step 3: Complete initialization successfully
            self.state_machine.complete_initialization(result={
                "status": "success",
                "entity": self.module_entry.name
            })
            
            # Step 4: Set operational ready
            self.state_machine.set_ready(metadata={
                "capabilities": "full",
                "ready_for_tasks": True
            })
            
            logger.info(f"Entity '{self.module_entry.name}' startup completed successfully")
            self.news_feeder.feed(f"Entity '{self.module_entry.name}' is ready for operation")
            
            return True
            
        except Exception as e:
            logger.error(f"Entity startup failed: {e}")
            
            # Mark initialization as failed
            try:
                self.state_machine.fail_initialization(error=str(e))
                error_entry = ErrorEntry(
                    _type=self._error_entry_type,
                    level=ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT,
                    description=f"Entity startup failed: {e}",
                    solution="Check entity initialization and module dependencies",
                    module_id=self.module_entry.uuid,
                    module_name=self.module_entry.name,
                    timestamp=datetime.now()
                )
                self.error_feeder.feed(error_entry)
            except:
                pass  # State machine might already be in invalid state
            
            return False
    
    async def shutdown_entity(self) -> bool:
        """
        Execute the entity shutdown sequence using the state machine.
        
        This method performs graceful shutdown:
        1. Begin shutdown (Lifecycle: Active → ShuttingDown)
        2. Clean up resources (operational tasks, storages, etc.)
        3. Complete shutdown (Lifecycle: ShuttingDown → Deactivated)
        
        :return: True if shutdown successful, False otherwise.
        :rtype: bool
        """
        try:
            logger.info(f"Starting entity '{self.module_entry.name}' shutdown sequence")
            
            # Step 1: Begin shutdown
            self.state_machine.shutdown(reason="graceful_shutdown")
            
            # Step 2: Clean up resources
            # This is where cleanup would happen (close connections, save state, etc.)
            self._unregister_transport_providers()
            
            # Step 3: Complete shutdown
            self.state_machine.complete_shutdown()
            
            logger.info(f"Entity '{self.module_entry.name}' shutdown completed")
            self.news_feeder.feed(f"Entity '{self.module_entry.name}' has been shut down")
            
            return True
            
        except Exception as e:
            logger.error(f"Entity shutdown failed: {e}")
            error_entry = ErrorEntry(
                _type=self._error_entry_type,
                level=ErrorEntry.ERROR_LEVEL.MAJOR_FAULT,
                description=f"Entity shutdown error: {e}",
                solution="Check entity cleanup and resource deallocation",
                module_id=self.module_entry.uuid,
                module_name=self.module_entry.name,
                timestamp=datetime.now()
            )
            self.error_feeder.feed(error_entry)
            return False

    def _init_security_manager(self, module_config: dict[str, Any]):
        """
        Initialize the security manager for the entity.

        This method sets up the security manager based on the module configuration.
        The security manager provides authentication, session management, and
        cryptographic validation for inter-module communication.
        
        Configuration example:
        {
            'security': {
                'enabled': True,
                'max_level': 4,  # SecurityLevel.HMAC
                'session_duration': 3600,  # seconds
                'ca_key_path': '/path/to/ca.key',  # required for level 5
                'ca_cert_path': '/path/to/ca.cert',  # required for level 5
                'module_passwords': {'module_id': 'password'}  # for level 3+
            }
        }
        
        :param module_config: Module configuration dictionary
        :type module_config: dict[str, Any]
        :returns: Initialized SecurityManager instance
        :rtype: SecurityManager
        """
        security_config = module_config.get('security', {})
        
        max_level = security_config.get('max_level', SecurityLevel.BASIC_AUTH)
        session_duration = security_config.get('session_duration', 3600)
        ca_key_path = security_config.get('ca_key_path')
        ca_cert_path = security_config.get('ca_cert_path')
        module_passwords = security_config.get('module_passwords', {})
        
        logger.info(
            f"Initializing SecurityManager with max level: "
            f"{SecurityLevel.get_name(max_level)}"
        )
        
        self.security_manager = SecurityManager(
            max_security_level=max_level,
            session_duration_seconds=session_duration,
            ca_key_path=ca_key_path,
            ca_cert_path=ca_cert_path,
            module_passwords=module_passwords
        )

        VyraEntity.register_service_callbacks(self.security_manager)

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
            logger.warning("No storage configuration provided. Skipping storage setup.")
            return

        persistent_config: dict[str, Any] = {}
        transient_config: dict[str, Any] = {}

        for config_key in config.keys():
            if config_key in (item.value for item in DBTYPE):
                logger.debug(f"Configuring {config_key} for persistent storage.")
                persistent_config[config_key] = config[config_key]
            elif config_key == "redis":
                logger.debug(f"Configuring {config_key} for transient storage.")
                transient_config[config_key] = config[config_key]

        if not persistent_config or not transient_config:
            logger.warning(
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
            logger.warning(
                "No default database configuration provided. "
                "Skipping parameter initialization.")
            return
        else:
            dtype = self.database_access.db_type
            persistent_config[dtype]['database'] = persistent_config[dtype]['default_database']
            
            await self._init_params(persistent_config, parameter_base_types)

        self._init_volatiles(transient_base_types=transient_base_types)

        logger.info("Storage access initialized.")

    async def set_interfaces(
            self, 
            settings: list[FunctionConfigEntry]) -> None:
        """
        Add communication interfaces to this module.

        Interfaces are used to communicate with other modules or external systems.
        Interface types can be:

        - ``!vyra-service``: service function that can be invoked remotely.
        - ``!vyra-actionServer``: actionServer that can be executed in the background.
        - ``!vyra-publisher``: Publisher that can publish messages periodically.
        
        Protocols used depend on availability:
        - ROS2 (if available)
        - SharedMemory (always available)
        - UDS (always available)
        - Redis (if configured)
        - gRPC (if configured)

        :param settings: Settings for the module functions.
        :type settings: list[FunctionConfigEntry]
        :param permission_file: Path to the permission file for the interface.
        :type permission_file: str
        :returns: None
        :rtype: None
        """

        for setting in settings:
            if setting.functionname in [i.functionname for i in self._interface_list]:
                logger.warning(
                    f"Interface {setting.functionname} already "
                    "exists. Skipping creation.")
                continue
            
            self._interface_list.append(setting)

            # Use ROS2 if available, otherwise skip ROS2-specific creation
            # (InterfaceFactory will handle protocol selection in future versions)
            if not self._ros2_available:
                logger.debug(
                    f"⚠️ ROS2 not available, skipping ROS2 interface creation for {setting.functionname}. "
                    "Use InterfaceFactory for multi-protocol support."
                )
                continue

            if setting.type == FunctionConfigBaseTypes.service.value:
                logger.info(f"Creating callable: {setting.functionname}")
                await InterfaceFactory.create_server(
                    name=setting.functionname,
                    response_callback=setting.callback,
                    protocols=[ProtocolType.ROS2],
                    service_type=setting.interfacetypes,
                    node=self._node
                )
            elif setting.type == FunctionConfigBaseTypes.action.value:
                logger.info(f"Creating actionServer: {setting.functionname}")
                # TBD: vyra actionServer implementation
                # create_vyra_actionServer(...)
                logger.warning(f"Vyra actionServers are not implemented yet.")

            elif setting.type == FunctionConfigBaseTypes.publisher.value:
                logger.info(f"Creating publisher: {setting.functionname}")
                
                # Extract periodic parameters
                periodic: bool = False
                periodic_caller: Any = None
                periodic_interval: Union[float, None] = None

                if setting.periodic != None:
                    periodic = True
                    periodic_caller = setting.periodic.caller if periodic else None
                    periodic_interval = setting.periodic.interval if periodic else None
                
                # Publisher creation through InterfaceFactory
                await InterfaceFactory.create_publisher(
                    name=setting.functionname,
                    protocols=[ProtocolType.ROS2],
                    message_type=setting.interfacetypes,
                    node=self._node,
                    is_publisher=True,
                    qos_profile=setting.qosprofile,
                    description=setting.description,
                    periodic=periodic,
                    interval_time=periodic_interval,
                    periodic_caller=periodic_caller
                )
            else:
                fail_msg = (
                    f"Unsupported interface type: {setting.type}. "
                    "Supported types are publisher, service, action."
                )
                logger.error(fail_msg)
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
        logger.info(f"Storage {storage} registered successfully.")

    @classmethod
    def _check_node_availability(cls, node_name: str) -> bool:
        """
        Check if a node with the given name is available in the ROS2 system.

        :param node_name: The name of the node to check.
        :type node_name: str
        :returns: True if the node is available, False otherwise.
        :rtype: bool
        """
        if CheckerNode is None:
            logger.warning("CheckerNode not available. Check if ROS2 is available")
            return False
        
        checker_node = CheckerNode()
        return checker_node.is_node_available(node_name)
    
    @remote_service()
    async def get_interface_list(self, request: Any, response: Any) -> Any:
        """
        Retrieves all capabilities (publisher, service, action) of the entity that are set to
        visible for external access (ROS2 service interface).
        
        This is the external ROS2 service endpoint. For internal calls,
        use :meth:`get_interface_list_impl` instead.
        
        Returns a list of JSON-serialized interface configurations showing which
        ROS2 services, topics, and jobs are available for this module.

        :param request: The request object (unused).
        :type request: Any
        :param response: The response object to update with the result.
        :type response: Any
        :return: None
        :rtype: Any
        
        **ROS2 Service Usage:**
        
        .. code-block:: bash
        
            ros2 service call /module_name/get_interface_list \\
                vyra_base_interfaces/srv/GetInterfaceList "{}"
        
        **Internal Usage:**
        
        .. code-block:: python
        
            result = await entity.get_interface_list_impl()
            if result:
                interfaces = [json.loads(i) for i in result["interface_list"]]
                for interface in interfaces:
                    print(f"Interface: {interface['functionname']}")
                    print(f"  Type: {interface['type']}")
                    print(f"  Description: {interface['description']}")
        """
        interface_return = await self.get_interface_list_impl()
        if interface_return is None:
            response.interface_list = []
            return None
        
        response.interface_list = interface_return["interface_list"]
        return None
    
    async def get_interface_list_impl(self) -> Optional[dict]:
        """
        Retrieves all capabilities (publisher, service, action) of the entity that are set to
        visible for external access (internal implementation).
        
        This method filters the registered interfaces for those marked as visible
        and returns them as JSON-serialized strings. Useful for service discovery
        and dynamic interface introspection.

        :return: Dictionary containing list of interfaces, or None on error.
        :rtype: Optional[dict]
        
        **Return format:**
        
        .. code-block:: python
        
            {
                "interface_list": [
                    '{"functionname": "...", "type": "service", ...}',
                    '{"functionname": "...", "type": "publisher", ...}',
                    ...
                ]
            }
        
        **Example:**
        
        .. code-block:: python
        
            result = await entity.get_interface_list_impl()
            if result:
                for interface_json in result["interface_list"]:
                    interface = json.loads(interface_json)
                    
                    if interface["type"] == "service":
                        print(f"Service: {interface['functionname']}")
                    elif interface["type"] == "publisher":
                        print(f"Topic: {interface['functionname']}")
        """
        response: dict = {}
        response_interface_list = []
        
        for interface in self._interface_list:
            if interface.displaystyle.visible:
                response_interface_list.append(json.dumps(interface.asdict()))

        response["interface_list"] = response_interface_list
        return response

    @staticmethod
    def register_service_callbacks(callback_parent: object):
        """
        Registers all remote services defined in the callback_parent. 
        
        Remote services must be decorated with ``@remote_service``.

        Example::

            from vyra_base.com import remote_service

            class MyParentClass:
                @remote_service
                async def my_remote_function(self, request: Any, response: Any):
                    pass

            instance_my_parent = MyParentClass()

        To register the remote service in this example, the instance_my_parent
        object must be passed to this function:
        
        - ``register_services_callbacks(instance_my_parent)``

        Inside your MyParentClass in a method you can call the same function and
        set the callback_parent to self to register the services of the
        instance itself:
        
        - ``register_services_callbacks(self)``

        This function will iterate over all attributes of the instance and
        register those marked as remote service with the DataSpace.

        .. warning::
           This function will only register the callbacks. Run 
           ``entity.set_interfaces(your_config)`` afterwards to load the interfaces 
           in vyra.

        :param callback_parent: The class or instance containing the remote services.
        :type callback_parent: Type[object]
        :raises TypeError: If callback_parent is not an instance of object.
        :raises ValueError: If callback_parent does not have any remote services.
        :raises RuntimeError: If the service registration fails.
        :returns: None
        :rtype: None
        """
        for attr_name in dir(callback_parent):
            attr = getattr(callback_parent, attr_name)
            rc_active = getattr(attr, "_remote_service", False)

            if callable(attr) and rc_active:
                # Interface automatically registered via @remote_service decorator
                logger.debug(
                    f"Remote service <{attr.__name__}> registered via decorator")

                # Set on the underlying function object
                if hasattr(attr, "__func__"):
                    setattr(attr.__func__, "_remote_service", False)
                else:
                    setattr(attr, "_remote_service", False)