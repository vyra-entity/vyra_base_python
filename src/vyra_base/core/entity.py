
from __future__ import annotations


import asyncio

from typing import Any
from typing import Union

from pathlib import Path

from rclpy.qos import QoSProfile

from vyra_base.com.datalayer.interface_factory import remote_callable
from vyra_base.com.datalayer.interface_factory import create_vyra_callable
from vyra_base.com.datalayer.interface_factory import create_vyra_job
from vyra_base.com.datalayer.interface_factory import create_vyra_speaker

from vyra_base.defaults.entries import FunctionConfigEntry
from vyra_base.defaults.entries import FunctionConfigBaseTypes

from vyra_base.state.state_machine import StateMachine

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.node import NodeSettings
from vyra_base.com.datalayer.node import CheckerNode
from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.defaults.entries import StateEntry
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.state import state_machine
from vyra_base.helper.logger import Logger
from vyra_base.com.datalayer.callable import VyraCallable
from vyra_base.com.datalayer.interface_factory import DataSpace


class VyraEntity:

    """
    Base class for all V.Y.R.A. entities.
    """

    def __init__(
            self, 
            state_entry: StateEntry,
            module_config: ModuleEntry) -> None:

        log_config_path = Path(__file__).resolve().parent.parent
        log_config_path: Path = log_config_path / "helper" / "logger_config.json"

        Logger.initialize(log_config_path=log_config_path)

        self.register_remote_callables()

        if VyraEntity._check_node_availability(module_config.name):
            raise RuntimeError(
                f"Node {module_config.name} is available in the ROS2 system."
                " Please choose a different name."
            )

        self.module_config: ModuleEntry = module_config

        node_settings = NodeSettings(
            name=f"{self.module_config.name}"
        )

        self.ros2_node = VyraNode(node_settings)

        self.state_feeder = StateFeeder(
            type=state_entry.type,
            node=self.ros2_node
        )

        self.state_machine = StateMachine(
            self.state_feeder,
            state_entry.type,
            module_config=self.module_config
        )

        self.state_machine.initialize()


    def register_remote_callables(self):
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if callable(attr) and getattr(attr, "_remote_callable", False):
                callable_obj = VyraCallable(
                    name=attr.__name__,
                    connected_callback=attr
                )
                print(f"Registering callable {callable_obj.name} from method {attr}")
                DataSpace.add_callable(callable_obj)


    async def add_interface(self, settings: list[FunctionConfigEntry]) -> None:
        """
        Add an dds communication interface to this module. Interfaces are used 
        to communicate with other modules or external systems.
        Interface types can be:
            - !vyra-callable: Callable function that can be invoked remotely.
            - !vyra-job: Job that can be executed in the background.
            - !vyra-speaker: Speaker that can publish messages periodically.

        Args:
            settings (dict): Settings for the module functions.
        
        Returns:
            dict: A dictionary containing the built module functions.
        """
        self._module_function_list = settings

        async_loop = asyncio.get_event_loop()

        for setting in settings:
            
            if setting.type == FunctionConfigBaseTypes.callable.value:
                Logger.info(f"Creating callable: {setting.functionname}")
                create_vyra_callable(
                    name=setting.functionname,
                    type=setting.ros2type,
                    node=self.ros2_node,
                    callback=setting.callback,
                    async_loop=async_loop
                )
            elif setting.type == FunctionConfigBaseTypes.job.value:
                Logger.info(f"Creating job: {setting.functionname}")
                create_vyra_job(
                    name=setting.functionname,
                    type=setting.ros2type,
                    async_loop=async_loop
                )
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
                    name=setting.functionname,
                    type=setting.ros2type,
                    node=self.ros2_node,
                    description=setting.description,
                    periodic=periodic,
                    interval_time=periodic_interval,
                    periodic_caller= periodic_caller,
                    qos_profile=setting.qosprofile,
                    async_loop=async_loop
                )

    @classmethod
    def _check_node_availability(cls, node_name: str) -> bool:
        """
        Check if a node with the given name is available in the ROS2 system.
        
        Args:
            node_name (str): The name of the node to check.
        
        Returns:
            bool: True if the node is available, False otherwise.
        """
        checker_node = CheckerNode()
        return checker_node.is_node_available(node_name)

    @remote_callable
    async def trigger_transition(self, request: Any, response: Any) -> None:
        """
        Trigger a state transition for the entity from internal or remote.
        
        Args:
            transition (str): The name of the transition to trigger.
        
        Raises:
            NotImplementedError: If the method is not implemented in the subclass.
        """
        trigger_list = [t['trigger'] for t in self.state_machine.all_transitions]
        if request.trigger_name not in trigger_list:
            fail_msg = (
                f"Transition {request.trigger_name} not found in "
                f"available transitions: {trigger_list}."
            )
            
            response.success = False
            response.message = fail_msg
            Logger.warning(fail_msg)
            return None
        
        if not self.state_machine.is_transition_possible(request.trigger_name):
            fail_msg = (
                f"Transition {request.trigger_name} not possible in "
                f"current state {self.state_machine.model.state}."
            )
            
            response.success = False
            response.message = fail_msg
            Logger.warning(fail_msg)
            return None
        
        getattr(self.state_machine.model, f"{request.trigger_name}")()

        response.success = True
        response.message = f"Transition {request.trigger_name} triggered successfully."

    
    @remote_callable
    async def get_capabilities(self) -> Any:
        pass