
from __future__ import annotations


import asyncio

from typing import Any
from typing import Union

from pathlib import Path

from rclpy.qos import QoSProfile

from vos_base.com.datalayer.interface_factory import remote_callable
from vos_base.com.datalayer.interface_factory import create_vos_callable
from vos_base.com.datalayer.interface_factory import create_vos_job
from vos_base.com.datalayer.interface_factory import create_vos_speaker

from vos_base.defaults.entries import FunctionConfigEntry
from vos_base.defaults.entries import FunctionConfigBaseTypes

from vos_base.state.state_machine import StateMachine

from vos_base.com.datalayer.node import VOSNode
from vos_base.com.datalayer.node import NodeSettings
from vos_base.com.datalayer.node import CheckerNode
from vos_base.com.feeder.state_feeder import StateFeeder
from vos_base.defaults.entries import StateEntry
from vos_base.defaults.entries import ModuleEntry
from vos_base.state import state_machine
from vos_base.helper.logger import Logger
from vos_base.com.datalayer.callable import VOSCallable
from vos_base.com.datalayer.interface_factory import DataSpace


class VOSEntity:

    """
    Base class for all VOS entities.
    """

    def __init__(
            self, 
            state_entry: StateEntry,
            module_config: ModuleEntry) -> None:
        
        self.register_remote_callables()

        if VOSEntity._check_node_availability(module_config.name):
            raise RuntimeError(
                f"Node {module_config.name} is available in the ROS2 system."
                " Please choose a different name."
            )

        self.module_config: ModuleEntry = module_config

        node_settings = NodeSettings(
            name=f"{self.module_config.name}"
        )

        self.ros2_node = VOSNode(node_settings)

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

        log_config_path = Path(__file__).resolve().parent.parent
        log_config_path: Path = log_config_path / "helper" / "logger_config.json"

        Logger.initialize(log_config_path=log_config_path)

    def register_remote_callables(self):
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if callable(attr) and getattr(attr, "_remote_callable", False):
                callable_obj = VOSCallable(
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
            - !vos-callable: Callable function that can be invoked remotely.
            - !vos-job: Job that can be executed in the background.
            - !vos-speaker: Speaker that can publish messages periodically.
        
        Args:
            settings (dict): Settings for the module functions.
        
        Returns:
            dict: A dictionary containing the built module functions.
        """
        self._module_function_list = settings

        async_loop = asyncio.get_event_loop()

        for setting in settings:
            
            if setting.type == FunctionConfigBaseTypes.callable.value:
                print(f"Creating callable: {setting.functionname}")
                create_vos_callable(
                    name=setting.functionname,
                    type=setting.ros2type,
                    node=self.ros2_node,
                    callback=setting.callback,
                    async_loop=async_loop
                )
            elif setting.type == FunctionConfigBaseTypes.job.value:
                print(f"Creating job: {setting.functionname}")
                create_vos_job(
                    name=setting.functionname,
                    type=setting.ros2type,
                    async_loop=async_loop
                )
            elif setting.type == FunctionConfigBaseTypes.speaker.value:
                print(f"Creating speaker: {setting.functionname}")
                periodic: bool = False
                periodic_caller: Any = None
                periodic_interval: Union[float, None] = None

                if setting.periodic != None:
                    periodic: bool = True
                    periodic_caller = setting.periodic.caller if periodic else None
                    periodic_interval = setting.periodic.interval if periodic else None
                
                create_vos_speaker(
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
        if not self.state_machine.is_transition_possible(request.transition_name):
            response.success = False
            response.message = (
                f"Transition {request.transition_name} not possible "
                f"in current state {self.state_machine.current_state}."
            )
            return

        response.success = True
        response.message = f"Transition {request.transition_name} triggered successfully."

    
    @remote_callable
    async def get_capabilities(self) -> Any:
        pass

    
    # async def activate(user: str, psw: str, ident: str) -> str:
    #     """Activate Module.

    #     [Need to be activated before any action can be executed. Module
    #     can only activated by user and password]

    #     Args:
    #         user ([type]): [User-Name that is listed within the module]
    #         psw ([type]): [Password that is listed within the module]
    #         id ([type]): [User-Id that is listed within the module]

    #     Raises:
    #         ValueError: [description]

    #     Returns:
    #         [list]: [[True|False, Message]]
    #     """
    #     try:
    #         if str(user) == OpcuaRemote.user_name and \
    #             str(psw) == OpcuaRemote.password and (
    #                 str(ident) in OpcuaRemote.allowed_ids
    #                 or '--All' in OpcuaRemote.allowed_ids):

    #             if OpcuaRemote.state_machine_model.state_feed.get_feed_element() == \
    #                                          Constant.VOS_DEACTIVATED.value:
    #                 OpcuaRemote.state_machine_transition.s_activate()
    #                 Logger.log(f'User: {str(user)} \
    #                         successfully OpcuaRemote.logged in \
    #                         with ID: {str(ident)} at <{str(datetime.now())}>')
    #                 msg = StateInfo(True,
    #                     f'Access correct. Module {OpcuaRemote.module_id} \
    #                     is activated by {ident}',
    #                     OpcuaRemote.state_machine_model.state_feed.get_feed_element())

    #                 Logger.debug_print(f'{Style.get_style()} {msg.message}')
    #                 Logger.log(msg.message)
    #             else:
    #                 msg = StateInfo(False,
    #                     f'Module in {OpcuaRemote.state_machine_model \
    #                     .state_feed.get_feed_element()}-State. Cannot activate',
    #                     OpcuaRemote.state_machine_model.state_feed.get_feed_element())
    #                 Logger.log(f'{msg.message}')
    #             return json.dumps(str(msg))
    #         else:
    #             msg = StateInfo(False,
    #                 'Wrong user, password or id not listed. Please try again',
    #                 None)

    #             Logger.debug_print(f'{Style.get_style()} {msg}'[1])
    #             Logger.log('Wrong user {} or password {}'.format(user, str(psw)))
    #             Logger.log('Module accessed by {}'.format(ident))
    #             return json.dumps(str(msg))

    #     finally:
    #         if Utilities.check_error_exist():
    #             await OpcuaRemote.error_feed.append_object(
    #                 ErrorEntry(ErrorLookup.entry['0x40D'],
    #                      OpcuaRemote.module_id,
    #                      str(datetime.now())))