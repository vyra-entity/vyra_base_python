from __future__ import annotations

from datetime import datetime
from transitions import Machine
from typing import Callable
from typing import Any
from vyra_base.helper.logger import LogEntry
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.helper.logger import LogMode as LogMode

from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.defaults.entries import StateEntry
from vyra_base.helper.logger import Logger

from .state_config import Vyra_STATES
from .state_config import config_collection


class StateMachine:
    """ Initializes the global state machine for the module."""

    # __slots__ = (
    #     'state_feed', 
    #     'state_type',
    #     '_machine', 
    #     'model', 
    #     '_state',
    #     'module_config'
    # )
    
    def __init__(self, state_feed: StateFeeder, state_type: Any, module_config: ModuleEntry, ):
        self.state_feed: StateFeeder = state_feed
        self.state_type: Any = state_type
        self._machine: Machine
        self.model: StateModel
        self.module_config: ModuleEntry = module_config

    def initialize(self):
        self._state: StateEntry = StateEntry(
            Vyra_STATES.Resting,  # type: ignore
            Vyra_STATES.Resting,  # type: ignore
            self.module_config.uuid,
            self.module_config.name,
            datetime.now(),
            type=self.state_type
        )

        self.model: StateModel = StateModel(
            self._state, 
            self.state_feed,
            self.module_config
        )

        self.generate_state_door_methods(
            config_collection['states']
        )

        config_collection['model'] = self.model  # adding a model to the configuration

        self._machine: Machine = Machine(**config_collection)  # **modelConfig unpacks arguments as kwargs

    @property
    def current_state(self) -> str:
        """ Returns the current state of the state machine. """
        return self._state.current

    def is_transition_possible(self, trigger_name):
        Logger.log(f"Checking if transition at state '{self._state.current}' is possible '{self._machine.get_triggers(self._state.current)}'.")
        return trigger_name in self._machine.get_triggers(self._machine.state)

    def generate_state_door_methods(self, states: list[str]):
        """Generates methods for entering states dynamically."""
        for state in states:
            Logger.log(f"Generating state door methods 'on_enter_{state}' for state '{state}'.")
            setattr(self.model, f'on_enter_{state}', self.model.create_on_enter_function(state))
            setattr(self.model, f'on_exit_{state}', self.model.create_on_exit_function(state))
    
        Logger.log(self.model.__dict__)
    


class StateModel:
    """ State class for all states.
    """

    def __init__(
            self, 
            state: StateEntry, 
            state_feed: StateFeeder, 
            module_config: ModuleEntry):
        
        Logger.log(f'Initialize state model with state {state.current}.')

        self._state: StateEntry = state
        self._state_feed: StateFeeder = state_feed
        self._module_config: ModuleEntry = module_config

    def create_on_enter_function(self, state_name) -> Callable:
        def on_enter(self=self):
            Logger.debug(f"Entered state: {state_name}")
        return on_enter
    
    def create_on_exit_function(self, state_name) -> Callable:
        def on_exit(self=self):
            Logger.debug(f"Exit state: {state_name}")
        return on_exit
    
    # def on_enter_Awakening(self):
    #     """On enter function for the StartUp state."""
    #     Logger.debug("Entering StartUp state.")
        
      
# EOF: state_machine.py