from __future__ import annotations

from datetime import datetime
from transitions import Machine
from typing import Callable, List
from typing import Any
from vyra_base.helper.logger import LogEntry
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.helper.logger import LogMode as LogMode
from vyra_base.helper.error_handler import ErrorTraceback

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
        self._state_entry: StateEntry = StateEntry(
            previous='',  # type: ignore
            current=Vyra_STATES.Resting,  # type: ignore
            module_id=self.module_config.uuid,
            module_name=self.module_config.name,
            timestamp=datetime.now(),
            type=self.state_type
        )

        self.model: StateModel = StateModel(
            state_entry=self._state_entry, 
            state_feed=self.state_feed,
            module_config=self.module_config
        )

        config_collection['model'] = self.model  # adding a model to the configuration

        self._machine: Machine = Machine(**config_collection)  # **modelConfig unpacks arguments as kwargs

    @property
    def current_state(self) -> str:
        """ Returns the current state of the state machine. """
        return self._state_entry.current

    @property
    def all_transitions(self) -> List[str]:
        """ Returns all transitions of the state machine. """
        return config_collection['transitions']

    def is_transition_possible(self, trigger_name):
        triggers: List[str] = self._machine.get_triggers(self.model.state)
        return trigger_name in triggers

class StateModel:
    """ State class for all states.
    """
    state: str

    def __init__(
            self, 
            state_entry: StateEntry, 
            state_feed: StateFeeder, 
            module_config: ModuleEntry):
        
        Logger.log(f'Initialize state model with state {state_entry.current}.')

        self._state_entry: StateEntry = state_entry
        self._state_feed: StateFeeder = state_feed
        self._module_config: ModuleEntry = module_config

    @property
    def state_entry(self) -> StateEntry:
        """ Returns the current state of the state model. """
        return self._state_entry

    def base_enter(self):
        """On enter wrapper to be used for all on_enter methods.
        This wrapper updates the state entry and pushes it to the
        state feed.
        """
        try:
            self._state_entry.previous = self._state_entry.current
            self._state_entry.current = self.state
            self._state_entry.timestamp = datetime.now()
            
            Logger.debug("Sending state entry to state feed.")
            self._state_feed.feed(
                self._state_entry
            )
        finally:
            ErrorTraceback.check_error_exist()
    
    def on_enter_Resting(self):
        """On enter function for the Resting state."""
        self.base_enter()
        Logger.debug("Entering Resting state.")
    
    def on_enter_Awakening(self):
        """On enter function for the Awakening state."""
        self.base_enter()
        Logger.debug("Entering Awakening state.")

    def on_enter_Attentive(self):
        """On enter function for the Attentive state."""
        self.base_enter()
        Logger.debug("Entering Attentive state.")

    def on_enter_Active(self):
        """On enter function for the Active state."""
        self.base_enter()
        Logger.debug("Entering Active state.")

    def on_enter_Reflecting(self):
        """On enter function for the Reflecting state."""
        self.base_enter()
        Logger.debug("Entering Reflecting state.")

    def on_enter_Learning(self):
        """On enter function for the Learning state."""
        self.base_enter()
        Logger.debug("Entering Learning state.")

    def on_enter_Alert(self):
        """On enter function for the Alert state."""
        self.base_enter()
        Logger.debug("Entering Alert state.")

    def on_enter_Delegating(self):
        """On enter function for the Delegating state."""
        self.base_enter()
        Logger.debug("Entering Delegating state.")

    def on_enter_Recovering(self):
        """On enter function for the Recovering state."""
        self.base_enter()
        Logger.debug("Entering Recovering state.")

    def on_enter_Overloaded(self):
        """On enter function for the Overloaded state."""
        self.base_enter()
        Logger.debug("Entering Overloaded state.")

    def on_enter_ShuttingDown(self):
        """On enter function for the ShuttingDown state."""
        self.base_enter()
        Logger.debug("Entering ShuttingDown state.")

    def on_enter_Interrupting(self):
        """On enter function for the Interrupting state."""
        self.base_enter()
        Logger.debug("Entering Interrupting state.")

      
# EOF: state_machine.py