from __future__ import annotations

from datetime import datetime
from typing import Any, Callable, List

from transitions import Machine

from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.defaults.entries import ModuleEntry, StateEntry
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger, LogEntry, LogMode

from .state_config import Vyra_STATES, config_collection


class StateMachine:
    """
    Initializes the global state machine for the module.

    :param state_feed: The state feeder instance.
    :type state_feed: StateFeeder
    :param state_type: The type of the state.
    :type state_type: Any
    :param module_config: The module configuration entry.
    :type module_config: ModuleEntry
    """
    
    def __init__(self, state_feed: StateFeeder, state_type: Any, module_config: ModuleEntry, ):
        self.state_feed: StateFeeder = state_feed
        self.state_type: Any = state_type
        self._machine: Machine
        self.model: StateModel
        self.module_config: ModuleEntry = module_config

    def initialize(self):
        """
        Initializes the state machine and its model.
        """
        self._state_entry: StateEntry = StateEntry(
            previous='__CircleOfEternity__',  # ignore
            trigger='__toBeBorn__',  # ignore
            current=Vyra_STATES.Resting,
            module_id=self.module_config.uuid,
            module_name=self.module_config.name,
            timestamp=datetime.now(),
            _type=self.state_type
        )

        self.model: StateModel = StateModel(
            state_entry=self._state_entry, 
            state_feed=self.state_feed,
            module_config=self.module_config
        )

        config_collection['model'] = self.model  # adding a model to the configuration

        self._machine: Machine = Machine(**config_collection, send_event=True)  # **modelConfig unpacks arguments as kwargs

    @property
    def current_state(self) -> str:
        """
        Returns the current state of the state machine.

        :return: The current state.
        :rtype: str
        """
        return self._state_entry.current

    @property
    def all_transitions(self) -> List[dict]:
        """
        Returns all transitions of the state machine.

        :return: List of all transitions.
        :rtype: List[str]
        """
        return config_collection['transitions']

    def is_transition_possible(self, trigger_name: str) -> tuple[bool, List[str]]:
        """
        Checks if a transition is possible for the given trigger name.

        :param trigger_name: The name of the trigger.
        :type trigger_name: str
        :return: Tuple of (is_possible, available_triggers).
        :rtype: Tuple[bool, List[str]]
        """
        triggers: List[str] = self._machine.get_triggers(self.model.state)
        triggers = list(filter(lambda x: not x.startswith('to_'), triggers))
        return trigger_name in triggers, triggers

class StateModel:
    """
    State class for all states.

    :param state_entry: The state entry instance.
    :type state_entry: StateEntry
    :param state_feed: The state feeder instance.
    :type state_feed: StateFeeder
    :param module_config: The module configuration entry.
    :type module_config: ModuleEntry
    """
    state: str

    def __init__(
            self, 
            state_entry: StateEntry, 
            state_feed: StateFeeder, 
            module_config: ModuleEntry):
        """
        Initializes the state model.

        :param state_entry: The state entry instance.
        :type state_entry: StateEntry
        :param state_feed: The state feeder instance.
        :type state_feed: StateFeeder
        :param module_config: The module configuration entry.
        :type module_config: ModuleEntry
        """
        Logger.log(f'Initialize state model with state {state_entry.current}.')

        self._state_entry: StateEntry = state_entry
        self._state_feed: StateFeeder = state_feed
        self._module_config: ModuleEntry = module_config

        self._state_feed.feed(
            self._state_entry
        )

    @property
    def state_entry(self) -> StateEntry:
        """
        Returns the current state of the state model.

        :return: The current state entry.
        :rtype: StateEntry
        """
        return self._state_entry

    def base_enter(self, event):
        """
        On enter wrapper to be used for all on_enter methods.

        This wrapper updates the state entry and pushes it to the
        state feed.

        :param event: The event object.
        :type event: Any
        """
        try:
            self._state_entry.previous = self._state_entry.current
            self._state_entry.current = self.state
            self._state_entry.trigger = event.event.name
            self._state_entry.timestamp = datetime.now()
            
            Logger.debug("Sending state entry to state feed.")
            self._state_feed.feed(
                self._state_entry
            )
        finally:
            ErrorTraceback.check_error_exist()
    
    def on_enter_Resting(self, event):
        """
        On enter function for the Resting state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Resting state.")
    
    def on_enter_Awakening(self, event):
        """
        On enter function for the Awakening state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Awakening state.")

    def on_enter_Attentive(self, event):
        """
        On enter function for the Attentive state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Attentive state.")

    def on_enter_Active(self, event):
        """
        On enter function for the Active state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Active state.")

    def on_enter_Reflecting(self, event):
        """
        On enter function for the Reflecting state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Reflecting state.")

    def on_enter_Learning(self, event):
        """
        On enter function for the Learning state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Learning state.")

    def on_enter_Alert(self, event):
        """
        On enter function for the Alert state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Alert state.")

    def on_enter_Delegating(self, event):
        """
        On enter function for the Delegating state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Delegating state.")

    def on_enter_Recovering(self, event):
        """
        On enter function for the Recovering state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Recovering state.")

    def on_enter_Overloaded(self, event):
        """
        On enter function for the Overloaded state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Overloaded state.")

    def on_enter_ShuttingDown(self, event):
        """
        On enter function for the ShuttingDown state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering ShuttingDown state.")

    def on_enter_Interrupting(self, event):
        """
        On enter function for the Interrupting state.

        :param event: The event object.
        :type event: Any
        """
        self.base_enter(event)
        Logger.debug("Entering Interrupting state.")