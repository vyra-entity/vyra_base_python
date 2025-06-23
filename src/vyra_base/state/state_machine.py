from __future__ import annotations

import json
import os

from ast import Module
from datetime import datetime
from transitions import Machine
from collections import namedtuple
from typing import Union
from pathlib import Path
from typing import Any
from vyra_base.helper.logger import LogEntry
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.helper.logger import LogMode as LogMode

from vyra_base.helper.file_reader import FileReader
from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.defaults.entries import StateEntry
from vyra_base.helper.logger import Logger

from .state_config import Vyra_STATES
from .state_config import config_collection


class StateMachine:
    """ Initializes the global state machine for the module."""

    __slots__ = (
        'state_feed', 
        'state_type',
        '_machine', 
        'model', 
        '_state',
        'module_config'
    )
    
    def __init__(self, state_feed: StateFeeder, state_type: Any, module_config: ModuleEntry, ):
        self.state_feed: StateFeeder = state_feed
        self.state_type: Any = state_type
        self._machine: Machine
        self.model: StateModel
        self.module_config: ModuleEntry = module_config

    def initialize(self):
        self._machine: Machine = Machine(**config_collection)  # **modelConfig unpacks arguments as kwargs

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

        self.generate_state_methods(
            config_collection['states']
        )

        config_collection['model'] = self.model  # adding a model to the configuration

    @property
    def current_state(self) -> str:
        """ Returns the current state of the state machine. """
        return self._state.current

    def is_transition_possible(self, trigger_name):
        return trigger_name in self._machine.get_triggers(self._state.current)

    def generate_state_methods(self, states: list[str]):
        """Generates methods for entering states dynamically."""
        for state in states:
            async def on_enter(self, state=state):
                pass
                # print(f"{self.name}: entering state '{state}'")
                # Logger.log(
                #     LogEntry(f'Enter {self.name} state.', LogMode.INFO)
                # )
                # Logger.log("" + str(self._state))
                # self._state.previous = self._state.current
                # self._state.current = self.name
                # await self.state_feed.feed(self.state)

            setattr(
                self.model, 
                f'on_enter_{state}', 
                on_enter.__get__(
                    self, 
                    StateMachine
                )
            )

class StateModel:
    """ State class for all states.
    """

    __slots__ = (
        'state', 
        'state_feed', 
        'module_config'
    )

    def __init__(
            self, 
            state: StateEntry, 
            state_feed: StateFeeder, 
            module_config: ModuleEntry):
        
        Logger.log(f'Initialize state model with state {state.current}.')

        self.state: StateEntry = state
        self.state_feed: StateFeeder = state_feed
        self.module_config: ModuleEntry = module_config


    def aa(self):
        pass

    # Enter methods for all states
    async def enter_state(self, state_name: str):
        """Generic method to enter a state."""
        Logger.log(
            LogEntry(f'Enter {state_name} state.', LogMode.INFO)
        )
        Logger.log("" + str(self.state))
        self.state.previous = self.state.current
        self.state.current = state_name
        await self.state_feed.feed(self.state)

    # async def enter_resting(self):
    #     await self._enter_state('resting')

    # async def enter_awakening(self):
    #     await self._enter_state('awakening')

    # async def enter_attentive(self):
    #     await self._enter_state('attentive')

    # async def enter_active(self):
    #     await self._enter_state('active')

    # async def enter_reflecting(self):
    #     await self._enter_state('reflecting')

    # async def enter_learning(self):
    #     await self._enter_state('learning')

    # async def enter_alert(self):
    #     await self._enter_state('alert')

    # async def enter_delegating(self):
    #     await self._enter_state('delegating')

    # async def enter_recovering(self):
    #     await self._enter_state('recovering')

    # async def enter_overloaded(self):
    #     await self._enter_state('overloaded')

    # async def enter_shuttingdown(self):
    #     await self._enter_state('shuttingdown')

    # async def enter_interrupting(self):
    #     await self._enter_state('interrupting')

    # async def _enter_state(self, state_name: str):
    #     """Hilfsmethode für alle enter_<state>-Funktionen."""
    #     Logger.log(
    #         LogEntry(f'Enter {state_name} state.', LogMode.INFO)
    #     )
    #     Logger.log("" + str(self.state))
    #     self.state.previous = self.state.current
    #     self.state.current = state_name
    #     await self.state_feed.feed(
    #         self.state
    #     )

    # # Exit methods for all states

    async def exit_resting(self):
        await self._exit_state('resting')

    async def exit_awakening(self):
        await self._exit_state('awakening')

    async def exit_attentive(self):
        await self._exit_state('attentive')

    async def exit_active(self):
        await self._exit_state('active')

    async def exit_reflecting(self):
        await self._exit_state('reflecting')

    async def exit_learning(self):
        await self._exit_state('learning')

    async def exit_alert(self):
        await self._exit_state('alert')

    async def exit_delegating(self):
        await self._exit_state('delegating')

    async def exit_recovering(self):
        await self._exit_state('recovering')

    async def exit_overloaded(self):
        await self._exit_state('overloaded')

    async def exit_shuttingdown(self):
        await self._exit_state('shuttingdown')

    async def exit_interrupting(self):
        await self._exit_state('interrupting')

    async def _exit_state(self, state_name: str):
        Logger.log(
            LogEntry(f'Exit {state_name} state.', LogMode.INFO)
        )
      
# EOF: state_machine.py