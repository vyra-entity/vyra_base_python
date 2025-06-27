import json
import os
from dataclasses import dataclass
from typing import List, Union, NamedTuple

class Vyra_STATES:
    """
    Collection of possible Vyra state names.

    :cvar str Resting: The resting state.
    :cvar str Awakening: The awakening state.
    :cvar str Attentive: The attentive state.
    :cvar str Active: The active state.
    :cvar str Reflecting: The reflecting state.
    :cvar str Learning: The learning state.
    :cvar str Alert: The alert state.
    :cvar str Delegating: The delegating state.
    :cvar str Recovering: The recovering state.
    :cvar str Overloaded: The overloaded state.
    :cvar str ShuttingDown: The shutting down state.
    :cvar str Interrupting: The interrupting state.
    """
    Resting: str = "Resting"
    Awakening: str = "Awakening"
    Attentive: str = "Attentive"
    Active: str = "Active"
    Reflecting: str = "Reflecting"
    Learning: str = "Learning"
    Alert: str = "Alert"
    Delegating: str = "Delegating"
    Recovering: str = "Recovering"
    Overloaded: str = "Overloaded"
    ShuttingDown: str = "ShuttingDown"
    Interrupting: str = "Interrupting"


@dataclass
class Transition:
    """
    Represents a state machine transition.

    :param str trigger: The event that triggers the transition.
    :param Union[str, List[str]] source: The source state(s).
    :param str dest: The destination state.
    """
    trigger: str
    source: Union[str, List[str]]
    dest: str


Vyra_TRANSITIONS_LIST: List[Transition] = [
    Transition(trigger="StartUp", source="Resting", dest="Awakening"),
    Transition(trigger="ReadyForInput", source="Awakening", dest="Attentive"),
    Transition(trigger="BeginWork", source="Attentive", dest="Active"),
    Transition(trigger="StartReflection", source="Active", dest="Reflecting"),
    Transition(trigger="StartLearning", source="Reflecting", dest="Learning"),
    Transition(trigger="FinishLearning", source="Learning", dest="Active"),
    Transition(trigger="EnterAlertMode", source="Attentive", dest="Alert"),
    Transition(trigger="DelegateTask", source="Attentive", dest="Delegating"),
    Transition(trigger="DelegationComplete", source="Delegating", dest="Attentive"),
    Transition(trigger="DelegateWhileActive", source="Active", dest="Delegating"),
    Transition(trigger="RecoveryComplete", source="Recovering", dest="Attentive"),
    Transition(trigger="StartRecovery", source="Overloaded", dest="Recovering"),
    Transition(trigger="ShutdownComplete", source="ShuttingDown", dest="Resting"),
    Transition(
        trigger="OverloadDetected",
        source=[
            "Resting", "Awakening", "Attentive", "Active", "Reflecting",
            "Learning", "Alert", "Delegating", "Recovering", "ShuttingDown"
        ],
        dest="Overloaded"
    ),
    Transition(
        trigger="ControlledShutdown",
        source=[
            "Resting", "Awakening", "Attentive", "Active", "Reflecting",
            "Learning", "Alert", "Delegating", "Recovering", "Overloaded"
        ],
        dest="ShuttingDown"
    ),
    Transition(
        trigger="InterruptEvent",
        source=[
            "Resting", "Awakening", "Attentive", "Active", "Reflecting",
            "Learning", "Alert", "Delegating", "Recovering", "Overloaded", "ShuttingDown"
        ],
        dest="Interrupting"
    ),
    Transition(trigger="EscalateToOverload", source="Interrupting", dest="Overloaded"),
    Transition(trigger="EscalateToAlert", source="Interrupting", dest="Alert"),
    Transition(trigger="EscalateToShutdown", source="Interrupting", dest="ShuttingDown"),
    Transition(trigger="ReturnAfterInterrupt", source="Interrupting", dest="prevState"),
]

config_collection: dict = {
    """
    Configuration dictionary for the Vyra state machine.

    :key str name: Name of the state machine.
    :key list states: List of possible states.
    :key list transitions: List of transition dictionaries.
    :key str initial: Initial state.
    """
    "name": "vyra_base.state_machine",
    "states": [
        v for k, v in Vyra_STATES.__dict__.items() 
        if not k.startswith('__')],
    "transitions": [
        {
            "trigger": t.trigger,
            "source": t.source,
            "dest": t.dest
        }
        for t in Vyra_TRANSITIONS_LIST
    ],
    "initial": Vyra_STATES.Resting
}