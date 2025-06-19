from dataclasses import dataclass
from typing import NamedTuple
from typing import List, Union, NamedTuple
import json
import os

class Vyra_STATES:
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

class Vyra_TRANSITIONS:
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

state_config: dict = {
    "name": "vyra_base.state_machine",
    "states": [state for state in Vyra_STATES.__dict__.values() if isinstance(state, str)],
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