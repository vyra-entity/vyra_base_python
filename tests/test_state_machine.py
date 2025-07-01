import pytest
from unittest.mock import MagicMock, patch
from datetime import datetime

from vyra_base.state.state_machine import StateMachine, StateModel
from vyra_base.state.state_config import Vyra_STATES, config_collection

# Dummy-Objekte für die Abhängigkeiten
class DummyStateFeeder:
    def __init__(self):
        self.feed = MagicMock()

class DummyModuleEntry:
    def __init__(self):
        self.uuid = "test-uuid"
        self.name = "test-module"

class DummyStateEntry:
    def __init__(self, previous, trigger, current, module_id, module_name, timestamp, _type):
        self.previous = previous
        self.trigger = trigger
        self.current = current
        self.module_id = module_id
        self.module_name = module_name
        self.timestamp = timestamp
        self._type = _type

@pytest.fixture
def dummy_state_feeder():
    return DummyStateFeeder()

@pytest.fixture
def dummy_module_entry():
    return DummyModuleEntry()

@pytest.fixture
def dummy_state_type():
    return "dummy_type"

@pytest.fixture
def state_machine(dummy_state_feeder, dummy_state_type, dummy_module_entry):
    # Patch StateEntry, config_collection, Vyra_STATES, Machine
    with patch("vyra_base.state.state_machine.StateEntry") as MockStateEntry, \
         patch("vyra_base.state.state_machine.config_collection", config_collection), \
         patch("vyra_base.state.state_machine.Vyra_STATES", Vyra_STATES), \
         patch("vyra_base.state.state_machine.Machine") as MockMachine:
        MockStateEntry.return_value = DummyStateEntry(
            previous="__CircleOfEternity__",
            trigger="__toBeBorn__",
            current=Vyra_STATES.Resting,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="dummy_type"
        )
        mock_machine_instance = MagicMock()
        mock_machine_instance.get_triggers.return_value = [
            t["trigger"] for t in config_collection["transitions"]
        ]
        MockMachine.return_value = mock_machine_instance

        sm = StateMachine(dummy_state_feeder, dummy_state_type, dummy_module_entry)
        sm.initialize()
        return sm

def test_initial_state(state_machine):
    assert state_machine.current_state == Vyra_STATES.Resting

def test_all_transitions(state_machine):
    transitions = state_machine.all_transitions
    assert isinstance(transitions, list)
    assert transitions[0]["trigger"] == config_collection["transitions"][0]["trigger"]
    assert transitions[0]["source"] == config_collection["transitions"][0]["source"]
    assert transitions[0]["dest"] == config_collection["transitions"][0]["dest"]

def test_is_transition_possible_true(state_machine):
    trigger = config_collection["transitions"][0]["trigger"]
    possible, triggers = state_machine.is_transition_possible(trigger)
    assert possible is True
    assert trigger in triggers

def test_is_transition_possible_false(state_machine):
    possible, triggers = state_machine.is_transition_possible("invalid_trigger")
    assert possible is False
    assert config_collection["transitions"][0]["trigger"] in triggers

def test_state_model_base_enter_updates_state(monkeypatch):
    with patch("vyra_base.state.state_machine.Logger"), \
         patch("vyra_base.state.state_machine.ErrorTraceback"):
        class DummyEvent:
            class Event:
                name = "ReadyForInput"
            event = Event()
        state_entry = DummyStateEntry(
            previous=Vyra_STATES.Awakening,
            trigger="StartUp",
            current=Vyra_STATES.Awakening,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="dummy_type"
        )
        state_feed = DummyStateFeeder()
        module_entry = DummyModuleEntry()
        model = StateModel(state_entry, state_feed, module_entry)
        model.state = Vyra_STATES.Attentive
        model.base_enter(DummyEvent())
        assert state_entry.previous == Vyra_STATES.Awakening
        assert state_entry.current == Vyra_STATES.Attentive
        assert state_entry.trigger == "ReadyForInput"
        state_feed.feed.assert_called_with(state_entry)

@pytest.mark.parametrize("state_method", [
    "on_enter_Resting", "on_enter_Awakening", "on_enter_Attentive", "on_enter_Active",
    "on_enter_Reflecting", "on_enter_Learning", "on_enter_Alert", "on_enter_Delegating",
    "on_enter_Recovering", "on_enter_Overloaded", "on_enter_ShuttingDown", "on_enter_Interrupting"
])
def test_state_model_on_enter_methods(state_method):
    with patch("vyra_base.state.state_machine.Logger"), \
         patch("vyra_base.state.state_machine.ErrorTraceback"):
        class DummyEvent:
            class Event:
                name = "dummy"
            event = Event()
        state_entry = DummyStateEntry(
            previous=Vyra_STATES.Resting,
            trigger="dummy",
            current=Vyra_STATES.Resting,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="dummy_type"
        )
        state_feed = DummyStateFeeder()
        module_entry = DummyModuleEntry()
        model = StateModel(state_entry, state_feed, module_entry)
        getattr(model, state_method)(DummyEvent())
        # Test besteht, wenn keine Exception geworfen wird

def test_state_model_state_entry_property():
    state_entry = DummyStateEntry(
        previous=Vyra_STATES.Resting,
        trigger="sleep",
        current=Vyra_STATES.Resting,
        module_id="test-uuid",
        module_name="test-module",
        timestamp=datetime.now(),
        _type="dummy_type"
    )
    state_feed = DummyStateFeeder()
    module_entry = DummyModuleEntry()
    model = StateModel(state_entry, state_feed, module_entry)
    assert model.state_entry == state_entry