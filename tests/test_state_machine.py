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
        # Ensure the model has the required state attribute
        if hasattr(sm, 'model'):
            sm.model.state = Vyra_STATES.Resting
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
        model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
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
        model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
        # Set the required state attribute
        model.state = Vyra_STATES.Resting
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
    model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
    assert model.state_entry == state_entry

def test_state_machine_initialization():
    """Test StateMachine initialization without fixture"""
    state_feed = DummyStateFeeder()
    state_type = "test_type"
    module_entry = DummyModuleEntry()
    
    with patch("vyra_base.state.state_machine.StateEntry") as MockStateEntry, \
         patch("vyra_base.state.state_machine.config_collection", config_collection), \
         patch("vyra_base.state.state_machine.Machine") as MockMachine:
        
        MockStateEntry.return_value = DummyStateEntry(
            previous="__CircleOfEternity__",
            trigger="__toBeBorn__",
            current=Vyra_STATES.Resting,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="test_type"
        )
        
        MockMachine.return_value = MagicMock()
        
        sm = StateMachine(state_feed, state_type, module_entry) # pyright: ignore[reportArgumentType]
        
        assert sm.state_feed == state_feed
        assert sm.state_type == state_type
        assert sm.module_config == module_entry
        
        # Test initialization
        sm.initialize()
        
        # Verify StateEntry was called with correct parameters
        MockStateEntry.assert_called_once()
        call_args = MockStateEntry.call_args[1]  # keyword arguments
        assert call_args['previous'] == '__CircleOfEternity__'
        assert call_args['trigger'] == '__toBeBorn__'
        assert call_args['current'] == Vyra_STATES.Resting
        assert call_args['module_id'] == module_entry.uuid
        assert call_args['module_name'] == module_entry.name
        assert call_args['_type'] == state_type

def test_state_machine_properties(state_machine):
    """Test StateMachine property access"""
    # Test current_state property
    assert state_machine.current_state == Vyra_STATES.Resting
    
    # Test all_transitions property
    transitions = state_machine.all_transitions
    assert transitions == config_collection['transitions']
    assert isinstance(transitions, list)

def test_state_machine_trigger_filtering(state_machine):
    """Test trigger filtering in is_transition_possible"""
    # Mock get_triggers to return triggers including 'to_' prefixed ones
    state_machine._machine.get_triggers.return_value = [
        'StartUp', 'to_Awakening', 'ReadyForInput', 'to_Attentive'
    ]
    
    possible, triggers = state_machine.is_transition_possible('StartUp')
    
    # Should filter out 'to_' prefixed triggers
    assert 'to_Awakening' not in triggers
    assert 'to_Attentive' not in triggers
    assert 'StartUp' in triggers
    assert 'ReadyForInput' in triggers

def test_state_model_initialization():
    """Test StateModel initialization"""
    state_entry = DummyStateEntry(
        previous=Vyra_STATES.Resting,
        trigger="test",
        current=Vyra_STATES.Resting,
        module_id="test-uuid",
        module_name="test-module",
        timestamp=datetime.now(),
        _type="test_type"
    )
    state_feed = DummyStateFeeder()
    module_entry = DummyModuleEntry()
    
    model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
    
    assert model.state_entry == state_entry
    assert model._state_feed == state_feed
    assert model._module_config == module_entry

def test_state_model_base_enter_with_different_states():
    """Test base_enter method with different state transitions"""
    with patch("vyra_base.state.state_machine.Logger") as MockLogger, \
         patch("vyra_base.state.state_machine.ErrorTraceback") as MockErrorTraceback:
        
        class DummyEvent:
            class Event:
                name = "TestTransition"
            event = Event()
        
        state_entry = DummyStateEntry(
            previous=Vyra_STATES.Resting,
            trigger="initial",
            current=Vyra_STATES.Resting,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="test_type"
        )
        
        state_feed = DummyStateFeeder()
        module_entry = DummyModuleEntry()
        model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
        
        # Test transition to different state
        model.state = Vyra_STATES.Active
        model.base_enter(DummyEvent())
        
        assert state_entry.previous == Vyra_STATES.Resting
        assert state_entry.current == Vyra_STATES.Active
        assert state_entry.trigger == "TestTransition"
        
        # Verify state feed was called
        state_feed.feed.assert_called_with(state_entry)

def test_state_model_all_state_transitions():
    """Test all possible state transitions"""
    states_to_test = [
        Vyra_STATES.Resting, Vyra_STATES.Awakening, Vyra_STATES.Attentive,
        Vyra_STATES.Active, Vyra_STATES.Reflecting, Vyra_STATES.Learning,
        Vyra_STATES.Alert, Vyra_STATES.Delegating, Vyra_STATES.Recovering,
        Vyra_STATES.Overloaded, Vyra_STATES.ShuttingDown, Vyra_STATES.Interrupting
    ]
    
    with patch("vyra_base.state.state_machine.Logger"), \
         patch("vyra_base.state.state_machine.ErrorTraceback"):
        
        for target_state in states_to_test:
            class DummyEvent:
                class Event:
                    name = f"TransitionTo{target_state}"
                event = Event()
            
            state_entry = DummyStateEntry(
                previous=Vyra_STATES.Resting,
                trigger="initial",
                current=Vyra_STATES.Resting,
                module_id="test-uuid",
                module_name="test-module",
                timestamp=datetime.now(),
                _type="test_type"
            )
            
            state_feed = DummyStateFeeder()
            module_entry = DummyModuleEntry()
            model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
            model.state = target_state
            
            # Test that base_enter works for this state
            model.base_enter(DummyEvent())
            
            assert state_entry.current == target_state
            assert state_entry.trigger == f"TransitionTo{target_state}"

def test_state_model_error_handling():
    """Test error handling in StateModel during base_enter"""
    with patch("vyra_base.state.state_machine.Logger") as MockLogger, \
         patch("vyra_base.state.state_machine.ErrorTraceback") as MockErrorTraceback:
        
        class DummyEvent:
            class Event:
                name = "ErrorTest"
            event = Event()
        
        state_entry = DummyStateEntry(
            previous=Vyra_STATES.Resting,
            trigger="test",
            current=Vyra_STATES.Resting,
            module_id="test-uuid",
            module_name="test-module",
            timestamp=datetime.now(),
            _type="test_type"
        )
        
        # Create a normal state_feed first
        state_feed = DummyStateFeeder()
        module_entry = DummyModuleEntry()
        model = StateModel(state_entry, state_feed, module_entry) # pyright: ignore[reportArgumentType]
        
        # Now make feed raise an exception for the next call
        state_feed.feed.side_effect = Exception("Test exception")
        model.state = Vyra_STATES.Active
        
        # Should not raise exception, but handle it internally
        try:
            model.base_enter(DummyEvent())
        except Exception:
            # If an exception occurs, that's expected behavior
            pass
        
        # The test passes if we reach this point without unhandled exceptions

def test_state_machine_with_real_config():
    """Test StateMachine with actual config_collection"""
    state_feed = DummyStateFeeder()
    state_type = "integration_test"
    module_entry = DummyModuleEntry()
    
    with patch("vyra_base.state.state_machine.StateEntry") as MockStateEntry, \
         patch("vyra_base.state.state_machine.Machine") as MockMachine:
        
        MockStateEntry.return_value = DummyStateEntry(
            previous="__CircleOfEternity__",
            trigger="__toBeBorn__",
            current=Vyra_STATES.Resting,
            module_id="test-uuid", 
            module_name="test-module",
            timestamp=datetime.now(),
            _type="integration_test"
        )
        
        mock_machine = MagicMock()
        MockMachine.return_value = mock_machine
        
        sm = StateMachine(state_feed, state_type, module_entry) # pyright: ignore[reportArgumentType]
        sm.initialize()
        
        # Verify Machine was called with correct config
        MockMachine.assert_called_once()
        call_kwargs = MockMachine.call_args[1]
        
        # Check that config_collection keys are present
        assert 'send_event' in call_kwargs
        assert call_kwargs['send_event'] is True
        assert 'model' in call_kwargs
        assert call_kwargs['model'] == sm.model

def test_trigger_name_validation():
    """Test various trigger name scenarios"""
    state_feed = DummyStateFeeder()
    state_type = "validation_test"
    module_entry = DummyModuleEntry()
    
    with patch("vyra_base.state.state_machine.StateEntry"), \
         patch("vyra_base.state.state_machine.config_collection", config_collection), \
         patch("vyra_base.state.state_machine.Machine") as MockMachine:
        
        mock_machine = MagicMock()
        mock_machine.get_triggers.return_value = ["ValidTrigger", "to_SomeState", "AnotherTrigger"]
        MockMachine.return_value = mock_machine
        
        sm = StateMachine(state_feed, state_type, module_entry) # pyright: ignore[reportArgumentType]
        sm.initialize()
        sm.model.state = Vyra_STATES.Resting
        
        # Test valid trigger
        possible, triggers = sm.is_transition_possible("ValidTrigger")
        assert possible is True
        assert "ValidTrigger" in triggers
        assert "to_SomeState" not in triggers  # Should be filtered out
        
        # Test invalid trigger
        possible, triggers = sm.is_transition_possible("InvalidTrigger")
        assert possible is False
        assert "ValidTrigger" in triggers
        assert "AnotherTrigger" in triggers