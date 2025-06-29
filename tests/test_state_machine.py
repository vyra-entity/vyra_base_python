import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from datetime import datetime

from vyra_base.state.state_machine import StateMachine, StateModel

@pytest.fixture
def mock_state_feeder():
    return MagicMock()

@pytest.fixture
def mock_module_entry():
    mock = MagicMock()
    mock.uuid = "test-uuid"
    mock.name = "test-module"
    return mock

@pytest.fixture
def mock_state_entry():
    mock = MagicMock()
    mock.current = "resting"
    mock.previous = None
    return mock

@pytest.fixture
def state_machine(mock_state_feeder, mock_module_entry):
    # Patch Vyra_STATES and config_collection for initialization
    with patch("vyra_base.state.state_machine.Vyra_STATES") as vyra_STATES, \
         patch("vyra_base.state.state_machine.config_collection", {"states": ["resting"], "transitions": [], "initial": "resting"}):
        vyra_STATES.Resting = "resting"
        sm = StateMachine(mock_state_feeder, str, mock_module_entry)
        sm.initialize()
        return sm

def test_state_machine_initial_state(state_machine):
    assert state_machine.current_state == "resting"

def test_is_transition_possible(state_machine):
    # Keine Trigger definiert, sollte False sein
    # assert is [False, []]
    possible, triggers = state_machine.is_transition_possible("any_trigger")
    assert not possible
    assert triggers == []

@pytest.mark.asyncio
async def test_state_model_enter_and_exit_states(mock_state_feeder, mock_module_entry):
    # Patch Logger.log, StateEntry und StateFeeder.feed
    with patch("vyra_base.state.state_machine.Logger.log"), \
         patch("vyra_base.state.state_machine.StateEntry") as MockStateEntry:
        state = MockStateEntry()
        state.current = "resting"
        state.previous = None
        state_feed = AsyncMock()
        model = StateModel(state, state_feed, mock_module_entry)

        await model.StartUp()
        assert state.previous == "Resting"
        assert state.current == "Awakening"
        state_feed.feed.assert_awaited_once_with(state)

        await model.exit_resting()
        # exit_state ruft Logger.log auf, prüfe, ob keine Exception geworfen wird
