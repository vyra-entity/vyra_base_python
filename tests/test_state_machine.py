import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from datetime import datetime

from vos_base.state.state_machine import StateMachine, StateModel

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
    # Patch VOS_STATES and state_config for initialization
    with patch("vos_base.state.state_machine.VOS_STATES") as VOS_STATES, \
         patch("vos_base.state.state_machine.state_config", {"states": ["resting"], "transitions": [], "initial": "resting"}):
        VOS_STATES.Resting = "resting"
        sm = StateMachine(mock_state_feeder, str, mock_module_entry)
        sm.initialize()
        return sm

def test_state_machine_initial_state(state_machine):
    assert state_machine.current_state == "resting"

def test_is_transition_possible(state_machine):
    # Keine Trigger definiert, sollte False sein
    assert not state_machine.is_transition_possible("any_trigger")

@pytest.mark.asyncio
async def test_state_model_enter_and_exit_states(mock_state_feeder, mock_module_entry):
    # Patch Logger.log, StateEntry und StateFeeder.feed
    with patch("vos_base.state.state_machine.Logger.log"), \
         patch("vos_base.state.state_machine.StateEntry") as MockStateEntry:
        state = MockStateEntry()
        state.current = "resting"
        state.previous = None
        state_feed = AsyncMock()
        model = StateModel(state, state_feed, mock_module_entry)

        await model.enter_resting()
        assert state.previous == "resting"
        assert state.current == "resting"
        state_feed.feed.assert_awaited_once_with(state)

        await model.exit_resting()
        # exit_state ruft Logger.log auf, prüfe, ob keine Exception geworfen wird
