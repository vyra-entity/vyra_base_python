"""
Test Suite for 3-Layer State Machine - Events

Tests event types, StateEvent dataclass, and event routing.
"""

import pytest
from datetime import datetime
from vyra_base.state.state_events import (
    EventType,
    StateEvent,
    get_event_target_layer,
    is_interrupt_event,
    EVENT_LAYER_MAP,
)


class TestEventType:
    """Test EventType enum."""

    def test_lifecycle_events_exist(self):
        """Verify all lifecycle events are defined."""
        lifecycle_events = {
            EventType.START,
            EventType.INIT_SUCCESS,
            EventType.INIT_FAILURE,
            EventType.SHUTDOWN,
            EventType.FINISHED,
            EventType.FAULT_DETECTED,
            EventType.RECOVERY_SUCCESS,
            EventType.RECOVERY_FAILED,
        }
        for event in lifecycle_events:
            assert isinstance(event, EventType)

    def test_operational_events_exist(self):
        """Verify all operational events are defined."""
        operational_events = {
            EventType.SET_READY,
            EventType.TASK_START,
            EventType.TASK_PAUSE,
            EventType.TASK_RESUME,
            EventType.TASK_STOP,
            EventType.TASK_RESET,
        }
        for event in operational_events:
            assert isinstance(event, EventType)

    def test_health_events_exist(self):
        """Verify all health events are defined."""
        health_events = {
            EventType.WARN,
            EventType.CLEAR_WARNING,
            EventType.FAULT,
            EventType.RECOVER,
            EventType.RESET,
        }
        for event in health_events:
            assert isinstance(event, EventType)

    def test_interrupt_events_exist(self):
        """Verify interrupt events are defined."""
        interrupt_events = {
            EventType.EMERGENCY_STOP,
            EventType.INTERRUPT,
            EventType.PRIORITY_OVERRIDE,
        }
        for event in interrupt_events:
            assert isinstance(event, EventType)


class TestStateEvent:
    """Test StateEvent dataclass."""

    def test_create_minimal_event(self):
        """Test creating event with only event_type."""
        event = StateEvent(EventType.START)

        assert event.event_type == EventType.START
        assert isinstance(event.timestamp, datetime)
        assert event.origin_layer is None
        assert event.payload == {}
        assert event.event_id is not None

    def test_create_full_event(self):
        """Test creating event with all parameters."""
        payload = {"key": "value"}
        event = StateEvent(
            event_type=EventType.TASK_START,
            origin_layer="operational",
            payload=payload,
        )

        assert event.event_type == EventType.TASK_START
        assert event.origin_layer == "operational"
        assert event.payload == payload
        assert event.event_id is not None

    def test_event_id_generation(self):
        """Test automatic event ID generation."""
        event1 = StateEvent(EventType.START)
        event2 = StateEvent(EventType.START)

        # IDs should be unique
        assert event1.event_id != event2.event_id

        # IDs should contain event type value (lowercase)
        assert "start" in str(event1.event_id)
        assert "start" in str(event2.event_id)

    def test_custom_event_id(self):
        """Test custom event ID."""
        custom_id = "custom-event-123"
        event = StateEvent(EventType.START, event_id=custom_id)

        assert event.event_id == custom_id

    def test_event_str_representation(self):
        """Test string representation of event."""
        event = StateEvent(EventType.FAULT, origin_layer="health")

        string_repr = str(event)
        assert "fault" in string_repr
        assert "health" in string_repr
        assert str(event.event_id) in string_repr

    def test_event_to_dict(self):
        """Test event serialization to dictionary."""
        payload = {"error": "test error"}
        event = StateEvent(
            event_type=EventType.FAULT,
            origin_layer="health",
            payload=payload,
        )

        event_dict = event.to_dict()

        # event_type value is the enum's .value (lowercase string)
        assert event_dict["event_type"] == "fault"
        assert event_dict["origin_layer"] == "health"
        assert event_dict["payload"] == payload
        assert "timestamp" in event_dict
        assert "event_id" in event_dict

    def test_event_immutability(self):
        """Test that events are immutable (frozen dataclass)."""
        event = StateEvent(EventType.START)

        with pytest.raises(AttributeError):
            setattr(event, "event_type", EventType.SHUTDOWN)

    def test_payload_none_handling(self):
        """Test that None payload is converted to empty dict."""
        event = StateEvent(EventType.START, payload=None)
        assert event.payload == {}


class TestEventRouting:
    """Test event routing to layers."""

    def test_lifecycle_event_routing(self):
        """Test that lifecycle events route to lifecycle layer."""
        lifecycle_events = [
            EventType.START,
            EventType.INIT_SUCCESS,
            EventType.INIT_FAILURE,
            EventType.SHUTDOWN,
            EventType.FINISHED,
            EventType.RECOVERY_SUCCESS,
            EventType.RECOVERY_FAILED,
        ]

        for event_type in lifecycle_events:
            assert get_event_target_layer(event_type) == "lifecycle"

    def test_operational_event_routing(self):
        """Test that operational events route to operational layer."""
        operational_events = [
            EventType.SET_READY,
            EventType.TASK_START,
            EventType.TASK_PAUSE,
            EventType.TASK_RESUME,
            EventType.TASK_COMPLETE,
            EventType.TASK_STOP,
            EventType.TASK_RESET,
            EventType.TASK_ERROR,
        ]

        for event_type in operational_events:
            assert get_event_target_layer(event_type) == "operational"

    def test_health_event_routing(self):
        """Test that health events route to health layer."""
        health_events = [
            EventType.WARN,
            EventType.CLEAR_WARNING,
            EventType.FAULT,
            EventType.RECOVER,
            EventType.RESET,
        ]

        for event_type in health_events:
            assert get_event_target_layer(event_type) == "health"

    def test_interrupt_event_identification(self):
        """Test interrupt event identification."""
        assert is_interrupt_event(EventType.EMERGENCY_STOP)
        assert is_interrupt_event(EventType.INTERRUPT)
        assert is_interrupt_event(EventType.PRIORITY_OVERRIDE)

        assert not is_interrupt_event(EventType.START)
        assert not is_interrupt_event(EventType.TASK_START)
        assert not is_interrupt_event(EventType.WARN)

    def test_event_layer_map_completeness(self):
        """Verify EVENT_LAYER_MAP covers expected lifecycle/operational/health events."""
        # These events must all be present in the map
        required_events = [
            EventType.START, EventType.INIT_SUCCESS, EventType.INIT_FAILURE,
            EventType.SET_SUSPENDED, EventType.SHUTDOWN, EventType.FINISHED,
            EventType.FAULT_DETECTED, EventType.RECOVERY_SUCCESS, EventType.RECOVERY_FAILED,
            EventType.SET_READY, EventType.TASK_START, EventType.TASK_PAUSE,
            EventType.TASK_RESUME, EventType.TASK_COMPLETE, EventType.TASK_STOP,
            EventType.TASK_RESET, EventType.TASK_ERROR,
            EventType.WARN, EventType.CLEAR_WARNING, EventType.FAULT, EventType.RECOVER, EventType.RESET,
        ]
        for event_type in required_events:
            assert event_type in EVENT_LAYER_MAP, f"{event_type} missing from EVENT_LAYER_MAP"

    def test_event_layer_map_values(self):
        """Verify EVENT_LAYER_MAP values are valid layers."""
        valid_layers = {"lifecycle", "operational", "health", "interrupt"}

        for event_type, layer in EVENT_LAYER_MAP.items():
            assert layer in valid_layers, f"Invalid layer '{layer}' for {event_type}"


class TestEventCategorization:
    """Test event categorization and grouping."""

    def test_lifecycle_category(self):
        """Test lifecycle event category."""
        lifecycle_count = sum(
            1 for event_type in EVENT_LAYER_MAP
            if EVENT_LAYER_MAP[event_type] == "lifecycle"
        )
        assert lifecycle_count >= 7

    def test_operational_category(self):
        """Test operational event category."""
        operational_count = sum(
            1 for event_type in EVENT_LAYER_MAP
            if EVENT_LAYER_MAP[event_type] == "operational"
        )
        assert operational_count >= 7

    def test_health_category(self):
        """Test health event category."""
        health_count = sum(
            1 for event_type in EVENT_LAYER_MAP
            if EVENT_LAYER_MAP[event_type] == "health"
        )
        assert health_count >= 5


class TestEventPayloads:
    """Test event payload handling."""

    def test_empty_payload(self):
        """Test event with empty payload."""
        event = StateEvent(EventType.START)
        assert event.payload == {}

    def test_simple_payload(self):
        """Test event with simple payload."""
        payload = {"message": "test"}
        event = StateEvent(EventType.WARN, payload=payload)
        assert event.payload == payload

    def test_complex_payload(self):
        """Test event with complex nested payload."""
        payload = {
            "error": {
                "code": 500,
                "message": "Internal error",
                "details": {
                    "component": "sensor",
                    "timestamp": "2025-12-05T10:00:00",
                },
            },
            "metrics": [1, 2, 3],
        }
        event = StateEvent(EventType.FAULT, payload=payload)
        assert event.payload == payload
        assert event.payload["error"]["code"] == 500
        assert len(event.payload["metrics"]) == 3

    def test_payload_serialization(self):
        """Test that payload is properly serialized in to_dict."""
        payload = {"key": "value", "number": 42}
        event = StateEvent(EventType.START, payload=payload)

        event_dict = event.to_dict()
        assert event_dict["payload"] == payload


class TestEventTimestamps:
    """Test event timestamp handling."""

    def test_automatic_timestamp(self):
        """Test automatic timestamp generation."""
        before = datetime.now()
        event = StateEvent(EventType.START)
        after = datetime.now()

        assert before <= event.timestamp <= after

    def test_timestamp_serialization(self):
        """Test timestamp is serialized in ISO format."""
        event = StateEvent(EventType.START)
        event_dict = event.to_dict()

        timestamp_str = event_dict["timestamp"]
        assert "T" in timestamp_str
        datetime.fromisoformat(timestamp_str)
