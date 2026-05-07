"""
Unit tests for ProtocolType enum and core type definitions.
"""
import json
from unittest.mock import AsyncMock, MagicMock
import pytest
from vyra_base.com.core.types import (
    ProtocolType, InterfaceType, CallbackType, AccessLevel, ActionStatus,
    DisplayStyle, InterfaceMetadata, GoalHandle,
    VyraPublisher, VyraSubscriber, VyraServer, VyraClient,
    VyraActionServer, VyraActionClient,
)
from vyra_base.com.core.topic_builder import TopicBuilder


class TestProtocolType:
    """Test protocol type enum."""
    
    def test_protocol_types_exist(self):
        """Test all protocol types are defined."""
        assert hasattr(ProtocolType, 'ROS2')
        assert hasattr(ProtocolType, 'SHARED_MEMORY')
        assert hasattr(ProtocolType, 'UDS')
        assert hasattr(ProtocolType, 'REDIS')
        assert hasattr(ProtocolType, 'GRPC')
        assert hasattr(ProtocolType, 'MQTT')
        assert hasattr(ProtocolType, 'REST')
        assert hasattr(ProtocolType, 'WEBSOCKET')
        assert hasattr(ProtocolType, 'MODBUS')
        assert hasattr(ProtocolType, 'OPCUA')
    
    def test_protocol_values(self):
        """Test protocol types have expected string values."""
        assert ProtocolType.ROS2 == "ros2"
        assert ProtocolType.SHARED_MEMORY == "sharedmemory"
        assert ProtocolType.UDS == "uds"
        assert ProtocolType.REDIS == "redis"
        assert ProtocolType.GRPC == "grpc"
        assert ProtocolType.MQTT == "mqtt"
        assert ProtocolType.REST == "rest"
        assert ProtocolType.WEBSOCKET == "websocket"
        assert ProtocolType.MODBUS == "modbus"
        assert ProtocolType.OPCUA == "opcua"
    
    def test_protocol_comparison(self):
        """Test protocol types can be compared."""
        assert ProtocolType.ROS2 == ProtocolType.ROS2
        assert ProtocolType.ROS2 != ProtocolType.REDIS
        assert ProtocolType.GRPC == "grpc"
    
    def test_protocol_in_list(self):
        """Test protocol types can be used in lists."""
        protocols = [ProtocolType.ROS2, ProtocolType.REDIS]
        assert ProtocolType.ROS2 in protocols
        assert ProtocolType.MQTT not in protocols


class TestInterfaceType:
    """Test interface type enum."""
    
    def test_interface_types_exist(self):
        """Test all interface types are defined."""
        assert hasattr(InterfaceType, 'SERVER')
        assert hasattr(InterfaceType, 'PUBLISHER')
        assert hasattr(InterfaceType, 'ACTION_SERVER')
    
    def test_interface_values(self):
        """Test interface types have expected string values."""
        assert InterfaceType.SERVER == "server"
        assert InterfaceType.PUBLISHER == "publisher"
        assert InterfaceType.ACTION_SERVER == "actionServer"
    
    def test_interface_comparison(self):
        """Test interface types can be compared."""
        assert InterfaceType.SERVER == InterfaceType.SERVER
        assert InterfaceType.SERVER != InterfaceType.PUBLISHER
        assert InterfaceType.ACTION_SERVER == "actionServer"


# ---------------------------------------------------------------------------
# CallbackType & AccessLevel & ActionStatus
# ---------------------------------------------------------------------------

class TestCallbackType:
    """Tests for CallbackType enum."""

    def test_all_values_exist(self):
        """All callback type members are accessible."""
        assert CallbackType.SUBSCRIBER == "subscriber_callback"
        assert CallbackType.SERVER == "response_callback"
        assert CallbackType.ACTION_SERVER_GOAL == "handle_goal_request"
        assert CallbackType.ACTION_SERVER_CANCEL == "handle_cancel_request"
        assert CallbackType.ACTION_SERVER_EXECUTION == "execution_callback"

    def test_action_client_types(self):
        """Action client callback types exist."""
        assert CallbackType.ACTION_CLIENT_DIRECT_RESPONSE == "direct_response_callback"
        assert CallbackType.ACTION_CLIENT_FEEDBACK == "feedback_callback"
        assert CallbackType.ACTION_CLIENT_GOAL == "goal_callback"


class TestAccessLevel:
    """Tests for AccessLevel enum."""

    def test_values(self):
        """All access levels have expected values."""
        assert AccessLevel.PUBLIC == "public"
        assert AccessLevel.PROTECTED == "protected"
        assert AccessLevel.PRIVATE == "private"
        assert AccessLevel.INTERNAL == "internal"


class TestActionStatus:
    """Tests for ActionStatus enum."""

    def test_status_values(self):
        """Action status integers are correct."""
        assert ActionStatus.UNKNOWN.value == 0
        assert ActionStatus.ACCEPTED.value == 1
        assert ActionStatus.EXECUTING.value == 2
        assert ActionStatus.CANCELING.value == 3
        assert ActionStatus.SUCCEEDED.value == 4
        assert ActionStatus.CANCELED.value == 5
        assert ActionStatus.ABORTED.value == 6


# ---------------------------------------------------------------------------
# DisplayStyle & InterfaceMetadata
# ---------------------------------------------------------------------------

class TestDisplayStyle:
    """Tests for DisplayStyle dataclass."""

    def test_defaults(self):
        """Default values are correct."""
        ds = DisplayStyle()
        assert ds.visible is True
        assert ds.icon == ""
        assert ds.color == ""
        assert ds.category == ""

    def test_custom_values(self):
        """Can set custom values."""
        ds = DisplayStyle(visible=False, icon="gear", color="red", category="system")
        assert ds.visible is False
        assert ds.icon == "gear"


class TestInterfaceMetadata:
    """Tests for InterfaceMetadata dataclass."""

    def _make(self, **kwargs):
        defaults = {
            "name": "test",
            "interface_type": InterfaceType.PUBLISHER,
            "protocol": ProtocolType.REDIS,
        }
        defaults.update(kwargs)
        return InterfaceMetadata(**defaults)

    def test_minimal_creation(self):
        """Creates with required fields only."""
        m = self._make()
        assert m.name == "test"
        assert m.protocol == ProtocolType.REDIS
        assert m.interface_type == InterfaceType.PUBLISHER
        assert m.access_level == AccessLevel.PUBLIC

    def test_to_dict_returns_dict(self):
        """to_dict returns a dict."""
        m = self._make()
        d = m.to_dict()
        assert isinstance(d, dict)
        assert d["name"] == "test"

    def test_to_json_returns_string(self):
        """to_json returns a JSON string."""
        m = self._make(description="hello")
        s = m.to_json()
        assert isinstance(s, str)
        parsed = json.loads(s)
        assert parsed["name"] == "test"
        assert parsed["description"] == "hello"

    def test_to_json_has_iso_datetime(self):
        """to_json converts datetime to ISO format string."""
        m = self._make()
        s = m.to_json()
        parsed = json.loads(s)
        # ISO format contains 'T' or '-'
        assert "T" in parsed["created_at"] or "-" in parsed["created_at"]

    def test_tags_default_empty(self):
        """tags defaults to empty list."""
        m = self._make()
        assert m.tags == []

    def test_custom_data_default_empty(self):
        """custom_data defaults to empty dict."""
        m = self._make()
        assert m.custom_data == {}


# ---------------------------------------------------------------------------
# GoalHandle
# ---------------------------------------------------------------------------

class TestGoalHandle:
    """Tests for GoalHandle concrete implementation."""

    def _make(self, feedback_fn=None):
        if feedback_fn is None:
            feedback_fn = AsyncMock()
        return GoalHandle(goal_id="gid-001", goal={"target": 100}, feedback_fn=feedback_fn)

    def test_initial_status_executing(self):
        """GoalHandle starts with 'executing' status."""
        gh = self._make()
        assert gh.status == "executing"

    def test_goal_property(self):
        """goal property returns the original goal payload."""
        gh = self._make()
        assert gh.goal == {"target": 100}

    def test_goal_id(self):
        """goal_id is accessible."""
        gh = self._make()
        assert gh.goal_id == "gid-001"

    def test_succeed_sets_status(self):
        """succeed() sets status to 'succeeded'."""
        gh = self._make()
        gh.succeed()
        assert gh.status == "succeeded"

    def test_abort_sets_status(self):
        """abort() sets status to 'aborted'."""
        gh = self._make()
        gh.abort()
        assert gh.status == "aborted"

    def test_canceled_sets_status(self):
        """canceled() sets status to 'canceled'."""
        gh = self._make()
        gh.canceled()
        assert gh.status == "canceled"

    def test_cancel_not_requested_initially(self):
        """Cancel is not requested initially."""
        gh = self._make()
        assert gh.is_cancel_requested() is False

    def test_request_cancel_sets_flag(self):
        """request_cancel() sets cancel flag."""
        gh = self._make()
        gh.request_cancel()
        assert gh.is_cancel_requested() is True

    @pytest.mark.asyncio
    async def test_publish_feedback_calls_fn(self):
        """publish_feedback calls the feedback_fn with goal_id and feedback."""
        fn = AsyncMock()
        gh = self._make(feedback_fn=fn)
        await gh.publish_feedback({"progress": 50})
        fn.assert_awaited_once_with("gid-001", {"progress": 50})

    def test_set_succeeded_alias(self):
        """set_succeeded is alias for succeed."""
        gh = self._make()
        gh.set_succeeded()
        assert gh.status == "succeeded"

    def test_set_aborted_alias(self):
        """set_aborted is alias for abort."""
        gh = self._make()
        gh.set_aborted("error")
        assert gh.status == "aborted"

    def test_set_canceled_alias(self):
        """set_canceled is alias for canceled."""
        gh = self._make()
        gh.set_canceled()
        assert gh.status == "canceled"


# ---------------------------------------------------------------------------
# VyraPublisher, VyraSubscriber, VyraServer, VyraClient
# ---------------------------------------------------------------------------

def _topic_builder():
    """Return a minimal TopicBuilder for tests."""
    return TopicBuilder(module_name="test_mod", module_id="test_module")


class TestVyraPublisher:
    """Tests for VyraPublisher transport class."""

    def _make(self, **kwargs):
        return VyraPublisher(
            name="pub1",
            topic_builder=_topic_builder(),
            protocol=ProtocolType.REDIS,
            **kwargs
        )

    def test_creation(self):
        """Creates without error."""
        p = self._make()
        assert p.name == "pub1"
        assert p.protocol == ProtocolType.REDIS

    def test_not_initialized_initially(self):
        """is_initialized returns False before initialize()."""
        p = self._make()
        assert p.is_initialized() is False

    @pytest.mark.asyncio
    async def test_initialize_sets_flag(self):
        """initialize() sets _initialized to True."""
        p = self._make()
        result = await p.initialize()
        assert result is True
        assert p.is_initialized() is True

    @pytest.mark.asyncio
    async def test_shutdown_clears_flag(self):
        """shutdown() clears initialized flag."""
        p = self._make()
        await p.initialize()
        await p.shutdown()
        assert p.is_initialized() is False

    @pytest.mark.asyncio
    async def test_publish_not_initialized_raises(self):
        """publish raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        p = self._make()
        with pytest.raises(InterfaceError):
            await p.publish("msg")

    @pytest.mark.asyncio
    async def test_publish_not_implemented_when_initialized(self):
        """publish raises NotImplementedError on base class after init."""
        p = self._make()
        await p.initialize()
        with pytest.raises(NotImplementedError):
            await p.publish("msg")

    def test_missing_topic_builder_raises(self):
        """Raises InterfaceError if topic_builder is None."""
        from vyra_base.com.core.exceptions import InterfaceError
        with pytest.raises(InterfaceError):
            VyraPublisher(name="pub", topic_builder=None)


class TestVyraSubscriber:
    """Tests for VyraSubscriber transport class."""

    def _make(self, cb=None):
        return VyraSubscriber(
            name="sub1",
            topic_builder=_topic_builder(),
            subscriber_callback=cb,
            protocol=ProtocolType.REDIS,
        )

    def test_creation_with_callback(self):
        """Creates with callback assigned."""
        cb = MagicMock()
        s = self._make(cb=cb)
        assert s.subscriber_callback is cb

    @pytest.mark.asyncio
    async def test_initialize(self):
        """initialize returns True."""
        s = self._make()
        assert await s.initialize() is True

    @pytest.mark.asyncio
    async def test_subscribe_not_initialized_raises(self):
        """subscribe raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        s = self._make()
        with pytest.raises(InterfaceError):
            await s.subscribe()

    @pytest.mark.asyncio
    async def test_subscribe_raises_not_implemented(self):
        """subscribe raises NotImplementedError on base class."""
        s = self._make()
        await s.initialize()
        with pytest.raises(NotImplementedError):
            await s.subscribe()


class TestVyraServer:
    """Tests for VyraServer transport class."""

    def _make(self):
        return VyraServer(
            name="srv1",
            topic_builder=_topic_builder(),
            protocol=ProtocolType.REDIS,
        )

    @pytest.mark.asyncio
    async def test_initialize(self):
        """initialize returns True."""
        s = self._make()
        assert await s.initialize() is True

    @pytest.mark.asyncio
    async def test_serve_not_initialized_raises(self):
        """serve raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        s = self._make()
        with pytest.raises(InterfaceError):
            await s.serve()

    @pytest.mark.asyncio
    async def test_serve_raises_not_implemented(self):
        """serve raises NotImplementedError on base class."""
        s = self._make()
        await s.initialize()
        with pytest.raises(NotImplementedError):
            await s.serve()


class TestVyraClient:
    """Tests for VyraClient transport class."""

    def _make(self):
        return VyraClient(
            name="cli1",
            topic_builder=_topic_builder(),
            protocol=ProtocolType.REDIS,
        )

    @pytest.mark.asyncio
    async def test_initialize(self):
        """initialize returns True."""
        c = self._make()
        assert await c.initialize() is True

    @pytest.mark.asyncio
    async def test_call_not_initialized_raises(self):
        """call raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        c = self._make()
        with pytest.raises(InterfaceError):
            await c.call({"x": 1})

    @pytest.mark.asyncio
    async def test_call_raises_not_implemented(self):
        """call raises NotImplementedError on base class."""
        c = self._make()
        await c.initialize()
        with pytest.raises(NotImplementedError):
            await c.call({"x": 1})


class TestVyraActionServer:
    """Tests for VyraActionServer transport class."""

    def _make(self):
        return VyraActionServer(
            name="act_srv",
            topic_builder=_topic_builder(),
            protocol=ProtocolType.REDIS,
        )

    @pytest.mark.asyncio
    async def test_initialize(self):
        """initialize returns True."""
        s = self._make()
        assert await s.initialize() is True

    @pytest.mark.asyncio
    async def test_start_not_initialized_raises(self):
        """start raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        s = self._make()
        with pytest.raises(InterfaceError):
            await s.start()

    @pytest.mark.asyncio
    async def test_start_raises_not_implemented(self):
        """start raises NotImplementedError on base class."""
        s = self._make()
        await s.initialize()
        with pytest.raises(NotImplementedError):
            await s.start()

    @pytest.mark.asyncio
    async def test_shutdown(self):
        """shutdown clears initialized flag."""
        s = self._make()
        await s.initialize()
        await s.shutdown()
        assert s.is_initialized() is False

    def test_action_channel_no_subsection(self):
        """_action_channel builds channel without subsection."""
        s = self._make()
        channel = s._action_channel("goal")
        assert "goal" in channel

    def test_action_channel_with_subsection(self):
        """_action_channel prepends subsection."""
        s = VyraActionServer(
            name="act_srv",
            topic_builder=_topic_builder(),
            subsection="tasks",
        )
        channel = s._action_channel("goal")
        assert "tasks" in channel
        assert "goal" in channel

    def test_missing_topic_builder_raises(self):
        """Raises InterfaceError if topic_builder is None."""
        from vyra_base.com.core.exceptions import InterfaceError
        with pytest.raises(InterfaceError):
            VyraActionServer(name="x", topic_builder=None)


class TestVyraActionClient:
    """Tests for VyraActionClient transport class."""

    def _make(self):
        return VyraActionClient(
            name="act_cli",
            topic_builder=_topic_builder(),
            protocol=ProtocolType.REDIS,
        )

    @pytest.mark.asyncio
    async def test_initialize(self):
        """initialize returns True."""
        c = self._make()
        assert await c.initialize() is True

    @pytest.mark.asyncio
    async def test_send_goal_not_initialized_raises(self):
        """send_goal raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        c = self._make()
        with pytest.raises(InterfaceError):
            await c.send_goal({"goal": 1})

    @pytest.mark.asyncio
    async def test_send_goal_raises_not_implemented(self):
        """send_goal raises NotImplementedError on base class."""
        c = self._make()
        await c.initialize()
        with pytest.raises(NotImplementedError):
            await c.send_goal({"goal": 1})

    @pytest.mark.asyncio
    async def test_cancel_goal_not_initialized_raises(self):
        """cancel_goal raises InterfaceError if not initialized."""
        from vyra_base.com.core.exceptions import InterfaceError
        c = self._make()
        with pytest.raises(InterfaceError):
            await c.cancel_goal(MagicMock())

    @pytest.mark.asyncio
    async def test_cancel_goal_raises_not_implemented(self):
        """cancel_goal raises NotImplementedError on base class."""
        c = self._make()
        await c.initialize()
        with pytest.raises(NotImplementedError):
            await c.cancel_goal(MagicMock())

    def test_action_channel_no_subsection(self):
        """_action_channel builds channel without subsection."""
        c = self._make()
        channel = c._action_channel("result")
        assert "result" in channel

    def test_missing_topic_builder_raises(self):
        """Raises InterfaceError if topic_builder is None."""
        from vyra_base.com.core.exceptions import InterfaceError
        with pytest.raises(InterfaceError):
            VyraActionClient(name="x", topic_builder=None)


class TestVyraTransportKwargs:
    """Tests for VyraTransport __init__ kwargs handling."""

    def test_namespace_extracted(self):
        """namespace kwarg is stored as attribute."""
        p = VyraPublisher(
            name="p",
            topic_builder=_topic_builder(),
            namespace="my_ns",
        )
        assert p.namespace == "my_ns"

    def test_subsection_extracted(self):
        """subsection kwarg is stored as attribute."""
        p = VyraPublisher(
            name="p",
            topic_builder=_topic_builder(),
            subsection="sub",
        )
        assert p.subsection == "sub"

    def test_extra_kwargs_go_to_custom_data(self):
        """Unknown kwargs are stored in metadata.custom_data."""
        p = VyraPublisher(
            name="p",
            topic_builder=_topic_builder(),
            my_custom_field="hello",
        )
        assert p.metadata.custom_data.get("my_custom_field") == "hello"

    def test_description_kwarg(self):
        """description kwarg maps to metadata.description."""
        p = VyraPublisher(
            name="p",
            topic_builder=_topic_builder(),
            description="test publisher",
        )
        assert p.metadata.description == "test publisher"

    def test_is_connected_delegates_to_initialized(self):
        """is_connected returns same as is_initialized."""
        p = VyraPublisher(name="p", topic_builder=_topic_builder())
        assert p.is_connected() is False
