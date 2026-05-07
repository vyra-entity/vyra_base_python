"""
Tests for vyra_base.com.core.blueprints
"""
import pytest

from vyra_base.com.core.blueprints import (
    HandlerBlueprint,
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    InterfaceType,
)
from vyra_base.com.core.types import ProtocolType


# ── ServiceBlueprint tests ────────────────────────────────────────────────────

class TestServiceBlueprint:
    def test_create_defaults(self):
        bp = ServiceBlueprint(name="my_service")
        assert bp.name == "my_service"
        assert bp.interface_type == InterfaceType.SERVICE
        assert bp.protocols == []
        assert bp.metadata == {}
        assert not bp.is_bound()

    def test_create_with_protocols(self):
        bp = ServiceBlueprint(name="svc", protocols=[ProtocolType.ROS2])
        assert ProtocolType.ROS2 in bp.protocols

    def test_create_with_metadata(self):
        bp = ServiceBlueprint(name="svc", metadata={"qos": 10})
        assert bp.get_metadata("qos") == 10

    def test_create_with_service_type(self):
        bp = ServiceBlueprint(name="svc", service_type="MyServiceType")
        assert bp.service_type == "MyServiceType"

    def test_bind_valid_callback(self):
        bp = ServiceBlueprint(name="svc")

        async def handler(request, response=None):
            pass

        bp.bind_callback(handler)
        assert bp.is_bound()
        assert bp.callback is handler

    def test_bind_no_param_callback_raises(self):
        bp = ServiceBlueprint(name="svc")

        async def bad_handler():
            pass

        with pytest.raises(ValueError, match="at least 1 parameter"):
            bp.bind_callback(bad_handler)

    def test_bind_twice_raises(self):
        bp = ServiceBlueprint(name="svc")

        async def handler(request):
            pass

        bp.bind_callback(handler)
        with pytest.raises(RuntimeError, match="already has a bound callback"):
            bp.bind_callback(handler)

    def test_unbind_callback(self):
        bp = ServiceBlueprint(name="svc")

        async def handler(request):
            pass

        bp.bind_callback(handler)
        old = bp.unbind_callback()
        assert old is handler
        assert not bp.is_bound()

    def test_unbind_when_not_bound_returns_none(self):
        bp = ServiceBlueprint(name="svc")
        result = bp.unbind_callback()
        assert result is None

    def test_get_metadata_default(self):
        bp = ServiceBlueprint(name="svc")
        assert bp.get_metadata("missing_key", "default") == "default"

    def test_update_metadata(self):
        bp = ServiceBlueprint(name="svc", metadata={"a": 1})
        bp.update_metadata(b=2)
        assert bp.get_metadata("b") == 2
        assert bp.get_metadata("a") == 1

    def test_repr(self):
        bp = ServiceBlueprint(name="svc", protocols=[ProtocolType.ROS2])
        r = repr(bp)
        assert "ServiceBlueprint" in r
        assert "svc" in r

    def test_callback_with_unconventional_first_param_logs_warning(self, caplog):
        bp = ServiceBlueprint(name="svc")

        import logging
        with caplog.at_level(logging.WARNING):
            async def handler(data):
                pass
            bp.bind_callback(handler)

        # Should have bound successfully
        assert bp.is_bound()

    def test_bind_with_self_param_bound_method(self):
        """Callbacks with 'self' as first param should be allowed."""
        bp = ServiceBlueprint(name="svc")

        async def handler(self, request):
            pass

        bp.bind_callback(handler)
        assert bp.is_bound()


# ── PublisherBlueprint tests ──────────────────────────────────────────────────

class TestPublisherBlueprint:
    def test_create_defaults(self):
        bp = PublisherBlueprint(name="my_topic")
        assert bp.name == "my_topic"
        assert bp.interface_type == InterfaceType.PUBLISHER

    def test_create_with_message_type(self):
        bp = PublisherBlueprint(name="topic", message_type="MyMsgType")
        assert bp.message_type == "MyMsgType"

    def test_bind_valid_callback(self):
        bp = PublisherBlueprint(name="topic")

        async def publish(message):
            pass

        bp.bind_callback(publish)
        assert bp.is_bound()

    def test_bind_no_param_callback_raises(self):
        bp = PublisherBlueprint(name="topic")

        async def no_param():
            pass

        with pytest.raises(ValueError, match="at least 1 parameter"):
            bp.bind_callback(no_param)

    def test_bind_with_self_param_allowed(self):
        bp = PublisherBlueprint(name="topic")

        async def publish(self, message):
            pass

        bp.bind_callback(publish)
        assert bp.is_bound()


# ── SubscriberBlueprint tests ─────────────────────────────────────────────────

class TestSubscriberBlueprint:
    def test_create_defaults(self):
        bp = SubscriberBlueprint(name="sub_topic")
        assert bp.name == "sub_topic"
        assert bp.interface_type == InterfaceType.SUBSCRIBER

    def test_create_with_message_type(self):
        bp = SubscriberBlueprint(name="topic", message_type="SensorData")
        assert bp.message_type == "SensorData"

    def test_bind_valid_callback(self):
        bp = SubscriberBlueprint(name="topic")

        async def on_message(message):
            pass

        bp.bind_callback(on_message)
        assert bp.is_bound()

    def test_bind_no_param_callback_raises(self):
        bp = SubscriberBlueprint(name="topic")

        async def bad():
            pass

        with pytest.raises(ValueError, match="at least 1 parameter"):
            bp.bind_callback(bad)

    def test_bind_with_self_param_allowed(self):
        bp = SubscriberBlueprint(name="topic")

        async def on_message(self, message):
            pass

        bp.bind_callback(on_message)
        assert bp.is_bound()


# ── ActionBlueprint tests ─────────────────────────────────────────────────────

class TestActionBlueprint:
    def test_create_defaults(self):
        bp = ActionBlueprint(name="my_action")
        assert bp.name == "my_action"
        assert bp.interface_type == InterfaceType.ACTION
        assert not bp.is_bound()
        assert not bp.is_fully_bound()

    def test_create_with_action_type(self):
        bp = ActionBlueprint(name="action", action_type="MyAction")
        assert bp.action_type == "MyAction"

    def test_callback_property_returns_execute(self):
        bp = ActionBlueprint(name="action")

        async def execute(goal_handle):
            pass

        bp.bind_callback(execute, "execute")
        assert bp.callback is execute

    def test_bind_execute_callback(self):
        bp = ActionBlueprint(name="action")

        async def execute(goal_handle):
            pass

        bp.bind_callback(execute)  # default is 'execute'
        assert bp.is_bound()
        assert bp.is_bound("execute")

    def test_bind_on_goal_callback(self):
        bp = ActionBlueprint(name="action")

        async def on_goal(goal_request):
            return True

        bp.bind_callback(on_goal, "on_goal")
        assert bp.is_bound("on_goal")
        assert bp.get_callback("on_goal") is on_goal

    def test_bind_on_cancel_callback(self):
        bp = ActionBlueprint(name="action")

        async def on_cancel(goal_handle):
            return True

        bp.bind_callback(on_cancel, "on_cancel")
        assert bp.is_bound("on_cancel")

    def test_bind_invalid_type_raises(self):
        bp = ActionBlueprint(name="action")

        async def cb(x):
            pass

        with pytest.raises(ValueError, match="Invalid callback_type"):
            bp.bind_callback(cb, "invalid_type")

    def test_bind_twice_same_type_raises(self):
        bp = ActionBlueprint(name="action")

        async def execute(goal_handle):
            pass

        bp.bind_callback(execute)
        with pytest.raises(RuntimeError, match="already bound"):
            bp.bind_callback(execute)

    def test_bind_callback_no_params_raises(self):
        bp = ActionBlueprint(name="action")

        async def bad():
            pass

        with pytest.raises(ValueError, match="at least 1 parameter"):
            bp.bind_callback(bad)

    def test_bind_callbacks_multi(self):
        bp = ActionBlueprint(name="action")

        async def on_goal(goal_request): return True
        async def on_cancel(gh): return True
        async def execute(gh): pass

        bp.bind_callbacks(on_goal=on_goal, on_cancel=on_cancel, execute=execute)
        assert bp.is_fully_bound()

    def test_bind_callbacks_with_none_skips(self):
        bp = ActionBlueprint(name="action")

        async def execute(gh): pass

        bp.bind_callbacks(execute=execute, on_goal=None)
        assert bp.is_bound("execute")
        assert not bp.is_bound("on_goal")

    def test_unbind_callback(self):
        bp = ActionBlueprint(name="action")

        async def execute(gh): pass

        bp.bind_callback(execute)
        old = bp.unbind_callback("execute")
        assert old is execute
        assert not bp.is_bound("execute")

    def test_unbind_unbound_returns_none(self):
        bp = ActionBlueprint(name="action")
        result = bp.unbind_callback("on_goal")
        assert result is None

    def test_is_fully_bound_false_when_partial(self):
        bp = ActionBlueprint(name="action")

        async def execute(gh): pass

        bp.bind_callback(execute)
        assert not bp.is_fully_bound()

    def test_repr(self):
        bp = ActionBlueprint(name="action")
        r = repr(bp)
        assert "ActionBlueprint" in r
        assert "action" in r

    def test_get_metadata(self):
        bp = ActionBlueprint(name="action", metadata={"timeout": 30})
        assert bp.get_metadata("timeout") == 30

    def test_update_metadata(self):
        bp = ActionBlueprint(name="action")
        bp.update_metadata(timeout=60, priority="high")
        assert bp.get_metadata("timeout") == 60
        assert bp.get_metadata("priority") == "high"

    def test_bind_with_self_param(self):
        bp = ActionBlueprint(name="action")

        async def execute(self, goal_handle):
            pass

        bp.bind_callback(execute)
        assert bp.is_bound()


# ── InterfaceType enum tests ──────────────────────────────────────────────────

class TestInterfaceType:
    def test_values_exist(self):
        assert InterfaceType.SERVICE.value == "service"
        assert InterfaceType.PUBLISHER.value == "publisher"
        assert InterfaceType.SUBSCRIBER.value == "subscriber"
        assert InterfaceType.ACTION.value == "action"


# ── HandlerBlueprint repr with long protocols list ────────────────────────────

class TestHandlerBlueprintRepr:
    def test_repr_with_many_protocols(self):
        bp = ServiceBlueprint(
            name="svc",
            protocols=[ProtocolType.ROS2, ProtocolType.REDIS, ProtocolType.MQTT]
        )
        r = repr(bp)
        assert "..." in r  # truncated

    def test_repr_with_two_protocols(self):
        bp = ServiceBlueprint(
            name="svc",
            protocols=[ProtocolType.ROS2, ProtocolType.REDIS]
        )
        r = repr(bp)
        assert "..." not in r
