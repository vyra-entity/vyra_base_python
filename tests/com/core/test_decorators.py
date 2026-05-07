"""
Unit tests for decorators (@remote_service, @remote_publisher, @remote_actionServer, @remote_subscriber).
"""
import pytest
import asyncio
from vyra_base.com.core.decorators import (
    remote_service,
    remote_publisher,
    remote_actionServer,
    remote_subscriber,
    get_decorated_methods,
    bind_decorated_callbacks,
    ActionServerDecorator,
)
from vyra_base.com.core.callback_registry import CallbackRegistry
from vyra_base.com.core.blueprints import (
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
)
from vyra_base.com.core.types import ProtocolType


@pytest.fixture(autouse=True)
def clear_registry():
    """Save registry state before test, restore after to avoid cross-test pollution."""
    saved = dict(CallbackRegistry._blueprints)
    CallbackRegistry.clear()
    yield
    CallbackRegistry.clear()
    CallbackRegistry._blueprints.update(saved)


class TestRemoteCallableDecorator:
    """Test @remote_service decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_service(name="test_callable")
        async def test_func(request):
            return {"result": "success"}
        
        assert hasattr(test_func, '_vyra_remote_server')
        assert test_func._vyra_remote_server is True
        assert test_func._vyra_server_name == "test_callable"
    
    def test_decorator_on_sync_function(self):
        """Test decorator converts sync to async."""
        @remote_service(name="sync_callable")
        def sync_func(request):
            return {"result": "sync"}
        
        # Should be converted to async
        assert asyncio.iscoroutinefunction(sync_func)
        assert hasattr(sync_func, '_vyra_remote_server')
    
    def test_decorator_default_name(self):
        """Test decorator uses function name if name not provided."""
        @remote_service()
        async def my_function(request):
            return {}
        
        assert my_function._vyra_server_name == "my_function"
    
    def test_decorator_with_protocols(self):
        """Test decorator stores protocol list."""
        @remote_service(protocols=[ProtocolType.ROS2, ProtocolType.REDIS])
        async def test_func(request):
            return {}
        
        assert test_func._vyra_protocols == [ProtocolType.ROS2, ProtocolType.REDIS]
    
    def test_decorator_auto_register_false(self):
        """Test decorator with auto_register=False."""
        @remote_service(auto_register=False)
        async def test_func(request):
            return {}
        
        assert test_func._vyra_auto_register is False
    
    @pytest.mark.asyncio
    async def test_decorated_function_callable(self):
        """Test decorated function can be called."""
        @remote_service()
        async def add_numbers(request):
            return request['a'] + request['b']
        
        result = await add_numbers({'a': 5, 'b': 3})
        assert result == 8


class TestRemoteSpeakerDecorator:
    """Test @remote_publisher decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_publisher(name="test_speaker")
        async def test_func(self, message):
            pass
        
        assert hasattr(test_func, '_vyra_remote_publisher')
        assert test_func._vyra_remote_publisher is True
        assert test_func._vyra_publisher_name == "test_speaker"
    
    def test_decorator_default_name(self):
        """Test decorator uses function name if name not provided."""
        @remote_publisher()
        async def broadcast_status(self, message):
            pass
        
        assert broadcast_status._vyra_publisher_name == "broadcast_status"
    
    def test_decorator_with_protocols(self):
        """Test decorator stores protocol list."""
        @remote_publisher(protocols=[ProtocolType.REDIS, ProtocolType.MQTT])
        async def test_func(self, message):
            pass
        
        assert test_func._vyra_protocols == [ProtocolType.REDIS, ProtocolType.MQTT]


class TestRemoteJobDecorator:
    """Test @remote_actionServer decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_actionServer.on_goal(name="test_job")
        async def test_func(self, goal):
            return {"status": "completed"}
        
        assert hasattr(test_func, '_vyra_remote_action')
        assert test_func._vyra_remote_action is True
        assert test_func._vyra_action_name == "test_job"
    
    def test_decorator_default_name(self):
        """Test decorator uses provided name (name is required for action server)."""
        @remote_actionServer.on_goal(name="long_running_task")
        async def long_running_task(self, goal):
            return {}
        
        assert long_running_task._vyra_action_name == "long_running_task"
    
    def test_decorator_with_protocols(self):
        """Test decorator stores protocol list."""
        @remote_actionServer.on_goal(name="test_func", protocols=[ProtocolType.ROS2])
        async def test_func(self, goal):
            return {}
        
        assert test_func._vyra_protocols == [ProtocolType.ROS2]
    
    @pytest.mark.asyncio
    async def test_decorated_function_callable(self):
        """Test decorated function can be called."""
        @remote_actionServer.on_goal(name="process_data")
        async def process_data(self, goal):
            return {"processed": goal['data']}
        
        # Create mock self object
        class MockSelf:
            pass
        
        result = await process_data(MockSelf(), {'data': 'test'})
        assert result == {"processed": "test"}


# ---------------------------------------------------------------------------
# TestRemoteSubscriberDecorator (new)
# ---------------------------------------------------------------------------

class TestRemoteSubscriberDecorator:
    def test_decorator_sets_metadata(self):
        @remote_subscriber(name="sub_test")
        async def on_msg(msg):
            pass

        assert hasattr(on_msg, "_vyra_remote_subscriber")
        assert on_msg._vyra_subscriber_name == "sub_test"

    def test_decorator_default_name(self):
        @remote_subscriber()
        async def on_message(msg):
            pass

        assert on_message._vyra_subscriber_name == "on_message"

    def test_decorator_registers_blueprint(self):
        @remote_subscriber(name="sub_reg")
        async def on_msg(msg):
            pass

        bp = CallbackRegistry.get_blueprint("sub_reg")
        assert isinstance(bp, SubscriberBlueprint)

    def test_decorator_no_auto_register(self):
        @remote_subscriber(name="sub_no_reg", auto_register=False)
        async def on_msg(msg):
            pass

        assert CallbackRegistry.get_blueprint("sub_no_reg") is None

    @pytest.mark.asyncio
    async def test_async_subscriber_callable(self):
        @remote_subscriber(name="sub_call")
        async def on_msg(msg):
            return {"got": msg}

        result = await on_msg({"x": 1})
        assert result == {"got": {"x": 1}}

    @pytest.mark.asyncio
    async def test_sync_subscriber_wrapped_async(self):
        @remote_subscriber(name="sub_sync_call")
        def on_msg(msg):
            return {"sync": True}

        result = await on_msg("hi")
        assert result == {"sync": True}

    def test_decorator_duplicate_logs_warning(self):
        @remote_subscriber(name="sub_dup")
        async def on1(msg):
            pass

        @remote_subscriber(name="sub_dup")
        async def on2(msg):
            pass


# ---------------------------------------------------------------------------
# TestActionServerDecoratorClass (new)
# ---------------------------------------------------------------------------

class TestActionServerDecoratorClass:
    def test_action_server_on_goal_registers_blueprint(self):
        @remote_actionServer.on_goal(name="act_dec2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        bp = CallbackRegistry.get_blueprint("act_dec2")
        assert isinstance(bp, ActionBlueprint)

    def test_execute_decorator(self):
        @remote_actionServer.on_goal(name="act_exec2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        @remote_actionServer.execute(name="act_exec2", protocols=[ProtocolType.ZENOH])
        async def execute(goal_handle):
            return {}

        assert execute._vyra_action_callback_type == "execute"
        assert execute._vyra_action_name == "act_exec2"

    def test_on_goal_decorator(self):
        @remote_actionServer.on_goal(name="act_goal2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        assert accept._vyra_action_callback_type == "on_goal"

    def test_on_cancel_decorator(self):
        @remote_actionServer.on_goal(name="act_cancel2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        @remote_actionServer.on_cancel(name="act_cancel2")
        async def cancel(goal_handle):
            pass

        assert cancel._vyra_action_callback_type == "on_cancel"

    def test_no_auto_register_on_execute(self):
        """execute with auto_register=False doesn't re-register blueprint."""
        @remote_actionServer.on_goal(name="act_no_rereg", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        @remote_actionServer.execute(name="act_no_rereg")
        async def execute(goal_handle):
            return {}

        assert execute._vyra_auto_register is False

    def test_blueprint_stored_on_wrapper(self):
        @remote_actionServer.on_goal(name="act_bp2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        assert isinstance(accept._vyra_blueprint, ActionBlueprint)


# ---------------------------------------------------------------------------
# TestGetDecoratedMethods (new)
# ---------------------------------------------------------------------------

class TestGetDecoratedMethodsNew:
    def test_empty_object(self):
        class Plain:
            pass

        result = get_decorated_methods(Plain())
        assert result == {"servers": [], "publishers": [], "subscribers": [], "actions": []}

    def test_finds_service(self):
        class Comp:
            @remote_service(name="gdm_svc")
            async def calculate(self, req, resp=None):
                return {}

        result = get_decorated_methods(Comp())
        assert len(result["servers"]) == 1
        assert result["servers"][0]["name"] == "gdm_svc"

    def test_finds_subscriber(self):
        class Comp:
            @remote_subscriber(name="gdm_sub")
            async def on_data(self, msg):
                pass

        result = get_decorated_methods(Comp())
        assert len(result["subscribers"]) == 1

    def test_finds_all_types(self):
        class Comp:
            @remote_service(name="all_svc")
            async def svc(self, req, resp=None):
                return {}

            @remote_publisher(name="all_pub")
            async def pub(self, msg):
                pass

            @remote_subscriber(name="all_sub")
            async def sub(self, msg):
                pass

        result = get_decorated_methods(Comp())
        assert len(result["servers"]) == 1
        assert len(result["publishers"]) == 1
        assert len(result["subscribers"]) == 1

    def test_blueprint_in_result(self):
        class Comp:
            @remote_service(name="bp_in_result")
            async def svc(self, req, resp=None):
                return {}

        result = get_decorated_methods(Comp())
        assert isinstance(result["servers"][0]["blueprint"], ServiceBlueprint)


# ---------------------------------------------------------------------------
# TestBindDecoratedCallbacks (new)
# ---------------------------------------------------------------------------

class TestBindDecoratedCallbacksNew:
    def test_bind_service(self):
        class Comp:
            @remote_service(name="bdc_svc")
            async def svc(self, req, resp=None):
                return {}

        results = bind_decorated_callbacks(Comp())
        assert results.get("bdc_svc") is True

    def test_bind_subscriber(self):
        class Comp:
            @remote_subscriber(name="bdc_sub")
            async def on_msg(self, msg):
                pass

        results = bind_decorated_callbacks(Comp())
        assert results.get("bdc_sub") is True

    def test_bind_publisher(self):
        class Comp:
            @remote_publisher(name="bdc_pub")
            async def pub(self, msg):
                pass

        results = bind_decorated_callbacks(Comp())
        assert results.get("bdc_pub") is True

    def test_force_rebind(self):
        class Comp:
            @remote_service(name="bdc_rebind")
            async def svc(self, req, resp=None):
                return {}

        comp = Comp()
        bind_decorated_callbacks(comp)
        results = bind_decorated_callbacks(comp, force_rebind=True)
        assert results.get("bdc_rebind") is True

    def test_empty_component_returns_empty_dict(self):
        class Empty:
            pass

        results = bind_decorated_callbacks(Empty())
        assert results == {}

    def test_bind_action(self):
        @remote_actionServer.on_goal(name="bdc_action2", protocols=[ProtocolType.ZENOH])
        async def accept(goal):
            return True

        class Comp:
            @remote_actionServer.execute(name="bdc_action2")
            async def execute(self, goal_handle):
                return {}

        results = bind_decorated_callbacks(Comp())
        assert any("bdc_action2" in k for k in results)
