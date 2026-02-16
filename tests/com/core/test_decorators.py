"""
Unit tests for decorators (@remote_callable, @remote_speaker, @remote_job).
"""
import pytest
import asyncio
from vyra_base.com.core.decorators import remote_service, remote_speaker, remote_job
from vyra_base.com.core.types import ProtocolType


class TestRemoteCallableDecorator:
    """Test @remote_callable decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_service(name="test_callable")
        async def test_func(request):
            return {"result": "success"}
        
        assert hasattr(test_func, '_vyra_remote_callable')
        assert test_func._vyra_remote_callable is True
        assert test_func._vyra_callable_name == "test_callable"
    
    def test_decorator_on_sync_function(self):
        """Test decorator converts sync to async."""
        @remote_service(name="sync_callable")
        def sync_func(request):
            return {"result": "sync"}
        
        # Should be converted to async
        assert asyncio.iscoroutinefunction(sync_func)
        assert hasattr(sync_func, '_vyra_remote_callable')
    
    def test_decorator_default_name(self):
        """Test decorator uses function name if name not provided."""
        @remote_service()
        async def my_function(request):
            return {}
        
        assert my_function._vyra_callable_name == "my_function"
    
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
    """Test @remote_speaker decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_speaker(name="test_speaker")
        async def test_func(self, message):
            pass
        
        assert hasattr(test_func, '_vyra_remote_speaker')
        assert test_func._vyra_remote_speaker is True
        assert test_func._vyra_speaker_name == "test_speaker"
    
    def test_decorator_default_name(self):
        """Test decorator uses function name if name not provided."""
        @remote_speaker()
        async def broadcast_status(self, message):
            pass
        
        assert broadcast_status._vyra_speaker_name == "broadcast_status"
    
    def test_decorator_with_protocols(self):
        """Test decorator stores protocol list."""
        @remote_speaker(protocols=[ProtocolType.REDIS, ProtocolType.MQTT])
        async def test_func(self, message):
            pass
        
        assert test_func._vyra_protocols == [ProtocolType.REDIS, ProtocolType.MQTT]


class TestRemoteJobDecorator:
    """Test @remote_job decorator."""
    
    def test_decorator_on_async_function(self):
        """Test decorator works on async functions."""
        @remote_job(name="test_job")
        async def test_func(self, goal):
            return {"status": "completed"}
        
        assert hasattr(test_func, '_vyra_remote_job')
        assert test_func._vyra_remote_job is True
        assert test_func._vyra_job_name == "test_job"
    
    def test_decorator_default_name(self):
        """Test decorator uses function name if name not provided."""
        @remote_job()
        async def long_running_task(self, goal):
            return {}
        
        assert long_running_task._vyra_job_name == "long_running_task"
    
    def test_decorator_with_protocols(self):
        """Test decorator stores protocol list."""
        @remote_job(protocols=[ProtocolType.ROS2])
        async def test_func(self, goal):
            return {}
        
        assert test_func._vyra_protocols == [ProtocolType.ROS2]
    
    @pytest.mark.asyncio
    async def test_decorated_function_callable(self):
        """Test decorated function can be called."""
        @remote_job()
        async def process_data(self, goal):
            return {"processed": goal['data']}
        
        # Create mock self object
        class MockSelf:
            pass
        
        result = await process_data(MockSelf(), {'data': 'test'})
        assert result == {"processed": "test"}
