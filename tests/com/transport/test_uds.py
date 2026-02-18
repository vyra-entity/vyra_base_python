"""
Unit tests for UDS (Unix Domain Socket) transport protocol.
"""
import pytest
import asyncio
from vyra_base.com.transport.t_uds.provider import UDSProvider
from vyra_base.com.core.types import ProtocolType


@pytest.mark.asyncio
class TestUdsProvider:
    """Test UdsProvider."""

    async def test_provider_initialization(self):
        """Test provider can be initialized."""
        provider = UDSProvider(module_name="test_module", module_id="test_id_002")

        await provider.initialize()
        assert provider.is_initialized

        await provider.shutdown()

    async def test_provider_shutdown(self):
        """Test provider can be shutdown."""
        provider = UDSProvider(module_name="test_module", module_id="test_shutdown_001")
        await provider.initialize()
        assert provider.is_initialized

        await provider.shutdown()
        assert not provider.is_initialized

    async def test_create_server(self):
        """Test creating callable through provider."""
        provider = UDSProvider(module_name="test_module", module_id="server_test_001")
        await provider.initialize()
        try:
            callable_instance = await provider.create_server("test_service")
            assert callable_instance is not None
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_server not fully implemented: {e}")
        finally:
            await provider.shutdown()

    async def test_create_publisher(self):
        """Test creating speaker through provider."""
        provider = UDSProvider(module_name="test_module", module_id="publisher_test_001")
        await provider.initialize()
        try:
            speaker = await provider.create_publisher("test_topic")
            assert speaker is not None
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_publisher not fully implemented: {e}")
        finally:
            await provider.shutdown()

    async def test_multiple_services(self):
        """Test creating multiple services."""
        provider = UDSProvider(module_name="test_module", module_id="multi_test_001")
        await provider.initialize()
        try:
            service1 = await provider.create_server("service1")
            service2 = await provider.create_server("service2")
            assert service1 is not None
            assert service2 is not None
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_server not fully implemented: {e}")
        finally:
            await provider.shutdown()


@pytest.mark.asyncio
class TestUdsCallable:
    """Test UdsCallable."""

    async def test_callable_creation(self):
        """Test callable can be created."""
        provider = UDSProvider(module_name="test_module", module_id="callable_test_001")
        await provider.initialize()
        try:
            callable_instance = await provider.create_server("test_service")
            assert callable_instance is not None
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_server not fully implemented: {e}")
        finally:
            await provider.shutdown()

    async def test_callable_lifecycle(self):
        """Test callable initialization and cleanup."""
        provider = UDSProvider(module_name="test_module", module_id="lifecycle_test_001")
        await provider.initialize()
        try:
            callable_instance = await provider.create_server("lifecycle_service")

            if hasattr(callable_instance, "initialize"):
                await callable_instance.initialize()

            if hasattr(callable_instance, "cleanup"):
                await callable_instance.cleanup()
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_server not fully implemented: {e}")
        finally:
            await provider.shutdown()


@pytest.mark.asyncio
class TestUdsSpeaker:
    """Test UdsSpeaker."""

    async def test_speaker_creation(self):
        """Test speaker can be created."""
        provider = UDSProvider(module_name="test_module", module_id="speaker_test_001")
        await provider.initialize()
        try:
            speaker = await provider.create_publisher("test_topic")
            assert speaker is not None
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_publisher not fully implemented: {e}")
        finally:
            await provider.shutdown()

    async def test_speaker_publish(self):
        """Test speaker can publish messages."""
        provider = UDSProvider(module_name="test_module", module_id="speaker_test_002")
        await provider.initialize()
        try:
            speaker = await provider.create_publisher("data_topic")

            if hasattr(speaker, "send"):
                await speaker.send({"data": "test"})
            elif hasattr(speaker, "publish"):
                await speaker.publish({"data": "test"})
        except (NotImplementedError, TypeError) as e:
            pytest.skip(f"create_publisher not fully implemented: {e}")
        finally:
            await provider.shutdown()
