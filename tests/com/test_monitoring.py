"""
Unit Tests for CommunicationMetrics and monitoring decorators.

Tests behavior with prometheus_client unavailable (graceful degradation)
and with mocked prometheus_client (full path coverage).
"""
import pytest
from unittest.mock import patch, MagicMock

from vyra_base.com.core.types import ProtocolType, InterfaceType


# ============================================================================
# Helpers
# ============================================================================

def _reset_metrics_singleton():
    """Reset CommunicationMetrics singleton for test isolation."""
    from vyra_base.com import monitoring as monitoring_module
    monitoring_module.CommunicationMetrics._instance = None
    monitoring_module.CommunicationMetrics._initialized = False


# ============================================================================
# CommunicationMetrics — prometheus unavailable path
# ============================================================================

class TestCommunicationMetricsNoprometheus:
    """Test CommunicationMetrics when prometheus_client is not installed."""

    @pytest.fixture(autouse=True)
    def no_prometheus(self):
        """Patch PROMETHEUS_AVAILABLE to False and reset singleton."""
        _reset_metrics_singleton()
        with patch('vyra_base.com.monitoring.PROMETHEUS_AVAILABLE', False):
            yield
        _reset_metrics_singleton()

    def test_init_disabled(self):
        """Metrics should be disabled when prometheus not available."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        assert m.is_enabled() is False

    def test_singleton(self):
        """Two instances should be the same object."""
        from vyra_base.com.monitoring import CommunicationMetrics
        assert CommunicationMetrics() is CommunicationMetrics()

    def test_record_call_noop(self):
        """record_call should not raise when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.record_call(ProtocolType.REDIS, InterfaceType.SERVER, "fn", 0.1, True)
        m.record_call(ProtocolType.REDIS, InterfaceType.SERVER, "fn", 0.1, False)

    def test_record_error_noop(self):
        """record_error should not raise when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.record_error(ProtocolType.REDIS, InterfaceType.SERVER, "SomeError")

    def test_record_message_size_noop(self):
        """record_message_size should not raise when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.record_message_size(ProtocolType.REDIS, InterfaceType.SERVER, 512, 'send')
        m.record_message_size(ProtocolType.REDIS, InterfaceType.SERVER, 512, 'receive')

    def test_track_connection_noop(self):
        """track_connection context manager should work when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        with m.track_connection(ProtocolType.REDIS):
            pass  # Should not raise

    def test_update_protocol_info_noop(self):
        """update_protocol_info should not raise when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.update_protocol_info(['redis', 'ros2'])

    def test_ros2_methods_noop(self):
        """All ROS2 record methods should be noops when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.record_ros2_topic_message("topic", "send")
        m.record_ros2_message_drop("topic", "reliability")
        m.record_ros2_qos_violation("topic", "deadline")
        m.update_ros2_discovery(5, 10)

    def test_dds_methods_noop(self):
        """All DDS record methods should be noops when disabled."""
        from vyra_base.com.monitoring import CommunicationMetrics
        m = CommunicationMetrics()
        m.record_dds_writer_liveliness_lost("topic")
        m.record_dds_reader_sample_lost("topic", 3)
        m.record_dds_deadline_missed("topic", "publisher")


# ============================================================================
# CommunicationMetrics — prometheus available path (mocked)
# ============================================================================

class TestCommunicationMetricsWithPrometheus:
    """Test CommunicationMetrics with mocked prometheus_client.
    
    Skipped if prometheus_client is not installed and cannot be mocked cleanly.
    The disabled-path is already covered by TestCommunicationMetricsNoprometheus.
    """

    @pytest.fixture(autouse=True)
    def mock_prometheus(self):
        """Inject fake prometheus_client and reset singleton."""
        import importlib
        import sys
        import vyra_base.com.monitoring as monitoring_module

        _reset_metrics_singleton()

        mock_counter = MagicMock()
        mock_counter.labels.return_value = MagicMock()
        mock_histogram = MagicMock()
        mock_histogram.labels.return_value = MagicMock()
        mock_gauge = MagicMock()
        mock_gauge.labels.return_value = MagicMock()
        mock_gauge.set = MagicMock()
        mock_info = MagicMock()

        fake_prom = MagicMock()
        fake_prom.Counter.return_value = mock_counter
        fake_prom.Histogram.return_value = mock_histogram
        fake_prom.Gauge.return_value = mock_gauge
        fake_prom.Info.return_value = mock_info

        original = sys.modules.get('prometheus_client')
        sys.modules['prometheus_client'] = fake_prom

        try:
            importlib.reload(monitoring_module)
            _reset_metrics_singleton()
            yield mock_counter, mock_histogram, mock_gauge, mock_info
        finally:
            if original is None:
                sys.modules.pop('prometheus_client', None)
            else:
                sys.modules['prometheus_client'] = original
            importlib.reload(monitoring_module)
            _reset_metrics_singleton()

    def test_init_enabled(self, mock_prometheus):
        """Metrics should be enabled when prometheus is available."""
        import vyra_base.com.monitoring as monitoring_module
        m = monitoring_module.CommunicationMetrics()
        assert m.is_enabled() is True

    def test_record_call(self, mock_prometheus):
        """record_call should invoke counter and histogram."""
        import vyra_base.com.monitoring as monitoring_module
        mock_counter, mock_histogram, _, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.record_call(ProtocolType.REDIS, InterfaceType.SERVER, "fn", 0.05, True)
        mock_counter.labels.assert_called()
        mock_histogram.labels.assert_called()

    def test_record_call_failure(self, mock_prometheus):
        """record_call with success=False should use 'error' status."""
        import vyra_base.com.monitoring as monitoring_module
        mock_counter, _, _, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.record_call(ProtocolType.REDIS, InterfaceType.SERVER, "fn", 0.05, False)
        mock_counter.labels.assert_called()

    def test_record_error(self, mock_prometheus):
        """record_error should call error counter."""
        import vyra_base.com.monitoring as monitoring_module
        mock_counter, _, _, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.record_error(ProtocolType.REDIS, InterfaceType.SERVER, "ValueError")
        mock_counter.labels.assert_called()

    def test_record_message_size(self, mock_prometheus):
        """record_message_size should call histogram."""
        import vyra_base.com.monitoring as monitoring_module
        _, mock_histogram, _, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.record_message_size(ProtocolType.REDIS, InterfaceType.SERVER, 1024, 'send')
        mock_histogram.labels.assert_called()

    def test_track_connection(self, mock_prometheus):
        """track_connection should call gauge."""
        import vyra_base.com.monitoring as monitoring_module
        _, _, mock_gauge, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        with m.track_connection(ProtocolType.REDIS):
            mock_gauge.labels.assert_called()

    def test_update_protocol_info(self, mock_prometheus):
        """update_protocol_info should call info."""
        import vyra_base.com.monitoring as monitoring_module
        _, _, _, mock_info = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.update_protocol_info(['redis', 'ros2'])
        mock_info.info.assert_called()

    def test_ros2_and_dds_methods(self, mock_prometheus):
        """ROS2 and DDS methods should not raise."""
        import vyra_base.com.monitoring as monitoring_module
        mock_counter, _, mock_gauge, _ = mock_prometheus
        m = monitoring_module.CommunicationMetrics()
        m.record_ros2_topic_message("/topic", "send")
        m.record_ros2_message_drop("/topic", "reliability")
        m.record_ros2_qos_violation("/topic", "deadline")
        m.update_ros2_discovery(3, 7)
        m.record_dds_writer_liveliness_lost("/topic")
        m.record_dds_reader_sample_lost("/topic", 2)
        m.record_dds_deadline_missed("/topic", "publisher")
        mock_counter.labels.assert_called()


# ============================================================================
# Decorators — monitored_callable
# ============================================================================

class TestMonitoredCallable:
    """Test the monitored_callable decorator."""

    @pytest.fixture(autouse=True)
    def no_prometheus(self):
        """Use disabled metrics (no prometheus) for isolation."""
        _reset_metrics_singleton()
        with patch('vyra_base.com.monitoring.PROMETHEUS_AVAILABLE', False):
            yield
        _reset_metrics_singleton()

    @pytest.mark.asyncio
    async def test_async_callable_success(self):
        """Decorated async function should return result normally."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS, interface_name="test_fn")
        async def my_func(request):
            return {"ok": True}

        result = await my_func("req")
        assert result == {"ok": True}

    @pytest.mark.asyncio
    async def test_async_callable_exception_propagates(self):
        """Decorated async function should re-raise exceptions."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS, interface_name="failing_fn")
        async def my_func(request):
            raise ValueError("boom")

        with pytest.raises(ValueError, match="boom"):
            await my_func("req")

    def test_sync_callable_success(self):
        """Decorated sync function should return result normally."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS, interface_name="sync_fn")
        def my_func(x):
            return x * 2

        result = my_func(5)
        assert result == 10

    def test_sync_callable_exception_propagates(self):
        """Decorated sync function should re-raise exceptions."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS)
        def my_func():
            raise RuntimeError("sync error")

        with pytest.raises(RuntimeError, match="sync error"):
            my_func()

    @pytest.mark.asyncio
    async def test_auto_detect_interface_name(self):
        """If interface_name is None, function name should be used."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS)
        async def auto_named():
            return "result"

        result = await auto_named()
        assert result == "result"

    @pytest.mark.asyncio
    async def test_auto_detect_protocol_from_instance(self):
        """If protocol is None, should auto-detect from instance.protocol."""
        from vyra_base.com.monitoring import monitored_callable

        class FakeCallable:
            protocol = ProtocolType.REDIS

            @monitored_callable()
            async def action(self, request):
                return "done"

        obj = FakeCallable()
        result = await obj.action("req")
        assert result == "done"

    @pytest.mark.asyncio
    async def test_track_message_size(self):
        """track_message_size=True should not raise even without prometheus."""
        from vyra_base.com.monitoring import monitored_callable

        @monitored_callable(protocol=ProtocolType.REDIS, track_message_size=True)
        async def sized_fn(request):
            return {"response": "data"}

        result = await sized_fn("test_request")
        assert result == {"response": "data"}


# ============================================================================
# Decorators — monitored_speaker
# ============================================================================

class TestMonitoredSpeaker:
    """Test the monitored_speaker decorator."""

    @pytest.fixture(autouse=True)
    def no_prometheus(self):
        _reset_metrics_singleton()
        with patch('vyra_base.com.monitoring.PROMETHEUS_AVAILABLE', False):
            yield
        _reset_metrics_singleton()

    @pytest.mark.asyncio
    async def test_speaker_success(self):
        """Decorated speaker should return result normally."""
        from vyra_base.com.monitoring import monitored_speaker

        @monitored_speaker(protocol=ProtocolType.REDIS, speaker_name="my_topic")
        async def publish(message):
            return True

        result = await publish("hello")
        assert result is True

    @pytest.mark.asyncio
    async def test_speaker_exception_propagates(self):
        """Speaker decorator should re-raise exceptions."""
        from vyra_base.com.monitoring import monitored_speaker

        @monitored_speaker(protocol=ProtocolType.REDIS)
        async def publish(message):
            raise ConnectionError("lost")

        with pytest.raises(ConnectionError, match="lost"):
            await publish("msg")

    @pytest.mark.asyncio
    async def test_speaker_auto_detect_name(self):
        """speaker_name defaults to function name."""
        from vyra_base.com.monitoring import monitored_speaker

        @monitored_speaker(protocol=ProtocolType.REDIS)
        async def auto_named_speaker(msg):
            return msg

        result = await auto_named_speaker("data")
        assert result == "data"

    @pytest.mark.asyncio
    async def test_speaker_auto_detect_protocol(self):
        """Protocol auto-detected from instance.protocol."""
        from vyra_base.com.monitoring import monitored_speaker

        class FakeSpeaker:
            protocol = ProtocolType.REDIS

            @monitored_speaker()
            async def broadcast(self, msg):
                return msg

        obj = FakeSpeaker()
        result = await obj.broadcast("hello")
        assert result == "hello"
