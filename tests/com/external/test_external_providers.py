"""Tests for external communication clients and the external registry."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from vyra_base.com.external.grpc import grpc_client
from vyra_base.com.external.mqtt import mqtt_client
from vyra_base.com.external.registry import (
    ExternalRegistry,
    ProtocolStatus,
    get_global_registry,
)
from vyra_base.com.external.rest import rest_client
from vyra_base.com.external.websocket import websocket_client


class TestExternalClientAvailabilityGuards:
    """Verify deterministic dependency guards for optional external clients."""

    def test_grpc_client_raises_when_grpc_unavailable(self, monkeypatch):
        """GrpcClient should fail fast when grpc dependency is unavailable."""
        monkeypatch.setattr(grpc_client, "GRPC_AVAILABLE", False)
        with pytest.raises(ImportError, match="grpcio not installed"):
            grpc_client.GrpcClient(target="localhost:50051")

    def test_mqtt_client_raises_when_mqtt_unavailable(self, monkeypatch):
        """MqttClient should fail fast when paho-mqtt dependency is unavailable."""
        monkeypatch.setattr(mqtt_client, "MQTT_AVAILABLE", False)
        with pytest.raises(ImportError, match="paho-mqtt not installed"):
            mqtt_client.MqttClient(broker="localhost")

    def test_rest_client_raises_when_httpx_unavailable(self, monkeypatch):
        """RestClient should fail fast when httpx dependency is unavailable."""
        monkeypatch.setattr(rest_client, "REST_AVAILABLE", False)
        with pytest.raises(ImportError, match="httpx not installed"):
            rest_client.RestClient(base_url="http://localhost")

    def test_websocket_client_raises_when_websocket_unavailable(self, monkeypatch):
        """WebSocketClient should fail fast when websockets dependency is unavailable."""
        monkeypatch.setattr(websocket_client, "WEBSOCKET_AVAILABLE", False)
        with pytest.raises(ImportError, match="websockets not installed"):
            websocket_client.WebSocketClient(url="ws://localhost")


class TestExternalRegistry:
    """Test ExternalRegistry behavior without network dependencies."""

    @pytest.mark.asyncio
    async def test_register_get_and_filter_connections(self):
        """Registered connections should be retrievable and filterable."""
        registry = ExternalRegistry()
        await registry.register("rest_api", "rest", "http://localhost:8000")
        await registry.register("mqtt_broker", "mqtt", "localhost:1883")

        assert registry.has_connection("rest_api")
        assert registry.get_connection("rest_api") is not None
        assert registry.get_connection("rest_api").endpoint == "http://localhost:8000"

        rest_connections = registry.list_connections(protocol_type="rest")
        assert len(rest_connections) == 1
        assert rest_connections[0].name == "rest_api"

    @pytest.mark.asyncio
    async def test_unregister_calls_async_close(self):
        """Unregister should close async clients when available."""
        registry = ExternalRegistry()

        closed = {"value": False}

        class AsyncClosableClient:
            """Minimal async-closable test client."""

            async def close(self):
                """Track close invocation for assertions."""
                closed["value"] = True

        await registry.register(
            "grpc_service",
            "grpc",
            "localhost:50051",
            client=AsyncClosableClient(),
        )

        result = await registry.unregister("grpc_service")
        assert result is True
        assert closed["value"] is True
        assert registry.get_connection("grpc_service") is None

    @pytest.mark.asyncio
    async def test_health_check_not_found(self):
        """Health check for unknown connection should return not_found payload."""
        registry = ExternalRegistry()
        result = await registry.health_check("missing")

        assert result["name"] == "missing"
        assert result["status"] == "not_found"
        assert result["healthy"] is False

    @pytest.mark.asyncio
    async def test_health_check_uses_client_is_connected(self):
        """Health check should prioritize client connection state when available."""
        registry = ExternalRegistry()
        client = SimpleNamespace(is_connected=True)
        connection = await registry.register(
            "ws_events",
            "websocket",
            "ws://localhost:8765",
            client=client,
        )
        connection.update_status(ProtocolStatus.DISCONNECTED)

        result = await registry.health_check("ws_events")
        assert result["healthy"] is True
        assert result["status"] == ProtocolStatus.DISCONNECTED.value

    @pytest.mark.asyncio
    async def test_health_check_handles_client_errors(self):
        """Health checks should downgrade status to error when client inspection fails."""
        registry = ExternalRegistry()

        class BrokenClient:
            """Client whose health probe always fails."""

            @property
            def is_connected(self):
                """Raise runtime error to simulate probing failure."""
                raise RuntimeError("probe failed")

        connection = await registry.register(
            "broken_rest",
            "rest",
            "http://localhost:9000",
            client=BrokenClient(),
        )
        connection.update_status(ProtocolStatus.CONNECTED)

        result = await registry.health_check("broken_rest")
        assert result["healthy"] is False
        assert result["status"] == ProtocolStatus.ERROR.value
        assert connection.error_count == 1

    @pytest.mark.asyncio
    async def test_registry_statistics_counts_protocols_and_statuses(self):
        """Statistics should report accurate counts by protocol and status."""
        registry = ExternalRegistry()
        conn_a = await registry.register("a", "grpc", "localhost:5001")
        conn_b = await registry.register("b", "grpc", "localhost:5002")
        conn_c = await registry.register("c", "mqtt", "localhost:1883")

        conn_a.update_status(ProtocolStatus.CONNECTED)
        conn_b.update_status(ProtocolStatus.ERROR, "boom")
        conn_c.update_status(ProtocolStatus.CONNECTED)

        stats = registry.get_statistics()
        assert stats["total_connections"] == 3
        assert stats["by_protocol"] == {"grpc": 2, "mqtt": 1}
        assert stats["by_status"][ProtocolStatus.CONNECTED.value] == 2
        assert stats["by_status"][ProtocolStatus.ERROR.value] == 1

    def test_get_global_registry_returns_singleton(self):
        """Global registry accessor should return the same singleton instance."""
        registry_a = get_global_registry()
        registry_b = get_global_registry()

        assert registry_a is registry_b
