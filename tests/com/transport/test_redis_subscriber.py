"""
Unit tests for Redis Subscriber callback wrapping.

Verifies that the RedisSubscriberImpl correctly adapts the Redis
transport signature (message_dict, context) to the user-facing
single-argument callback.
"""
import pytest
import asyncio
import json
from unittest.mock import AsyncMock, MagicMock, patch

from vyra_base.com.transport.t_redis.vyra_models.subscriber import RedisSubscriberImpl


@pytest.mark.unit
@pytest.mark.asyncio
class TestRedisSubscriberCallbackWrapping:
    """Ensure the Redis subscriber wraps callbacks to extract data."""

    async def test_callback_receives_parsed_data(self):
        """User callback should receive parsed JSON data, not the raw Redis message."""
        received = []

        async def user_cb(data):
            received.append(data)

        topic_builder = MagicMock()
        topic_builder.build.return_value = "mod/ns/topic"

        redis_client = AsyncMock()
        redis_client.subscribe_channel = AsyncMock()

        # Capture the callback that create_pubsub_listener receives
        captured_cb = None

        async def capture_listener(channel, cb):
            nonlocal captured_cb
            captured_cb = cb

        redis_client.create_pubsub_listener = capture_listener

        sub = RedisSubscriberImpl(
            name="test_msg",
            topic_builder=topic_builder,
            subscriber_callback=user_cb,
            redis_client=redis_client,
            message_type=dict,
        )
        sub._topic_name = "mod/ns/topic"

        await sub.subscribe()
        assert captured_cb is not None

        # Simulate a Redis pubsub message with 2 args
        redis_message = {
            "type": "message",
            "channel": "mod/ns/topic",
            "pattern": None,
            "data": json.dumps({"key": "value"}),
        }
        await captured_cb(redis_message, None)

        assert len(received) == 1
        assert received[0] == {"key": "value"}

    async def test_callback_handles_bytes_data(self):
        """Callback should decode bytes data from Redis."""
        received = []

        async def user_cb(data):
            received.append(data)

        topic_builder = MagicMock()
        topic_builder.build.return_value = "t"

        redis_client = AsyncMock()
        redis_client.subscribe_channel = AsyncMock()

        captured_cb = None

        async def capture_listener(channel, cb):
            nonlocal captured_cb
            captured_cb = cb

        redis_client.create_pubsub_listener = capture_listener

        sub = RedisSubscriberImpl(
            name="test",
            topic_builder=topic_builder,
            subscriber_callback=user_cb,
            redis_client=redis_client,
            message_type=dict,
        )
        sub._topic_name = "t"
        await sub.subscribe()

        await captured_cb({"data": b'{"hello": "world"}'}, None)

        assert received == [{"hello": "world"}]

    async def test_sync_callback_also_works(self):
        """A synchronous user callback should also be called correctly."""
        received = []

        def sync_cb(data):
            received.append(data)

        topic_builder = MagicMock()
        topic_builder.build.return_value = "t"

        redis_client = AsyncMock()
        redis_client.subscribe_channel = AsyncMock()

        captured_cb = None

        async def capture_listener(channel, cb):
            nonlocal captured_cb
            captured_cb = cb

        redis_client.create_pubsub_listener = capture_listener

        sub = RedisSubscriberImpl(
            name="test",
            topic_builder=topic_builder,
            subscriber_callback=sync_cb,
            redis_client=redis_client,
            message_type=dict,
        )
        sub._topic_name = "t"
        await sub.subscribe()

        await captured_cb({"data": '{"x": 1}'}, None)

        assert received == [{"x": 1}]
