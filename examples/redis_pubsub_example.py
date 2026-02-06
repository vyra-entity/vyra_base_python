"""
Example: Redis Pub/Sub with RedisProvider

Demonstrates Redis transport:
- RedisProvider and create_speaker (publisher and subscriber)
- Publish with shout()
- Subscribe with listen(callback)

Requires Redis running (e.g. docker run -p 6379:6379 redis).

Usage:
  # Terminal 1: subscriber (listener)
  python examples/redis_pubsub_example.py subscriber

  # Terminal 2: publisher
  python examples/redis_pubsub_example.py publisher
"""

import asyncio
import sys

try:
    from vyra_base.com.core.types import ProtocolType
    from vyra_base.com.transport.t_redis import RedisProvider, REDIS_AVAILABLE
except ImportError as e:
    print(f"Import error: {e}")
    REDIS_AVAILABLE = False


async def run_subscriber():
    """Run Redis subscriber: listen on channel and print messages."""
    if not REDIS_AVAILABLE:
        print("Redis transport not available.")
        return

    provider = RedisProvider(
        protocol=ProtocolType.REDIS,
        module_name="example_sub",
        host="localhost",
        port=6379,
    )
    await provider.initialize()

    channel = "vyra_demo_events"
    speaker = await provider.create_speaker(channel)

    def on_message(msg):
        print(f"Received: {msg}")

    await speaker.listen(on_message)
    print(f"Subscriber listening on channel '{channel}'. (Ctrl+C to stop)")
    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        pass
    await speaker.shutdown()
    await provider.shutdown()


async def run_publisher():
    """Run Redis publisher: send a few messages then exit."""
    if not REDIS_AVAILABLE:
        print("Redis transport not available.")
        return

    provider = RedisProvider(
        protocol=ProtocolType.REDIS,
        module_name="example_pub",
        host="localhost",
        port=6379,
    )
    await provider.initialize()

    channel = "vyra_demo_events"
    speaker = await provider.create_speaker(channel)

    for i in range(5):
        msg = {"event": "tick", "count": i + 1}
        await speaker.shout(msg)
        print(f"Published: {msg}")
        await asyncio.sleep(1.0)

    await speaker.shutdown()
    await provider.shutdown()
    print("Publisher done.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python redis_pubsub_example.py subscriber | publisher")
        return
    mode = sys.argv[1].lower()
    if mode == "subscriber":
        asyncio.run(run_subscriber())
    elif mode == "publisher":
        asyncio.run(run_publisher())
    else:
        print("Use 'subscriber' or 'publisher'")


if __name__ == "__main__":
    main()
