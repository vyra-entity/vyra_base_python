"""Shared memory roundtrip example with local create/write/read cycle."""

from vyra_base.com.external.shared_memory.segment import SharedMemorySegment


def main() -> None:
    """Create a temporary shared-memory segment and roundtrip one message."""
    segment_name = "/vyra_example_external_shm"
    with SharedMemorySegment(segment_name, 2048, create=True) as segment:
        ok = segment.write({"message": "hello", "value": 42})
        print(f"write_ok={ok}")
        message_type, payload, timestamp = segment.read()
        print(f"message_type={message_type}")
        print(f"payload={payload}")
        print(f"timestamp={timestamp}")


if __name__ == "__main__":
    main()
