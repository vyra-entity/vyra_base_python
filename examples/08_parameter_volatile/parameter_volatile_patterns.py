"""Patterns for using Parameter and Volatile structures in modules."""

from __future__ import annotations

import asyncio


async def parameter_pattern() -> None:
    """Show the typical Parameter workflow in a module."""
    print("1) Create Parameter manager during entity setup")
    print("2) Load defaults via load_defaults()")
    print("3) Read with get_parameter_impl(key)")
    print("4) Update through set_parameter service")


async def volatile_pattern() -> None:
    """Show the typical Volatile workflow in a module."""
    print("1) Create Volatile manager with Redis transient storage")
    print("2) Set value with set_volatile_value(key, value)")
    print("3) Read value with get_volatile_value(key)")
    print("4) Publish changes via publish_volatile_to_ros2(key)")


async def main() -> None:
    """Run both pattern summaries."""
    await parameter_pattern()
    await volatile_pattern()


if __name__ == "__main__":
    asyncio.run(main())
