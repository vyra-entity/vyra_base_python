"""Coverage-style usage samples for helper functions."""

from __future__ import annotations

import asyncio
from pathlib import Path

from vyra_base.helper.env_handler import get_env_required
from vyra_base.helper.file_lock import (
    get_lock_for_file,
    get_lock_for_file_sync,
    release_lock_for_file,
    release_lock_for_file_sync,
)
from vyra_base.helper.func import deep_merge, fuzzy_match


async def run_async_lock_example(path: Path) -> None:
    """Exercise async file lock helpers."""
    await get_lock_for_file(path)
    await release_lock_for_file(path)


def run_sync_lock_example(path: Path) -> None:
    """Exercise sync file lock helpers."""
    get_lock_for_file_sync(path)
    release_lock_for_file_sync(path)


def run_merge_and_match_examples() -> None:
    """Exercise dict merge and fuzzy matching helpers."""
    merged = deep_merge({"a": {"b": 1}}, {"a": {"c": 2}, "d": 3})
    print("deep_merge:", merged)

    score = fuzzy_match("status_stream", ["state_stream", "status_stream", "stat"], 3)
    print("fuzzy_match:", score)


def run_env_example() -> None:
    """Show get_env_required error handling pattern."""
    try:
        value = get_env_required("VYRA_EXAMPLE_ENV")
        print("VYRA_EXAMPLE_ENV:", value)
    except ValueError as exc:
        print("Expected when env var missing:", exc)


async def main() -> None:
    """Run all helper usage snippets."""
    tmp_file = Path("/tmp/vyra_helper_example.txt")
    run_merge_and_match_examples()
    run_sync_lock_example(tmp_file)
    await run_async_lock_example(tmp_file)
    run_env_example()


if __name__ == "__main__":
    asyncio.run(main())
