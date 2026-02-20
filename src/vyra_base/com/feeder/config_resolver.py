"""
Interface-config based protocol resolver for VYRA feeders.

When a feeder starts, it calls :func:`FeederConfigResolver.resolve` with its
``feeder_name`` (= ``functionname`` in the JSON config).  The resolver
searches all ``interface config`` JSON files for a matching publisher entry
and returns which transport protocol (zenoh, ros2, redis, uds, …) to use,
which protobuf/message file type to load, and what tags are attached to the
interface.

Interface config file schema (array of entries)::

    [
        {
            "type":         "publisher",        # only these are relevant
            "functionname": "StateFeed",        # matched against feeder_name
            "tags":         ["zenoh"],          # → ProtocolType.ZENOH
            "filetype":     ["VBASEStateFeed.proto"],
            "displayname":  "State Feed",
            "description":  "...",
            ...
        },
        ...
    ]

If no matching publisher entry is found the resolver logs an actionable error
message and uses :func:`~vyra_base.helper.func.fuzzy_match` to propose similar
publisher names from all discovered configs.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from vyra_base.helper.func import fuzzy_match

logger = logging.getLogger(__name__)

# Map tag strings → ProtocolType-compatible strings (avoid hard import cycle)
_TAG_TO_PROTOCOL: dict[str, str] = {
    "zenoh":  "zenoh",
    "ros2":   "ros2",
    "redis":  "redis",
    "uds":    "uds",
}

# Default fallback chain when no tag matches a known protocol
_DEFAULT_FALLBACK: list[str] = ["zenoh", "ros2", "redis", "uds"]


@dataclass
class FeederResolverResult:
    """Result returned by :meth:`FeederConfigResolver.resolve`.

    :param feeder_name: The ``functionname`` that was matched.
    :param protocol: The resolved protocol string (e.g. ``"zenoh"``).
    :param tags: Full list of tags from the matching entry.
    :param file_types: List of interface file names (e.g.
        ``["VBASEStateFeed.proto"]``).
    :param display_name: Human-readable display name from the config.
    :param description: Description from the config.
    :param raw_entry: The full raw JSON entry dict.
    :param source_file: Path to the JSON file that contained this entry.
    """

    feeder_name: str
    protocol: str
    tags: list[str] = field(default_factory=list)
    file_types: list[str] = field(default_factory=list)
    display_name: str = ""
    description: str = ""
    raw_entry: dict = field(default_factory=dict)
    source_file: str = ""


class FeederConfigResolver:
    """Resolves a feeder's transport protocol from interface config JSON files.

    The resolver is **stateless**: every :meth:`resolve` call re-reads and
    parses the config files.  In practice the files are small and cached by
    the OS page cache, so this is not a performance concern during module
    initialisation.

    Usage::

        from vyra_base.com.feeder.config_resolver import FeederConfigResolver

        result = FeederConfigResolver.resolve(
            feeder_name="StateFeed",
            interface_paths=["/workspace/install/my_interfaces/share/my_interfaces/config"],
        )
        if result is None:
            raise RuntimeError("StateFeed not found in interface configs")

        print(result.protocol)   # "zenoh"
        print(result.file_types) # ["VBASEStateFeed.proto"]
    """

    @staticmethod
    def resolve(
        feeder_name: str,
        interface_paths: Sequence[str | Path],
        *,
        use_fallback: bool = True,
        case_sensitive: bool = False,
    ) -> Optional[FeederResolverResult]:
        """Find the interface config entry for *feeder_name* and resolve its
        transport protocol.

        Searches all JSON files found (recursively) under each path in
        *interface_paths*.  Only entries with ``"type": "publisher"`` are
        considered.

        :param feeder_name: The ``functionname`` to look up (e.g.
            ``"StateFeed"``).
        :type feeder_name: str
        :param interface_paths: Directories or JSON file paths to search.
        :type interface_paths: Sequence[str | Path]
        :param use_fallback: If ``True`` and the matched entry carries no
            recognised protocol tag, fall back to the default chain
            ``[zenoh, ros2, redis, uds]`` and use the first available
            protocol.  Defaults to ``True``.
        :type use_fallback: bool
        :param case_sensitive: Whether ``functionname`` comparison is
            case-sensitive.  Defaults to ``False``.
        :type case_sensitive: bool
        :return: A :class:`FeederResolverResult` when a match is found,
            ``None`` otherwise.  On failure an actionable error is logged
            with fuzzy-match suggestions.
        :rtype: Optional[FeederResolverResult]
        """
        all_publisher_names: list[str] = []
        compare_name = feeder_name if case_sensitive else feeder_name.lower()

        for json_path in FeederConfigResolver._iter_json_files(interface_paths):
            entries = FeederConfigResolver._load_json(json_path)
            if entries is None:
                continue

            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                if entry.get("type", "").lower() != "publisher":
                    continue

                func_name: str = entry.get("functionname", "")
                all_publisher_names.append(func_name)

                match_name = func_name if case_sensitive else func_name.lower()
                if match_name != compare_name:
                    continue

                # ── Match found ──────────────────────────────────────────
                tags: list[str] = entry.get("tags", [])
                protocol = FeederConfigResolver._resolve_protocol(
                    tags, use_fallback=use_fallback
                )

                if protocol is None:
                    logger.error(
                        "❌ Feeder '%s' found in config '%s' but none of its tags "
                        "%s map to a known transport protocol (zenoh/ros2/redis/uds). "
                        "Add one of these tags to the interface config entry.",
                        feeder_name,
                        json_path,
                        tags,
                    )
                    return None

                result = FeederResolverResult(
                    feeder_name=func_name,
                    protocol=protocol,
                    tags=tags,
                    file_types=entry.get("filetype", []),
                    display_name=entry.get("displayname", func_name),
                    description=entry.get("description", ""),
                    raw_entry=entry,
                    source_file=str(json_path),
                )
                logger.debug(
                    "✅ Feeder '%s' resolved → protocol='%s', tags=%s, files=%s "
                    "(source: %s)",
                    feeder_name,
                    protocol,
                    tags,
                    result.file_types,
                    json_path,
                )
                return result

        # ── No match found ────────────────────────────────────────────────
        suggestions = fuzzy_match(feeder_name, all_publisher_names, n=5, cutoff=0.4)
        suggestion_hint = (
            f" Did you mean one of: {suggestions}?" if suggestions
            else " No similar publisher names found in the searched configs."
        )
        searched_paths = list(FeederConfigResolver._iter_json_files(interface_paths))
        logger.error(
            "❌ Feeder '%s' not found as a publisher in any interface config.\n"
            "   Searched %d JSON file(s) across paths: %s\n"
            "   All known publishers: %s\n"
            "  %s",
            feeder_name,
            len(searched_paths),
            [str(p) for p in interface_paths],
            sorted(set(all_publisher_names)),
            suggestion_hint,
        )
        return None

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _iter_json_files(paths: Sequence[str | Path]):
        """Yield all ``.json`` files found under *paths* (recursively)."""
        for raw_path in paths:
            p = Path(raw_path)
            if not p.exists():
                logger.warning("FeederConfigResolver: path does not exist: %s", p)
                continue
            if p.is_file() and p.suffix == ".json":
                yield p
            elif p.is_dir():
                yield from p.rglob("*.json")

    @staticmethod
    def _load_json(path: Path) -> Optional[list[Any]]:
        """Load and return the JSON content of *path*.

        Returns ``None`` if the file cannot be parsed or is not a list.
        """
        try:
            with path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, list):
                return data
            # Some configs are dicts — wrap in list for uniform handling
            if isinstance(data, dict):
                return [data]
            return None
        except (json.JSONDecodeError, OSError) as exc:
            logger.warning("FeederConfigResolver: could not load '%s': %s", path, exc)
            return None

    @staticmethod
    def _resolve_protocol(tags: list[str], *, use_fallback: bool) -> Optional[str]:
        """Return the first recognised protocol from *tags*.

        :param tags: Tag list from the interface config entry.
        :param use_fallback: Fall back to the default chain when no tag
            matches.
        :return: Protocol string or ``None``.
        """
        for tag in tags:
            protocol = _TAG_TO_PROTOCOL.get(tag.lower())
            if protocol:
                return protocol

        if use_fallback:
            logger.debug(
                "FeederConfigResolver: no protocol tag recognised in %s, "
                "using fallback chain %s.",
                tags,
                _DEFAULT_FALLBACK,
            )
            return _DEFAULT_FALLBACK[0]  # zenoh

        return None
