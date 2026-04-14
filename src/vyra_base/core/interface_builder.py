"""
InterfaceBuilder — Static helper for creating transport interfaces.

Centralises protocol-dispatch logic that was previously spread throughout
``VyraEntity.set_interfaces``.  Each ``create_*`` classmethod accepts a
:class:`~vyra_base.defaults.entries.FunctionConfigEntry` and creates the
appropriate transport handles for **all** protocols the entry's *tags*
request (or for all available protocols when *tags* is empty).

Tag semantics
-------------
- **Empty tags** → register on every available / applicable protocol
  (backward-compatible default).
- **Non-empty tags** → register *only* on the explicitly listed protocols.

Example::

    # tags=["ros2"]      → ROS2 only
    # tags=["zenoh"]     → Zenoh only
    # tags=["ros2","zenoh"] → both
    # tags=[]            → all applicable (backward compat)
"""
from __future__ import annotations

import logging
from typing import Any

from vyra_base.com import InterfaceFactory, ProtocolType
from vyra_base.com.providers.provider_registry import ProviderRegistry
from vyra_base.defaults.entries import (
    FunctionConfigEntry,
    FunctionConfigTags,
)

logger = logging.getLogger(__name__)


class InterfaceBuilder:
    """
    Static factory that creates transport interfaces for a
    :class:`~vyra_base.defaults.entries.FunctionConfigEntry`.

    All methods are classmethods — no instance state is held.
    """

    # ── Tag helpers ───────────────────────────────────────────────────────────

    @classmethod
    def _wants(cls, tags: list[FunctionConfigTags], protocol: FunctionConfigTags) -> bool:
        """Return True if *tags* is empty (default to all) or contains *protocol*."""
        return not tags or protocol in tags

    @classmethod
    def wants_ros2(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.ROS2)

    @classmethod
    def wants_zenoh(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.ZENOH)

    @classmethod
    def wants_redis(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.REDIS)

    @classmethod
    def wants_uds(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.UDS)

    @classmethod
    def has_ros2_type(cls, interfacetypes: Any) -> bool:
        """Return True if *interfacetypes* contains at least one actual Python class (ROS2 type).

        When a proto-only interface is configured, all entries in the list are strings.
        Only if at least one entry is a resolved class should we attempt ROS2 creation.
        """
        if interfacetypes is None:
            return False
        if isinstance(interfacetypes, list):
            return any(not isinstance(t, str) for t in interfacetypes)
        return not isinstance(interfacetypes, str)

    # ── Service (request / response) ─────────────────────────────────────────

    @classmethod
    async def create_service(
        cls,
        setting: FunctionConfigEntry,
        callbacks: dict[str, Any] | None,
        node: Any,
        ros2_available: bool,
        registry: ProviderRegistry | None = None,
    ) -> None:
        """
        Create service servers for all protocols the entry's tags request.

        :param setting: Interface configuration entry.
        :param callbacks: Dict with key ``'response'`` holding the callback.
        :param node: ROS2 node (may be ``None`` when ROS2 is unavailable).
        :param ros2_available: Whether ROS2 is up and a node is present.
        :param registry: Optional pre-created :class:`ProviderRegistry` instance.
                         A new one is created when *None* (avoids repeated look-ups
                         when called for many interfaces in one loop).
        """
        if registry is None:
            registry = ProviderRegistry()

        name = setting.functionname
        response_cb = callbacks.get("response") if callbacks else None
        tags = setting.tags

        # ROS2
        if ros2_available and cls.wants_ros2(tags) and cls.has_ros2_type(setting.interfacetypes):
            try:
                await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ROS2],
                    service_type=setting.interfacetypes,
                    node=node,
                )
                logger.info(f"✅ ROS2 service created: {name}")
            except Exception as exc:
                logger.warning(f"⚠️ ROS2 service '{name}' failed: {exc}")

        # Zenoh
        if registry.is_available(ProtocolType.ZENOH) and cls.wants_zenoh(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ZENOH],
                    service_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ Zenoh service created: {name}")
                else:
                    logger.debug(
                        f"⏳ Zenoh service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Zenoh service '{name}' failed: {exc}")

        # Redis
        if registry.is_available(ProtocolType.REDIS) and cls.wants_redis(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.REDIS],
                    service_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ Redis service created: {name}")
                else:
                    logger.debug(
                        f"⏳ Redis service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Redis service '{name}' failed: {exc}")

        # UDS
        if registry.is_available(ProtocolType.UDS) and cls.wants_uds(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.UDS],
                    service_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ UDS service created: {name}")
                else:
                    logger.debug(
                        f"⏳ UDS service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ UDS service '{name}' failed: {exc}")

    # ── Action server ─────────────────────────────────────────────────────────

    @classmethod
    async def create_action(
        cls,
        setting: FunctionConfigEntry,
        callbacks: dict[str, Any] | None,
        node: Any,
        ros2_available: bool,
        registry: ProviderRegistry | None = None,
    ) -> None:
        """
        Create action servers for all protocols the entry's tags request.

        Tries each tagged protocol in order (ROS2, Zenoh, Redis, UDS).
        A failure on one protocol is logged as a warning and the next is attempted.

        :raises ValueError: When callbacks are missing (required for action servers).
        """
        if registry is None:
            registry = ProviderRegistry()

        name = setting.functionname

        if not callbacks:
            msg = (
                f"❌ ActionServer '{name}' has no callbacks defined. "
                "Use @remote_actionServer.on_goal/on_cancel/execute decorators."
            )
            logger.error(msg)
            raise ValueError(msg)

        tags = setting.tags
        on_goal = callbacks.get("on_goal")
        on_cancel = callbacks.get("on_cancel")
        execute = callbacks.get("execute")

        logger.debug(
            f"ActionServer '{name}' callbacks — "
            f"on_goal={bool(on_goal)}, "
            f"on_cancel={bool(on_cancel)}, "
            f"execute={bool(execute)}"
        )

        # ROS2
        if ros2_available and cls.wants_ros2(tags) and cls.has_ros2_type(setting.interfacetypes):
            try:
                await InterfaceFactory.create_action_server(
                    name=name,
                    handle_goal_request=on_goal,
                    handle_cancel_request=on_cancel,
                    execution_callback=execute,
                    protocols=[ProtocolType.ROS2],
                    action_type=setting.interfacetypes,
                    node=node,
                    namespace=setting.namespace,
                )
                logger.info(f"✅ ROS2 action server created: {name}")
            except Exception as exc:
                logger.warning(f"⚠️ ROS2 action server '{name}' failed: {exc}")

        # Zenoh
        if registry.is_available(ProtocolType.ZENOH) and cls.wants_zenoh(tags):
            try:
                result = await InterfaceFactory.create_action_server(
                    name=name,
                    handle_goal_request=on_goal,
                    handle_cancel_request=on_cancel,
                    execution_callback=execute,
                    protocols=[ProtocolType.ZENOH],
                    action_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ Zenoh action server created: {name}")
                else:
                    logger.debug(
                        f"⏳ Zenoh action server '{name}' pending "
                        f"— no callback bound yet"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Zenoh action server '{name}' failed: {exc}")

        # Redis
        if registry.is_available(ProtocolType.REDIS) and cls.wants_redis(tags):
            try:
                result = await InterfaceFactory.create_action_server(
                    name=name,
                    handle_goal_request=on_goal,
                    handle_cancel_request=on_cancel,
                    execution_callback=execute,
                    protocols=[ProtocolType.REDIS],
                    action_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ Redis action server created: {name}")
                else:
                    logger.debug(
                        f"⏳ Redis action server '{name}' pending "
                        f"— no callback bound yet"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Redis action server '{name}' failed: {exc}")

        # UDS
        if registry.is_available(ProtocolType.UDS) and cls.wants_uds(tags):
            try:
                result = await InterfaceFactory.create_action_server(
                    name=name,
                    handle_goal_request=on_goal,
                    handle_cancel_request=on_cancel,
                    execution_callback=execute,
                    protocols=[ProtocolType.UDS],
                    action_type=setting.interfacetypes,
                    namespace=setting.namespace,
                )
                if result is not None:
                    logger.info(f"✅ UDS action server created: {name}")
                else:
                    logger.debug(
                        f"⏳ UDS action server '{name}' pending "
                        f"— no callback bound yet"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ UDS action server '{name}' failed: {exc}")

    # ── Publisher (one-way message) ───────────────────────────────────────────

    @classmethod
    async def create_publisher(
        cls,
        setting: FunctionConfigEntry,
        node: Any,
        ros2_available: bool,
    ) -> None:
        """
        Create publishers for all protocols the entry's tags request.
        """
        name = setting.functionname
        tags = setting.tags

        # Periodic config
        periodic = setting.periodic is not None

        if setting.periodic is None:
             periodic_caller = None
        else:
            periodic_caller = setting.periodic.caller

        if setting.periodic is None:
            periodic_interval = None
        else:
            periodic_interval = setting.periodic.interval

        # ROS2
        if ros2_available and cls.wants_ros2(tags) and cls.has_ros2_type(setting.interfacetypes):
            try:
                await InterfaceFactory.create_publisher(
                    name=name,
                    protocols=[ProtocolType.ROS2],
                    message_type=setting.interfacetypes,
                    node=node,
                    is_publisher=True,
                    qos_profile=setting.qosprofile,
                    description=setting.description,
                    periodic=periodic,
                    interval_time=periodic_interval,
                    periodic_caller=periodic_caller,
                )
                logger.info(f"✅ ROS2 publisher created: {name}")
            except Exception as exc:
                logger.warning(
                    f"⚠️ ROS2 publisher '{name}' could not be created (non-fatal): {exc}"
                )
        elif ros2_available and cls.wants_ros2(tags):
            logger.debug(f"⏭️ Skipping ROS2 publisher '{name}': no resolved ROS2 type in interfacetypes")
        else:
            logger.debug(f"⏭️ Skipping ROS2 publisher '{name}': tags={tags}")

    # ── Late-binding upgrade (callback added after initial registration) ───────

    @classmethod
    async def upgrade_service(
        cls,
        setting: FunctionConfigEntry,
        new_callbacks: dict[str, Any],
        registry: ProviderRegistry | None = None,
    ) -> None:
        """
        Upgrade a previously registered service that had no callback with a new one.

        Called when a decorated callback is bound after ``set_interfaces`` ran
        (two-phase / blueprint initialisation pattern).
        """
        if registry is None:
            registry = ProviderRegistry()

        name = setting.functionname
        response_cb = new_callbacks.get("response")

        if registry.is_available(ProtocolType.ZENOH):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ZENOH],
                    service_type=setting.interfacetypes,
                )
                if result is not None:
                    logger.info(f"✅ Zenoh server upgraded: {name}")
                else:
                    logger.warning(f"⚠️ Zenoh upgrade returned None for '{name}'")
            except Exception as exc:
                logger.warning(f"⚠️ Failed to upgrade Zenoh server '{name}': {exc}")
