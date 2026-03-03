from __future__ import annotations

import dataclasses
import hashlib
import json
import logging
import threading
import time
import asyncio
import functools
from dataclasses import dataclass
from inspect import iscoroutinefunction
from traceback import extract_tb
from types import TracebackType
from typing import Any, Callable, Optional, Literal, cast

from vyra_base.defaults.entries import ErrorEntry

logger = logging.getLogger(__name__)

ExecutionPoint = Literal["BEFORE", "DURING", "AFTER", "ALWAYS"]
ALLOWED_EXECUTION_POINTS: set[str] = {"BEFORE", "DURING", "AFTER", "ALWAYS"}


def _severity_to_error_level(severity: str) -> int:
    normalized = str(severity or "").upper()
    if normalized in {"DEBUG", "INFO", "MINOR", "MINOR_FAULT"}:
        return ErrorEntry.ERROR_LEVEL.MINOR_FAULT.value
    if normalized in {"WARNING", "WARN", "MAJOR", "MAJOR_FAULT"}:
        return ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value
    if normalized in {"ERROR", "CRITICAL", "CRITICAL_FAULT"}:
        return ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT.value
    if normalized in {"EMERGENCY", "EMERGENCY_FAULT"}:
        return ErrorEntry.ERROR_LEVEL.EMERGENCY_FAULT.value
    return ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value


def _to_serializable(value: Any) -> Any:
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return dataclasses.asdict(value)
    if isinstance(value, dict):
        return {str(k): _to_serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_to_serializable(v) for v in value]
    if hasattr(value, "__slots__"):
        result: dict[str, Any] = {}
        for slot in getattr(value, "__slots__", []):
            if hasattr(value, slot):
                result[slot] = _to_serializable(getattr(value, slot))
        if result:
            return result
    if hasattr(value, "__dict__"):
        return {str(k): _to_serializable(v) for k, v in vars(value).items()}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def build_message_signature(message: Any) -> str:
    normalized = _to_serializable(message)
    if isinstance(normalized, dict):
        for transient_key in ("timestamp", "uuid"):
            normalized.pop(transient_key, None)
    encoded = json.dumps(normalized, sort_keys=True, default=str)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


@dataclass
class DebounceHit:
    should_publish: bool
    duplicate_count: int = 0


class FeedDebouncer:
    def __init__(self, window_seconds: float = 5.0):
        self._window_seconds = window_seconds
        self._lock = threading.Lock()
        self._last_by_signature: dict[str, tuple[float, int]] = {}

    def evaluate(self, signature: str) -> DebounceHit:
        now = time.monotonic()
        with self._lock:
            prev = self._last_by_signature.get(signature)
            if prev is None:
                self._last_by_signature[signature] = (now, 0)
                return DebounceHit(should_publish=True, duplicate_count=0)

            last_ts, duplicate_count = prev
            if now - last_ts <= self._window_seconds:
                duplicate_count += 1
                self._last_by_signature[signature] = (now, duplicate_count)
                return DebounceHit(should_publish=False, duplicate_count=duplicate_count)

            self._last_by_signature[signature] = (now, 0)
            return DebounceHit(should_publish=True, duplicate_count=0)


@dataclass
class ConditionRule:
    name: str
    condition_function: Callable[[dict[str, Any]], bool]
    tag: str
    execution_point: ExecutionPoint = "ALWAYS"
    success_message: Optional[str] = None
    failure_message: Optional[str] = None


class FeedConditionRegistry:
    def __init__(self):
        self._rules: dict[str, ConditionRule] = {}

    def register(
        self,
        condition_function: Callable[[dict[str, Any]], bool],
        *,
        name: Optional[str] = None,
        tag: str = "news",
        execution_point: ExecutionPoint = "ALWAYS",
        success_message: Optional[str] = None,
        failure_message: Optional[str] = None,
    ) -> str:
        if iscoroutinefunction(condition_function):
            raise TypeError("Condition functions must be synchronous and return bool")
        normalized_execution_point = str(execution_point).upper()
        if normalized_execution_point not in ALLOWED_EXECUTION_POINTS:
            raise ValueError(
                f"execution_point '{execution_point}' is invalid. "
                f"Allowed values: {sorted(ALLOWED_EXECUTION_POINTS)}"
            )

        rule_name = name or getattr(condition_function, "__name__", "condition")
        self._rules[rule_name] = ConditionRule(
            name=rule_name,
            condition_function=condition_function,
            tag=tag,
            execution_point=cast(ExecutionPoint, normalized_execution_point),
            success_message=success_message,
            failure_message=failure_message,
        )
        return rule_name

    def unregister(self, name: str) -> bool:
        return self._rules.pop(name, None) is not None

    def evaluate(
        self,
        context: dict[str, Any],
        *,
        rule_names: Optional[list[str]] = None,
        tags: Optional[list[str]] = None,
        execution_point: Optional[ExecutionPoint] = None,
    ) -> list[tuple[str, str]]:
        outputs: list[tuple[str, str]] = []
        selected_names = set(rule_names) if rule_names else None
        selected_tags = set(tags) if tags else None
        selected_execution_point = str(execution_point).upper() if execution_point else None

        if selected_execution_point and selected_execution_point not in ALLOWED_EXECUTION_POINTS:
            raise ValueError(
                f"execution_point '{execution_point}' is invalid. "
                f"Allowed values: {sorted(ALLOWED_EXECUTION_POINTS)}"
            )

        for rule in self._rules.values():
            if selected_names is not None and rule.name not in selected_names:
                continue
            if selected_tags is not None and rule.tag not in selected_tags:
                continue
            if (
                selected_execution_point is not None
                and rule.execution_point not in {"ALWAYS", selected_execution_point}
            ):
                continue
            try:
                result = rule.condition_function(context)
                if not isinstance(result, bool):
                    raise TypeError(
                        f"Condition '{rule.name}' returned {type(result)}; expected bool"
                    )

                if result and rule.success_message:
                    outputs.append((rule.tag, rule.success_message))
                if (not result) and rule.failure_message:
                    outputs.append((rule.tag, rule.failure_message))
            except Exception as exc:
                logger.warning("Condition '%s' evaluation failed: %s", rule.name, exc)
        return outputs


def resolve_entity_from_call(
    args: tuple[Any, ...],
    kwargs: dict[str, Any],
    explicit_entity: Any = None,
) -> Any:
    if explicit_entity is not None:
        return explicit_entity
    if kwargs.get("entity") is not None:
        return kwargs.get("entity")
    if args:
        candidate = args[0]
        if hasattr(candidate, "entity"):
            return getattr(candidate, "entity")
        if hasattr(candidate, "error_feeder") or hasattr(candidate, "news_feeder"):
            return candidate
    return None


def describe_exception(exc: Exception) -> dict[str, Any]:
    tb: Optional[TracebackType] = exc.__traceback__
    frames = extract_tb(tb) if tb is not None else []
    frame = frames[-1] if frames else None
    origin = f"{frame.filename}:{frame.lineno} in {frame.name}" if frame else "unknown"
    return {
        "exception_type": exc.__class__.__name__,
        "message": str(exc),
        "origin": origin,
        "traceback": "".join(__import__("traceback").format_exception(exc.__class__, exc, tb)),
    }


class FeedTracker:
    """Decorator factory for monitored function execution and feeder dispatch."""

    def __init__(self, feeder: Optional[Any] = None):
        self._feeder = feeder

    def monitor(
        self,
        *,
        tag: str = "error",
        label: Optional[str] = None,
        severity: str = "WARNING",
        entity: Any = None,
        during_interval_seconds: float = 0.05,
    ) -> Callable:
        normalized_tag = str(tag or "").strip().lower() or "error"

        def decorator(func: Callable) -> Callable:
            if asyncio.iscoroutinefunction(func):
                @functools.wraps(func)
                async def async_wrapper(*args, **kwargs):
                    entity_obj, primary_feeder, error_feeder, news_feeder = self._resolve_feeders(
                        tag=normalized_tag,
                        args=args,
                        kwargs=kwargs,
                        explicit_entity=entity,
                    )
                    context_base = self._build_runtime_context(
                        args=args,
                        kwargs=kwargs,
                        func=func,
                        tag=normalized_tag,
                        label=label,
                        severity=severity,
                        entity_obj=entity_obj,
                    )

                    await self._evaluate_runtime_conditions_async(
                        primary_feeder=primary_feeder,
                        error_feeder=error_feeder,
                        news_feeder=news_feeder,
                        entity_obj=entity_obj,
                        context=self._with_execution_point(context_base, "BEFORE"),
                        execution_point="BEFORE",
                        severity=severity,
                    )

                    stop_during = threading.Event()
                    during_thread = self._start_during_monitor_thread(
                        stop_event=stop_during,
                        interval_seconds=during_interval_seconds,
                        primary_feeder=primary_feeder,
                        error_feeder=error_feeder,
                        news_feeder=news_feeder,
                        entity_obj=entity_obj,
                        context_supplier=lambda: self._with_execution_point(
                            self._build_runtime_context(
                                args=args,
                                kwargs=kwargs,
                                func=func,
                                tag=normalized_tag,
                                label=label,
                                severity=severity,
                                entity_obj=entity_obj,
                            ),
                            "DURING",
                        ),
                        severity=severity,
                    )

                    try:
                        result = await func(*args, **kwargs)
                        context_after = self._with_execution_point(
                            self._build_runtime_context(
                                args=args,
                                kwargs=kwargs,
                                func=func,
                                tag=normalized_tag,
                                label=label,
                                severity=severity,
                                entity_obj=entity_obj,
                                result=result,
                            ),
                            "AFTER",
                        )
                        await self._evaluate_runtime_conditions_async(
                            primary_feeder=primary_feeder,
                            error_feeder=error_feeder,
                            news_feeder=news_feeder,
                            entity_obj=entity_obj,
                            context=context_after,
                            execution_point="AFTER",
                            severity=severity,
                        )
                        return result
                    except Exception as exc:
                        context_error = self._with_execution_point(
                            self._build_runtime_context(
                                args=args,
                                kwargs=kwargs,
                                func=func,
                                tag=normalized_tag,
                                label=label,
                                severity=severity,
                                entity_obj=entity_obj,
                                exception=exc,
                            ),
                            "AFTER",
                        )
                        await self._handle_exception(
                            exc,
                            primary_feeder=primary_feeder,
                            error_feeder=error_feeder,
                            news_feeder=news_feeder,
                            entity_obj=entity_obj,
                            context=context_error,
                            severity=severity,
                            emit_exception_payload=(normalized_tag == "error"),
                        )
                        raise
                    finally:
                        stop_during.set()
                        if during_thread is not None:
                            during_thread.join(timeout=0.2)

                return async_wrapper

            @functools.wraps(func)
            def sync_wrapper(*args, **kwargs):
                entity_obj, primary_feeder, error_feeder, news_feeder = self._resolve_feeders(
                    tag=normalized_tag,
                    args=args,
                    kwargs=kwargs,
                    explicit_entity=entity,
                )
                context_base = self._build_runtime_context(
                    args=args,
                    kwargs=kwargs,
                    func=func,
                    tag=normalized_tag,
                    label=label,
                    severity=severity,
                    entity_obj=entity_obj,
                )

                self._evaluate_runtime_conditions_sync(
                    primary_feeder=primary_feeder,
                    error_feeder=error_feeder,
                    news_feeder=news_feeder,
                    entity_obj=entity_obj,
                    context=self._with_execution_point(context_base, "BEFORE"),
                    execution_point="BEFORE",
                    severity=severity,
                )

                stop_during = threading.Event()
                during_thread = self._start_during_monitor_thread(
                    stop_event=stop_during,
                    interval_seconds=during_interval_seconds,
                    primary_feeder=primary_feeder,
                    error_feeder=error_feeder,
                    news_feeder=news_feeder,
                    entity_obj=entity_obj,
                    context_supplier=lambda: self._with_execution_point(
                        self._build_runtime_context(
                            args=args,
                            kwargs=kwargs,
                            func=func,
                            tag=normalized_tag,
                            label=label,
                            severity=severity,
                            entity_obj=entity_obj,
                        ),
                        "DURING",
                    ),
                    severity=severity,
                )

                try:
                    result = func(*args, **kwargs)
                    context_after = self._with_execution_point(
                        self._build_runtime_context(
                            args=args,
                            kwargs=kwargs,
                            func=func,
                            tag=normalized_tag,
                            label=label,
                            severity=severity,
                            entity_obj=entity_obj,
                            result=result,
                        ),
                        "AFTER",
                    )
                    self._evaluate_runtime_conditions_sync(
                        primary_feeder=primary_feeder,
                        error_feeder=error_feeder,
                        news_feeder=news_feeder,
                        entity_obj=entity_obj,
                        context=context_after,
                        execution_point="AFTER",
                        severity=severity,
                    )
                    return result
                except Exception as exc:
                    context_error = self._with_execution_point(
                        self._build_runtime_context(
                            args=args,
                            kwargs=kwargs,
                            func=func,
                            tag=normalized_tag,
                            label=label,
                            severity=severity,
                            entity_obj=entity_obj,
                            exception=exc,
                        ),
                        "AFTER",
                    )
                    self._handle_exception_sync(
                        exc,
                        primary_feeder=primary_feeder,
                        error_feeder=error_feeder,
                        news_feeder=news_feeder,
                        entity_obj=entity_obj,
                        context=context_error,
                        severity=severity,
                        emit_exception_payload=(normalized_tag == "error"),
                    )
                    raise
                finally:
                    stop_during.set()
                    if during_thread is not None:
                        during_thread.join(timeout=0.2)

            return sync_wrapper

        return decorator

    def _resolve_feeders(
        self,
        *,
        tag: str,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        explicit_entity: Any,
    ):
        entity_obj = resolve_entity_from_call(args, kwargs, explicit_entity)
        if self._feeder is not None:
            primary_feeder = self._feeder
        elif entity_obj is not None:
            primary_feeder = getattr(entity_obj, f"{tag}_feeder", None)
        else:
            primary_feeder = None

        error_feeder = primary_feeder if tag == "error" else None
        if error_feeder is None and entity_obj is not None:
            error_feeder = getattr(entity_obj, "error_feeder", None)
        news_feeder = getattr(entity_obj, "news_feeder", None) if entity_obj is not None else None
        return entity_obj, primary_feeder, error_feeder, news_feeder

    def _build_runtime_context(
        self,
        *,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        func: Callable,
        tag: str,
        label: Optional[str],
        severity: str,
        entity_obj: Any,
        result: Any = None,
        exception: Optional[Exception] = None,
    ) -> dict[str, Any]:
        self_obj = args[0] if args else None
        context: dict[str, Any] = {
            "tag": tag,
            "label": label,
            "severity": severity,
            "function": getattr(func, "__qualname__", repr(func)),
            "args": args,
            "kwargs": kwargs,
            "entity": entity_obj,
            "self": self_obj,
        }
        if result is not None:
            context["result"] = result
        if exception is not None:
            context["exception"] = describe_exception(exception)
        return context

    def _with_execution_point(
        self,
        context: dict[str, Any],
        execution_point: ExecutionPoint,
    ) -> dict[str, Any]:
        updated = dict(context)
        updated["execution_point"] = execution_point
        return updated

    def _start_during_monitor_thread(
        self,
        *,
        stop_event: threading.Event,
        interval_seconds: float,
        primary_feeder: Any,
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        context_supplier: Callable[[], dict[str, Any]],
        severity: str,
    ) -> Optional[threading.Thread]:
        if primary_feeder is None:
            return None

        def _runner() -> None:
            while not stop_event.is_set():
                context = context_supplier()
                self._evaluate_runtime_conditions_sync(
                    primary_feeder=primary_feeder,
                    error_feeder=error_feeder,
                    news_feeder=news_feeder,
                    entity_obj=entity_obj,
                    context=context,
                    execution_point="DURING",
                    severity=severity,
                )
                stop_event.wait(interval_seconds)

        thread = threading.Thread(target=_runner, name="FeedTrackerDuring", daemon=True)
        thread.start()
        return thread

    async def _handle_exception(
        self,
        exc: Exception,
        *,
        primary_feeder: Any,
        severity: str,
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        context: dict[str, Any],
        emit_exception_payload: bool,
    ) -> None:
        if emit_exception_payload and error_feeder is None:
            logger.warning(
                "feed_tracker.monitor ignored for %s: no entity.error_feeder found",
                context.get("function", "unknown"),
            )
        elif emit_exception_payload:
            details = describe_exception(exc)
            payload = {
                "error_code": 0,
                "description": f"{details['exception_type']}: {details['message']}",
                "solution": context.get("label", "") or "",
                "miscellaneous": (
                    f"tag={context.get('tag')};severity={severity};origin={details['origin']};"
                    f"traceback={details['traceback']}"
                ),
                "level": _severity_to_error_level(severity),
            }
            await error_feeder.feed(payload)

        await self._evaluate_runtime_conditions_async(
            primary_feeder=primary_feeder,
            error_feeder=error_feeder,
            news_feeder=news_feeder,
            entity_obj=entity_obj,
            context=context,
            execution_point="AFTER",
            severity=severity,
        )

    def _handle_exception_sync(
        self,
        exc: Exception,
        *,
        primary_feeder: Any,
        severity: str,
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        context: dict[str, Any],
        emit_exception_payload: bool,
    ) -> None:
        if emit_exception_payload and error_feeder is None:
            logger.warning(
                "feed_tracker.monitor ignored for %s: no entity.error_feeder found",
                context.get("function", "unknown"),
            )
        elif emit_exception_payload:
            details = describe_exception(exc)
            payload = {
                "error_code": 0,
                "description": f"{details['exception_type']}: {details['message']}",
                "solution": context.get("label", "") or "",
                "miscellaneous": (
                    f"tag={context.get('tag')};severity={severity};origin={details['origin']};"
                    f"traceback={details['traceback']}"
                ),
                "level": _severity_to_error_level(severity),
            }
            error_feeder.feed_sync(payload)

        self._evaluate_runtime_conditions_sync(
            primary_feeder=primary_feeder,
            error_feeder=error_feeder,
            news_feeder=news_feeder,
            entity_obj=entity_obj,
            context=context,
            execution_point="AFTER",
            severity=severity,
        )

    async def _evaluate_runtime_conditions_async(
        self,
        *,
        primary_feeder: Any,
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        context: dict[str, Any],
        execution_point: ExecutionPoint,
        severity: str,
    ) -> None:
        if primary_feeder is None or not hasattr(primary_feeder, "evaluate_conditions"):
            return

        try:
            outputs = primary_feeder.evaluate_conditions(context, execution_point=execution_point)
        except TypeError:
            outputs = primary_feeder.evaluate_conditions(context)
        await self._dispatch_condition_messages(
            outputs=outputs,
            error_feeder=error_feeder,
            news_feeder=news_feeder,
            entity_obj=entity_obj,
            severity=severity,
        )

    def _evaluate_runtime_conditions_sync(
        self,
        *,
        primary_feeder: Any,
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        context: dict[str, Any],
        execution_point: ExecutionPoint,
        severity: str,
    ) -> None:
        if primary_feeder is None or not hasattr(primary_feeder, "evaluate_conditions"):
            return

        try:
            outputs = primary_feeder.evaluate_conditions(context, execution_point=execution_point)
        except TypeError:
            outputs = primary_feeder.evaluate_conditions(context)
        self._dispatch_condition_messages_sync(
            outputs=outputs,
            error_feeder=error_feeder,
            news_feeder=news_feeder,
            entity_obj=entity_obj,
            severity=severity,
        )

    async def _dispatch_condition_messages(
        self,
        *,
        outputs: list[tuple[str, str]],
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        severity: str,
    ) -> None:
        for message_tag, message in outputs:
            if message_tag == "error":
                if error_feeder is None:
                    logger.warning("No error_feeder found for condition output")
                    continue
                await error_feeder.feed({
                    "description": message,
                    "level": _severity_to_error_level(severity),
                })
                continue

            if message_tag == "news" and news_feeder is not None:
                await news_feeder.feed(message)
                continue

            feeder_candidate = None
            if entity_obj is not None:
                feeder_candidate = getattr(entity_obj, f"{message_tag}_feeder", None)
            if feeder_candidate is not None and hasattr(feeder_candidate, "feed"):
                await feeder_candidate.feed(message)
            else:
                logger.warning("No feeder found for condition tag '%s'", message_tag)

    def _dispatch_condition_messages_sync(
        self,
        *,
        outputs: list[tuple[str, str]],
        error_feeder: Any,
        news_feeder: Any,
        entity_obj: Any,
        severity: str,
    ) -> None:
        for message_tag, message in outputs:
            if message_tag == "error":
                if error_feeder is None:
                    logger.warning("No error_feeder found for condition output")
                    continue
                error_feeder.feed_sync({
                    "description": message,
                    "level": _severity_to_error_level(severity),
                })
                continue

            if message_tag == "news" and news_feeder is not None:
                if hasattr(news_feeder, "feed_sync"):
                    news_feeder.feed_sync(message)
                else:
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        loop.create_task(news_feeder.feed(message))
                    else:
                        loop.run_until_complete(news_feeder.feed(message))
                continue

            feeder_candidate = None
            if entity_obj is not None:
                feeder_candidate = getattr(entity_obj, f"{message_tag}_feeder", None)
            if feeder_candidate is not None:
                if hasattr(feeder_candidate, "feed_sync"):
                    feeder_candidate.feed_sync(message)
                elif hasattr(feeder_candidate, "feed"):
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        loop.create_task(feeder_candidate.feed(message))
                    else:
                        loop.run_until_complete(feeder_candidate.feed(message))
            else:
                logger.warning("No feeder found for condition tag '%s'", message_tag)


feed_tracker = FeedTracker()