"""Generic database persistence handler for VYRA feeders.

``ErrorLogDatabaseHandler`` writes structured records into **any** SQLAlchemy
table that is described by a :class:`~vyra_base.storage.tb_base.Base` subclass
and a ``field_definitions`` mapping.  The handler validates incoming data
against those definitions before writing, making it reusable across different
log tables.

The concrete default usage is writing ``ErrorEntry`` objects into the
``error_logs`` ring-buffer table (see
:class:`~vyra_base.storage.tb_error_log.ErrorLog`).

Ring-buffer rotation
--------------------
The table is kept at a maximum of ``max_rows`` entries (default 10 000).  A
module-level atomic counter tracks how many records have been written.  Each
new record receives ``slot_id = (counter % max_rows) + 1``, which means the
oldest slot is silently overwritten once the ring is full — **no DELETE is
ever issued**.

Handler lifecycle
-----------------
The handler is created *deactivated* when no :class:`~vyra_base.storage.db_access.DbAccess`
is available yet.  :py:meth:`configure` is called later (from
:py:meth:`~vyra_base.core.entity.VyraEntity._activate_errorfeed_db_handler`)
once ``db_access`` is ready, which activates the handler.
"""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Optional, Type

from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DbManipulator
from vyra_base.storage.tb_base import Base

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Field specification dataclass
# ---------------------------------------------------------------------------

@dataclass
class FieldSpec:
    """Describes a single column in the target log table.

    :param python_type: Expected Python type for the field value (used for
        validation).  Use ``Any`` to skip type checking.
    :param required: If ``True`` the field must be present in the record dict.
    :param nullable: If ``True`` the field value may be ``None`` even when
        required.
    """

    python_type: type = Any  # type: ignore[assignment]
    required: bool = True
    nullable: bool = False


# ---------------------------------------------------------------------------
# Handler
# ---------------------------------------------------------------------------

class ErrorLogDatabaseHandler(CommunicationHandler):
    """Generic database persistence handler for structured log records.

    The handler is intentionally *table-agnostic*: the target SQLAlchemy
    model and the field validation schema are injected at construction time,
    so the same class can be reused for different tables.

    Ring-buffer ID rotation keeps the table bounded at ``max_rows`` without
    ever running a DELETE.

    :param database: Async-capable :class:`~vyra_base.storage.db_access.DbAccess`
        instance.  If ``None`` the handler starts deactivated.
    :param model: SQLAlchemy declarative model that defines the target table.
    :param field_definitions: Dict mapping column names to
        :class:`FieldSpec` validation rules.
    :param max_rows: Maximum rows in the ring buffer.  Defaults to
        :data:`~vyra_base.storage.tb_error_log.ERROR_LOG_MAX_ROWS`.
    :param activated: Initial activation state.  Defaults to ``True`` when a
        ``database`` is provided, ``False`` otherwise.
    :param source: Label used in log messages.  Defaults to the class name.
    """

    __handlerName__: str = "ErrorLogDatabaseHandler"
    __doc__: str = "Generic database persistence handler for log records"

    def __init__(
        self,
        database: Optional[DbAccess],
        model: Type[Base],
        field_definitions: dict[str, FieldSpec],
        *,
        max_rows: int = 10_000,
        activated: Optional[bool] = None,
        source: str = "ErrorLogDatabaseHandler",
    ) -> None:
        super().__init__()
        self._model = model
        self._field_definitions = field_definitions
        self._max_rows = max_rows
        self._source = source
        self._counter: int = 0

        self._manipulator: Optional[DbManipulator] = None
        if database is not None:
            self._manipulator = DbManipulator(db_access=database, model=model)

        # Activation: explicit param > inferred from database availability
        if activated is not None:
            self.activated = activated
        else:
            self.activated = database is not None

    # ------------------------------------------------------------------
    # Late configuration (called when DbAccess becomes available)
    # ------------------------------------------------------------------

    def configure(self, database: DbAccess) -> None:
        """Wire a :class:`~vyra_base.storage.db_access.DbAccess` and activate.

        Meant to be called once by
        :py:meth:`~vyra_base.core.entity.VyraEntity._activate_errorfeed_db_handler`
        after storage has been initialised.

        :param database: Ready :class:`~vyra_base.storage.db_access.DbAccess`
            object.
        :type database: DbAccess
        """
        self._manipulator = DbManipulator(db_access=database, model=self._model)
        self.activated = True
        logger.info(
            "✅ %s configured and activated for table '%s'",
            self.__handlerName__,
            self._model.__tablename__,
        )

    # ------------------------------------------------------------------
    # IFeederHandler implementation
    # ------------------------------------------------------------------

    def get_protocol(self) -> str:
        """Return ``"database"``."""
        return "database"

    def is_available(self) -> bool:
        """Return ``True`` when a ``DbManipulator`` is configured."""
        return self._manipulator is not None

    async def dispatch(self, message: Any) -> None:
        """Validate and persist *message* to the database.

        The message is expected to be a ``dict`` (output of
        :meth:`~BaseFeeder._prepare_entry_for_publish`) or an object with a
        ``__dict__``.  The method calls :meth:`_build_record` to map the
        raw message to column values, validates them against
        :attr:`_field_definitions`, then writes with ID-rotation.

        :param message: Raw message dict or domain object.
        :type message: Any
        """
        if not self.activated:
            return
        if self._manipulator is None:
            logger.warning(
                "%s: no DbManipulator configured, skipping dispatch.", self.__handlerName__
            )
            return

        try:
            record = self._build_record(message)
            validated = self._validate(record)
            slot_id = self._next_slot_id()
            validated["id"] = slot_id
            await self._upsert(validated, slot_id)
        except Exception as exc:
            logger.error("❌ %s dispatch failed: %s", self.__handlerName__, exc)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _next_slot_id(self) -> int:
        """Compute the next ring-buffer slot ID.

        Uses modulo arithmetic so the ID cycles through ``1 … max_rows``
        forever without any external synchronisation:
        ``slot = (counter % max_rows) + 1``.

        :return: Slot ID in range ``[1, max_rows]``.
        :rtype: int
        """
        slot = (self._counter % self._max_rows) + 1
        self._counter += 1
        return slot

    def _build_record(self, message: Any) -> dict:
        """Convert a raw message to a flat column dict.

        Sub-classes or callers can override the mapping by subclassing and
        overriding this method.  The default implementation handles:

        * A ``dict`` that already contains the field keys (pass-through with
          ``__class__`` / ``_type`` keys stripped).
        * An object whose ``__dict__`` is used as the source.

        :param message: Raw message from the feeder's publish path.
        :type message: Any
        :return: Column dict ready for validation.
        :rtype: dict
        """
        if isinstance(message, dict):
            raw = {k: v for k, v in message.items() if not k.startswith("_")}
        elif hasattr(message, "__dict__"):
            raw = {k: v for k, v in vars(message).items() if not k.startswith("_")}
        else:
            raw = {"message": str(message)}

        # Map common ErrorEntry fields to the error_logs column names
        return {
            "error_id": raw.get("error_id") or (str(raw["uuid"]) if raw.get("uuid") else None),
            "occured_at": raw.get("timestamp") or raw.get("occured_at") or datetime.utcnow(),
            "error_code": raw.get("error_code") or raw.get("code"),
            "severity": raw.get("severity") or raw.get("level") or 0,
            "context_snap": _to_json_safe(raw.get("context_snap") or raw.get("miscellaneous")),
            "message": str(raw.get("message") or raw.get("description") or ""),
            "acknowledged": raw.get("acknowledged"),
        }

    def _validate(self, record: dict) -> dict:
        """Validate *record* against :attr:`_field_definitions`.

        :param record: Column dict to validate.
        :type record: dict
        :raises ValueError: If a required field is missing or a non-nullable
            field is ``None``.
        :raises TypeError: If a field value has the wrong Python type.
        :return: The original *record* (unchanged) on success.
        :rtype: dict
        """
        for col_name, spec in self._field_definitions.items():
            if spec.required and col_name not in record:
                raise ValueError(
                    f"{self.__handlerName__}: required field '{col_name}' missing in record"
                )
            value = record.get(col_name)
            if value is None:
                if not spec.nullable and spec.required:
                    raise ValueError(
                        f"{self.__handlerName__}: field '{col_name}' is non-nullable "
                        f"but received None"
                    )
                # None is acceptable for optional / nullable fields
                continue
            if spec.python_type is not Any and not isinstance(value, spec.python_type):
                # Attempt a lightweight coercion for common numeric promotions
                try:
                    record[col_name] = spec.python_type(value)
                except (TypeError, ValueError):
                    raise TypeError(
                        f"{self.__handlerName__}: field '{col_name}' expected "
                        f"{spec.python_type.__name__}, got {type(value).__name__}"
                    )
        return record

    async def _upsert(self, record: dict, slot_id: int) -> None:
        """INSERT or UPDATE the ring-buffer slot.

        First tries to retrieve the row with ``slot_id``.  If it exists the
        row is updated; if not a new row is inserted.  This avoids UPSERT
        dialect differences across SQLite / MySQL / PostgreSQL.

        :param record: Validated column dict including ``"id"``.
        :param slot_id: The pre-computed ring-buffer slot.
        """
        try:
            if not self._manipulator:
                raise RuntimeError("DbManipulator not configured")
            
            existing = await self._manipulator.get_by_id(slot_id)
            if existing.status == "success" and existing.value:
                # Row exists → update in-place
                update_data = {k: v for k, v in record.items() if k != "id"}
                await self._manipulator.update(
                    data=update_data, filters={"id": slot_id}
                )
            else:
                # Empty slot → insert
                await self._manipulator.add(record)
        except Exception as exc:
            logger.error("❌ %s _upsert(slot=%d) failed: %s", self.__handlerName__, slot_id, exc)

    # ------------------------------------------------------------------
    # Default field definitions
    # ------------------------------------------------------------------

    @classmethod
    def default_error_log_fields(cls) -> dict[str, FieldSpec]:
        """Return the standard field definitions for the ``error_logs`` table.

        :return: Dict of ``{column_name: FieldSpec}``.
        :rtype: dict[str, FieldSpec]
        """
        return {
            "error_id": FieldSpec(python_type=str, required=False, nullable=True),
            "occured_at": FieldSpec(python_type=datetime, required=True, nullable=False),
            "error_code": FieldSpec(python_type=int, required=False, nullable=True),
            "severity": FieldSpec(python_type=int, required=True, nullable=False),
            "context_snap": FieldSpec(python_type=Any, required=False, nullable=True),  # type: ignore[arg-type]
            "message": FieldSpec(python_type=str, required=True, nullable=False),
            "acknowledged": FieldSpec(python_type=Any, required=False, nullable=True),  # type: ignore[arg-type]
        }


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _to_json_safe(value: Any) -> Any:
    """Ensure *value* is JSON-serialisable for the ``context_snap`` column.

    * ``None`` → ``None`` (stored as SQL NULL)
    * ``dict`` / ``list`` / scalar → stored as-is (SQLAlchemy JSON type handles it)
    * Any other object → ``{"raw": str(value)}``

    :param value: Raw context value from the message.
    :return: JSON-safe value.
    """
    if value is None:
        return None
    if isinstance(value, (dict, list, str, int, float, bool)):
        return value
    try:
        json.dumps(value)
        return value
    except (TypeError, ValueError):
        return {"raw": str(value)}
