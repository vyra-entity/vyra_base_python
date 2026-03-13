"""SQLAlchemy model for the ``error_logs`` table.

The table is limited to :data:`ERROR_LOG_MAX_ROWS` rows using **ID
rotation**: the row ID is computed as ``(global_counter % MAX_ROWS) + 1``,
so the oldest row is silently overwritten once the ring is full.  This means
the physical table never grows beyond ``MAX_ROWS`` entries and no ``DELETE``
statement is ever needed.

Schema
------
.. code-block:: text

    error_logs
    ──────────────────────────────────────────────────
    id            INTEGER  PK (1 … ERROR_LOG_MAX_ROWS)
    occured_at    DATETIME NOT NULL
    error_code    INTEGER  NULL
    severity      INTEGER  NOT NULL   (ErrorEntry.ERROR_LEVEL)
    context_snap  JSON     NULL       (sensor values, variables, stacktrace …)
    message       TEXT     NOT NULL
    acknowledged  JSON     NULL       ({timestamp, module_name, user})
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Optional

from sqlalchemy import DateTime, Integer, JSON, Text
from sqlalchemy.orm import Mapped, mapped_column

from vyra_base.storage.tb_base import Base


#: Maximum number of rows kept in the error_logs ring buffer.
ERROR_LOG_MAX_ROWS: int = 10_000


class ErrorLog(Base):
    """SQLAlchemy model for the rotating ``error_logs`` ring-buffer table.

    Row IDs range from **1** to :data:`ERROR_LOG_MAX_ROWS`.  The ring
    wraps around so the oldest entry is always overwritten when full.

    :cvar ERROR_LOG_MAX_ROWS: Maximum rows before rotation begins.
    :ivar id: Row slot (1-based, computed via modulo rotation).
    :ivar error_id: UUID string of the error entry; used for acknowledgement lookup.
    :ivar occured_at: UTC timestamp of the event.
    :ivar error_code: Optional numeric error code.
    :ivar severity: Numeric severity level (``ErrorEntry.ERROR_LEVEL``).
    :ivar context_snap: Arbitrary JSON payload (sensor values, stacktrace …).
    :ivar message: Human-readable error description.
    :ivar acknowledged: JSON object set when a user acknowledges the entry
        (``{timestamp, module_name, user}``); ``None`` = unacknowledged.
    """

    __tablename__ = "error_logs"

    id: Mapped[int] = mapped_column(
        Integer,
        primary_key=True,
        autoincrement=False,
        comment="Ring-buffer slot (1 .. ERROR_LOG_MAX_ROWS)",
    )
    error_id: Mapped[Optional[str]] = mapped_column(
        Text,
        nullable=True,
        index=True,
        comment="UUID of the error entry for lookup and acknowledgement",
    )
    occured_at: Mapped[datetime] = mapped_column(
        DateTime,
        nullable=False,
    )
    error_code: Mapped[Optional[int]] = mapped_column(
        Integer,
        nullable=True,
    )
    severity: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
    )
    context_snap: Mapped[Optional[Any]] = mapped_column(
        JSON,
        nullable=True,
    )
    message: Mapped[str] = mapped_column(
        Text,
        nullable=False,
    )
    acknowledged: Mapped[Optional[Any]] = mapped_column(
        JSON,
        nullable=True,
    )
