"""
Database persistence handler for VYRA feeders.

``DBCommunicationHandler`` writes structured log/feed records to an
async-capable database backend.  Any object that exposes an
``async write(record: dict) -> None`` coroutine is accepted as *database*
(e.g. a Redis client, an async SQLite connection, a custom InfluxDB writer).

Record schema
-------------
Every ``dispatch`` call persists the following fields:

.. code-block:: json

    {
        "timestamp":  "<ISO-8601 UTC>",
        "level":      "INFO",
        "source":     "<feeder/initiator name>",
        "message":    "<str(message)>",
        "metadata":   {}
    }

The schema is intentionally flat so it can be stored in Redis hashes, SQL
tables, InfluxDB measurements, or a time-series database without
additional transformation.
"""

from __future__ import annotations

import asyncio
import datetime
import logging
from logging import LogRecord
from typing import Any, Protocol, runtime_checkable

from vyra_base.com.handler.communication import CommunicationHandler

logger = logging.getLogger(__name__)


@runtime_checkable
class DatabaseWriter(Protocol):
    """Structural protocol for database backends.

    Any object that implements ``async write(record: dict) -> None``
    qualifies — no inheritance required.
    """

    async def write(self, record: dict) -> None:  # noqa: D102
        ...


class DBCommunicationHandler(CommunicationHandler):
    """Feeder handler that persists messages to a database backend.

    Accepts **any** object that implements the :class:`DatabaseWriter`
    protocol (``async write(dict) → None``).  This makes the handler
    independent of a specific DB technology.

    :cvar __handlerName__: Identifies this handler as ``"DatabaseHandler"``.
    :param database: Async-capable database writer implementing
        :class:`DatabaseWriter`.
    :type database: DatabaseWriter
    :param source: Label used as ``"source"`` in the persisted record.
        Defaults to ``"DBCommunicationHandler"``.
    :type source: str, optional
    """

    __handlerName__: str = 'DatabaseHandler'
    __doc__: str = 'Database persistence handler'

    def __init__(self, database: Any, source: str = 'DBCommunicationHandler'):
        super().__init__()
        self.database = database
        self._source = source
        try:
            self._loop: asyncio.AbstractEventLoop = asyncio.get_event_loop()
        except RuntimeError:
            self._loop = asyncio.new_event_loop()

    # ------------------------------------------------------------------
    # IFeederHandler implementation
    # ------------------------------------------------------------------

    def get_protocol(self) -> str:
        """Return ``"database"``."""
        return "database"

    def is_available(self) -> bool:
        """Return ``True`` if a database writer is configured."""
        return self.database is not None

    async def dispatch(self, message: Any) -> None:
        """Persist *message* to the database with a structured schema.

        :param message: Domain object or any value to persist.  Converted
            to string for the ``"message"`` field.  If the object has a
            ``__dict__`` attribute the full dict is stored in
            ``"metadata"``.
        :type message: Any
        """
        if self.database is None:
            logger.warning("DBCommunicationHandler: no database configured, skipping.")
            return

        metadata: dict = {}
        if hasattr(message, '__dict__'):
            try:
                metadata = {k: str(v) for k, v in vars(message).items()}
            except Exception:
                pass

        record: dict = {
            "timestamp": datetime.datetime.utcnow().isoformat() + "Z",
            "level": "INFO",
            "source": self._source,
            "message": str(message),
            "metadata": metadata,
        }
        await self._write(record)

    # ------------------------------------------------------------------
    # logging.Handler bridge — formats the LogRecord as a plain string
    # ------------------------------------------------------------------

    def emit(self, record: LogRecord) -> None:  # type: ignore[override]
        """Persist the formatted log record string to the database.

        Unlike transport handlers, the DB handler stores the *formatted*
        string so that plain Python log records are human-readable.

        :param record: Python log record to persist.
        :type record: LogRecord
        """
        try:
            log_entry = self.format(record)
            db_record: dict = {
                "timestamp": datetime.datetime.utcnow().isoformat() + "Z",
                "level": record.levelname,
                "source": record.name,
                "message": log_entry,
                "metadata": {
                    "lineno": record.lineno,
                    "filename": record.filename,
                    "funcName": record.funcName,
                },
            }
            if self._loop.is_running():
                self._loop.create_task(self._write(db_record))
            else:
                self._loop.run_until_complete(self._write(db_record))
        except Exception:
            self.handleError(record)

    async def _write(self, record: dict) -> None:
        """Internal coroutine that delegates to ``database.write``."""
        try:
            await self.database.write(record)
        except Exception as exc:
            logger.error("DBCommunicationHandler: write failed: %s", exc)