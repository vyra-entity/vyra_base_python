"""
In-memory log ring-buffer handler for V.Y.R.A. modules.

The :class:`VyraLogHandler` captures recent log records into a fixed-size
deque.  It is installed by :class:`~vyra_base.core.entity.VyraEntity` on
the root logger during initialisation and the buffer is exposed via the
``get_log_history`` Zenoh service so the modulemanager dashboard can poll
log output without a dedicated log-stream topic.
"""

from __future__ import annotations

import collections
import logging
from datetime import datetime


__all__ = ["VyraLogHandler"]


class VyraLogHandler(logging.Handler):
    """
    Lightweight in-memory ring-buffer that captures recent log records.

    Each record is stored as a plain :class:`dict` so it can be JSON-serialised
    without extra effort.  The buffer holds at most *capacity* entries; older
    entries are dropped automatically (FIFO).

    Usage::

        handler = VyraLogHandler(capacity=1000)
        handler.setLevel(logging.DEBUG)
        logging.getLogger().addHandler(handler)

        # Later, retrieve entries:
        recent = handler.get_recent(limit=100)

    :param capacity: Maximum number of log records to keep in the ring-buffer.
    :type capacity: int
    :param max_message_length: Maximum length of a single log message (default 10000 chars).
    :type max_message_length: int
    """

    def __init__(self, capacity: int = 1000, max_message_length: int = 10000) -> None:
        super().__init__()
        self._records: collections.deque = collections.deque(maxlen=capacity)
        self._max_message_length: int = max_message_length

    # ------------------------------------------------------------------
    # logging.Handler interface
    # ------------------------------------------------------------------

    def emit(self, record: logging.LogRecord) -> None:
        """Append a formatted log record to the ring-buffer."""
        try:
            # Format the message (use formatter if available, else getMessage())
            message = self.format(record) if self.formatter else record.getMessage()
            
            # Truncate extremely long messages to prevent memory overflow and browser freezing
            if len(message) > self._max_message_length:
                message = message[:self._max_message_length] + f"... [TRUNCATED: {len(message) - self._max_message_length} chars]"
            
            self._records.append({
                "level": record.levelname,
                "message": message,
                "logger_name": record.name,
                "timestamp": datetime.fromtimestamp(record.created).isoformat(),
                # Use created * 1000 as a monotone sequence number (ms precision).
                # Consumers can use this field to de-duplicate when polling.
                "seq": record.created * 1_000,
            })
        except Exception:  # pragma: no cover
            self.handleError(record)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_recent(self, limit: int = 200) -> list:
        """
        Return up to *limit* most-recent log entries, ordered oldest-first.

        :param limit: Maximum number of entries to return.  ``0`` or a negative
            value returns the entire buffer.
        :type limit: int
        :returns: List of dicts with keys ``level``, ``message``,
            ``logger_name``, ``timestamp``, ``seq``.
        :rtype: list[dict]
        """
        buf = list(self._records)
        if 0 < limit < len(buf):
            return buf[-limit:]
        return buf
