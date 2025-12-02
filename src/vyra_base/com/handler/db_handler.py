import asyncio

from vyra_base.com.handler.communication_handler import CommunicationHandler

class DBCommunicationHandler(CommunicationHandler):
    """
    Abstract class for all Database communication handlers.

    This class provides the required interface for all Database communication
    handlers to work with the Feeder class.

    :cvar __handlerName__: Name of the handler.
    :cvar __doc__: Documentation string for the handler.
    :param database: Database connection or handler instance.
    """
    __handlerName__: str = 'DatabaseHandler'
    __doc__: str = 'Database(db) communication handler'

    def __init__(self, database):
        """
        Initialize the DBCommunicationHandler.

        :param database: Database connection or handler instance.
        """
        super().__init__()
        self.database = database
        self._feedLogger = FeedLogger(db_access=database)
        self.loop = asyncio.get_event_loop()

    def emit(self, record):
        """
        Emit a log record.

        Formats the record and schedules it to be written asynchronously.

        :param record: The log record to emit.
        """
        log_entry = self.format(record)
        asyncio.run_coroutine_threadsafe(self._async_emit(log_entry), self.loop)

    async def _async_emit(self, record):
        """
        Asynchronously emit a log record.

        :param record: The log record to emit.
        """
        await self._feedLogger.add(record)