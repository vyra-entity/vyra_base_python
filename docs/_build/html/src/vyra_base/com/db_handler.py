import asyncio

from vyra_base.com.communication_handler import CommunicationHandler

class DBCommunicationHandler(CommunicationHandler):
    """ Abstract class for all Database communication handlers.

        This class provides the required interface for all 
        Database communication handlers to work with the Feeder class.
    """
    __handlerName__: str = 'DatabaseHandler'
    __doc__: str = 'Database communication handler'


    def __init__(self, database):
        super().__init__()
        self.database = database
        self._feedLogger = FeedLogger(db_access=database)
        self.loop = asyncio.get_event_loop()

    def emit(self, record):
        log_entry = self.format(record)
        # Beispiel: Schreibe den Log-Eintrag als String in eine DDS Variable
        # Hier Database eintry
        asyncio.run_coroutine_threadsafe(self._async_emit(log_entry), self.loop)

    async def _async_emit(self, record):
        # Deine asynchrone Logik hier
        await self._feedLogger.add(record)
        