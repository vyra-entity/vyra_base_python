import logging

class CommunicationHandler(logging.Handler):
    """
    Abstract base class for all communication handlers.

    This class provides the required interface for all communication handlers
    to work with the Feeder class.
    """

    __handlerName__: str = 'AbstractHandler'
    __doc__: str = 'Abstract class for all communication handlers.'

    FILL_AT_LOADING: bool = True
    #: bool: The handler will fill the communication interface at loading time if feed list is not empty.

    def __init__(self, initiator: str = '', speaker=None, type=None):
        """
        Initialize the CommunicationHandler.

        :param initiator: Optional initiator string.
        :type initiator: str
        :param speaker: Optional speaker object.
        :type speaker: object, Optional
        :param type: Optional type information.
        :type type: object, Optional
        """
        super().__init__()

    def emit(self, record):
        """
        Emit a log record.

        This method should be implemented in subclasses.

        :param record: Log record to emit.
        :type record: logging.LogRecord
        :raises NotImplementedError: If the method is not implemented in a subclass.
        """
        raise NotImplementedError("Subclasses must implement this method.")
