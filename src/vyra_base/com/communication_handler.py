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

    def __init__(self, initiator: str = '', publisher=None, type=None):
        """
        Initialize the CommunicationHandler.

        :param initiator: Optional initiator string.
        :type initiator: str
        :param publisher: Optional publisher object.
        :type publisher: object, optional
        :param type: Optional type information.
        :type type: object, optional
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
