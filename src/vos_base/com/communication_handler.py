import logging

class CommunicationHandler(logging.Handler):
    """ Abstract class for all communication handlers.

        This class provides the required interface for all communication handlers
        to work with the Feeder class.
    """
    __handlerName__: str = 'AbstractHandler'
    __doc__: str = 'Abstract class for all communication handlers.'

    # The handler will fille the communication interface at 
    # loading time if feed list is not empty.
    FILL_AT_LOADING: bool = True

    def __init__(self):
        super().__init__()

    def emit(self, record):
        """ Emit a log record. """
        # This method should be implemented in subclasses
        raise NotImplementedError("Subclasses must implement this method.")
    

    
