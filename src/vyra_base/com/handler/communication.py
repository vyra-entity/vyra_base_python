import logging
from typing import Any

from vyra_base.com.handler.interfaces import IFeederHandler


class CommunicationHandler(IFeederHandler):
    """
    Concrete base class for all VYRA communication handlers.

    Inherits from :class:`~vyra_base.com.handler.interfaces.IFeederHandler`
    which combines :class:`logging.Handler` with the ``async dispatch()``
    transport interface.

    Subclasses override :meth:`dispatch` for their transport protocol and,
    if needed, override :meth:`emit` to customise the logging-pipeline bridge
    (e.g. format the record as a plain string for a database handler).

    :cvar __handlerName__: Name of the handler.
    :cvar FILL_AT_LOADING: If ``True`` the handler will flush the feeder
        buffer immediately after the transport connection is established.
    """

    __handlerName__: str = 'AbstractHandler'
    __doc__: str = 'Abstract base class for all communication handlers.'

    FILL_AT_LOADING: bool = True

    def __init__(self, initiator: str = '', publisher=None, type=None):
        """
        Initialise the CommunicationHandler.

        :param initiator: Optional initiator string (used in log messages).
        :type initiator: str
        :param publisher: Optional pre-created publisher object.
        :type publisher: object, optional
        :param type: Optional message-type information.
        :type type: object, optional
        """
        super().__init__()

    # ------------------------------------------------------------------
    # IFeederHandler stubs — subclasses must override
    # ------------------------------------------------------------------

    async def dispatch(self, message: Any) -> None:
        """
        Transport *message* over the backing protocol.

        Subclasses must override this method.

        :param message: The message to transport.
        :type message: Any
        :raises NotImplementedError: Always — subclasses must implement.
        """
        raise NotImplementedError("Subclasses must implement dispatch().")

    def get_protocol(self) -> str:
        """
        Return the protocol identifier string.

        Subclasses must override this method.

        :raises NotImplementedError: Always — subclasses must implement.
        """
        raise NotImplementedError("Subclasses must implement get_protocol().")
