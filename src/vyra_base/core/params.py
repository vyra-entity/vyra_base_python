

from typing import Any


class Param:
    """
    Managing module parameters.
    """
    def __init__(
            self, 
            storage_access_persistant: Any, 
            storage_access_transient: Any = None
        )   -> None:
        """
        Initialize vyra module parameters.

        :param storage_access: An object that provides access to the storage system.
        """
        self.storage_access = storage_access_persistant
        self.storage_access_transient: Any|None = storage_access_transient

        self._init_params()

    def _init_params(self) -> None:
        """
        Initialize parameters. Load default parameters if they does not exist. 
        Also add them to the transient storage.
        This method should be implemented to set up initial parameters.
        """
        pass

    def get_param(self, key: str) -> Any:
        """
        Get a parameter value by its key.

        :param key: The key of the parameter to retrieve.
        :return: The value of the parameter.
        """
        pass

    def set_param(self, key: str, value: Any) -> None:
        """
        Set a parameter value by its key.

        :param key: The key of the parameter to set.
        :param value: The value to set for the parameter.
        """
        pass

    def read_all_params(self) -> dict[str, Any]:
        """
        Read all parameters.

        :return: A dictionary containing all parameters.
        """
        pass

    def get_param_change_event_topic(self):
        """
        Get the topic for parameter change events.

        :return: The topic for parameter change events.
        """
        pass