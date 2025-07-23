

from typing import Any
import uuid

from vyra_base.helper.logger import Logger
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.redis_access import RedisAccess
from vyra_base.storage.db_access import DBSTATUS
from vyra_base.storage.db_manipulator import DBReturnValue, DbManipulator
from vyra_base.storage.redis_manipulator import RedisManipulator
from vyra_base.storage.tb_params import Parameter as DbParameter

class Parameter:
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
        self.storage_access_persistant = storage_access_persistant

        self.persistant_manipulator = DbManipulator(
            storage_access_persistant, 
            DbParameter
        )

        self.storage_access_transient: Any|None = storage_access_transient

    async def load_defaults(self, storage_access_default: DbAccess) -> None:
        """
        Initialize parameters. Load default parameters if they does not exist. 
        Also add them to the transient storage.
        This method should be implemented to set up initial parameters.
        """

        default_manipulator = DbManipulator(
            storage_access_default, 
            DbParameter
        )
        all_params: DBReturnValue = await self.persistant_manipulator.get_all()
        if all_params.status == DBSTATUS.NOT_FOUND:
            all_default: DBReturnValue = await default_manipulator.get_all()
            if not isinstance(all_default.value, dict):
                Logger.warn(
                    "Default parameters are not a dictionary. "
                    "Cannot load defaults. "
                    "Check the default parameters in the database."
                )
                return

            for name, param in all_default.value.items():
                param_obj: dict[str, Any] = DbParameter(
                    name=name,
                    value=param["value"],
                    type=param["type"],
                    visible=param["visible"],
                    editable=param["editable"],
                    displayname=param["displayname"],
                    description=param["description"]
                ).__dict__
                await default_manipulator.add(param_obj)


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

    def get_param_change_event_topic(self) -> str:
        """
        Get the topic for parameter change events.

        :return: The topic for parameter change events.
        """
        pass