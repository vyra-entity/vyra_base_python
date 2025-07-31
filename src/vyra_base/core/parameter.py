from typing import Any

from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DBReturnValue, DbManipulator
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

    @ErrorTraceback.w_check_error_exist
    async def load_defaults(
            self, 
            storage_access_default: DbAccess, 
            override_exist: bool=False) -> bool:
        """
        Initialize parameters. Load default parameters if they does not exist. 
        Also add them to the transient storage.
        This method should be implemented to set up initial parameters.
        :param storage_access_default: The database access object for default parameters.
        :param override_exist: If True, existing parameters will be overridden with defaults.
        :return: None
        :raises ValueError: If the storage_access_default is not of type DbAccess.
        """

        if not isinstance(storage_access_default, DbAccess):
            raise ValueError("Invalid storage_access_default. Must be of type DbAccess.")

        default_manipulator = DbManipulator(
            storage_access_default, 
            DbParameter
        )

        if not await storage_access_default.check_table_exists(DbParameter):
            Logger.warn(
                f"Table '{DbParameter.__tablename__}' does not exist in the default "
                "database. Skipping default parameter loading.")
            return False
        
        all_return: DBReturnValue = await self.persistant_manipulator.get_all()

        if not isinstance(all_return.value, list):
            raise ValueError(f"all_return.value must be a list. Is: {all_return.value}")

        all_params: list[dict] = [default_manipulator.to_dict(p) for p in all_return.value]

        all_default_return: DBReturnValue = await default_manipulator.get_all()

        if not isinstance(all_default_return.value, list):
            raise ValueError("all_default.value must be a list.")
        
        all_default: list[dict] = [
            default_manipulator.to_dict(p) for p in all_default_return.value
        ]

        param_name_list: list[str] = [n["name"] for n in all_params]

        for default_item in all_default:
            if default_item["name"] not in param_name_list:
                Logger.info("Loading default parameter "
                            f"'{default_item['name']}: {default_item['value']}'")

                param_obj: dict[str, Any] = {
                    key: default_item[key] for key in default_item if key != "id"
                }
                await self.persistant_manipulator.add(param_obj)
            elif override_exist:
                Logger.info("Overriding existing parameter "
                            f"'{default_item['name']}: {default_item['value']}'")

                param_obj: dict[str, Any] = {
                    key: default_item[key] for key in default_item if key != "id"
                }

                await self.persistant_manipulator.update(param_obj)

        return True

    @ErrorTraceback.w_check_error_exist
    async def get_param(self, key: str) -> DBReturnValue:
        """
        Get a parameter value by its key.

        :param key: The key of the parameter to retrieve.
        :return: The value of the parameter.
        """
        ret_obj: DBReturnValue = await self.persistant_manipulator.get_all(
            filters={"name": key})
        
        if not isinstance(ret_obj.value, list):
            raise ValueError("ret_obj.value must be a list.")
        
        ret_obj.value = self.persistant_manipulator.to_dict(ret_obj.value[0])
        
        return ret_obj

    @ErrorTraceback.w_check_error_exist
    async def set_param(self, key: str, value: Any) -> DBReturnValue:
        """
        Set a parameter value by its key.

        :param key: The key of the parameter to set.
        :param value: The value to set for the parameter.
        """
        param_obj = {
            "value": value
        }
        return await self.persistant_manipulator.update(
            param_obj, filters={"name": key})

    @ErrorTraceback.w_check_error_exist
    async def read_all_params(self) -> DBReturnValue:
        """
        Read all parameters.

        :return: A dictionary containing all parameters.
        """
        ret_obj: DBReturnValue = await self.persistant_manipulator.get_all()
        if not isinstance(ret_obj.value, list):
            raise ValueError("ret_obj.value must be a list.")

        ret_obj.value = [self.persistant_manipulator.to_dict(p) for p in ret_obj.value]

        return ret_obj

    @ErrorTraceback.w_check_error_exist
    def get_param_change_event_topic(self) -> str:
        """
        Get the topic for parameter change events.

        :return: The topic for parameter change events.
        """
        return ""