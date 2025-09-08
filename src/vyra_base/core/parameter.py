from typing import Any

from sqlalchemy import event

from vyra_base.com.datalayer.interface_factory import remote_callable
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger
from vyra_base.storage.db_access import DBSTATUS, DbAccess
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
        Initialize vyra module parameters. Parameter containing basic settings for the
        module which could be used bei the specific processes in the module and will
        be configured by the user or by other modules. They will be stored in a database 
        table. Other modules could subscribe to a parameter change event to be informed
        about changes on a parameter.

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

        event.listen(DbParameter, "after_update", self.after_update_param_callback)

        return True

    @ErrorTraceback.w_check_error_exist
    @remote_callable
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
    @remote_callable
    async def set_param(self, request: Any, response: Any) -> None:
        """
        Set a parameter value by its key.

        :param key: The key of the parameter to set.
        :param value: The value to set for the parameter.
        """
        key = request.key
        value = request.value

        param_ret: DBReturnValue = await self.persistant_manipulator.get_all(
            filters={"name": key})

        if param_ret.status != DBSTATUS.SUCCESS:
            response.success = False
            response.message = f"Failed to retrieve parameter with key '{key}'."
            return 

        if not isinstance(param_ret.value, list):
            raise ValueError("param_ret.value must be a list.")
        if len(param_ret.value) == 0:
            raise ValueError(f"Parameter with key '{key}' does not exist.")
        if len(param_ret.value) > 1:
            Logger.warn(
                f"Multiple parameters found with key '{key}'. "
                "Updating the first one found.")

        try:
            param_obj: dict[str, Any] = {
                "value": (eval(param_ret.value[0].type.value))(value)
            }
        except ValueError as ve:
            response.success = False
            response.message = (
                f"Failed to convert value '{value}' to type "
                f"'{param_ret.value[0].type.value}': {ve}")
            return None

        update_ret: DBReturnValue = await self.persistant_manipulator.update(
            param_obj, filters={"name": key})

        if update_ret.status != DBSTATUS.SUCCESS:
            response.status = False
            response.message = f"Failed to update parameter with key '{key}'."
            return response

        response.status = True
        response.message = f"Parameter '{key}' updated successfully."
        return None

    @ErrorTraceback.w_check_error_exist
    @remote_callable
    async def read_all_params(self) -> DBReturnValue:
        """
        Read all parameters.

        :return: A dictionary containing all parameters.
        """
        ret_obj: DBReturnValue = await self.persistant_manipulator.get_all()
        if not isinstance(ret_obj.value, list):
            raise ValueError("ret_obj.value must be a list.")

        ret_obj.value = [
            self.persistant_manipulator.to_dict(p) for p in ret_obj.value]

        return ret_obj

    def after_update_param_callback(self, mapper, connection, target) -> None:
        """
        Callback function that is called after a parameter is updated.

        :param mapper: The mapper.
        :param connection: The database connection.
        :param target: The target object that was updated.
        """
        Logger.info(f"Parameter '{target.name}' updated to '{target.value}'")
        # Here you can add code to publish an event or notify other components
        # about the parameter change.

    @ErrorTraceback.w_check_error_exist
    @remote_callable
    async def get_update_param_event_topic(self, request: Any, response: Any) -> None:
        """
        Get the topic for parameter change events.

        :return: The topic for parameter change events.
        """
        pass
        # return response