from datetime import datetime
import json
from typing import Any

from sqlalchemy import event
from sqlalchemy import inspect

from vyra_base.com.datalayer.interface_factory import (
    create_vyra_speaker,
    remote_callable
)
from vyra_base.com.datalayer.speaker import VyraSpeaker
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
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
            parameter_base_types: dict[str, Any],
            node: Any,
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
        :param node: The ROS2 node to attach the parameter manager to.
        :param parameter_base_types: A dictionary of base types for parameters.
        :type parameter_base_types: dict[str, Any]
        :param storage_access_persistant: The database access object for persistent parameters.
        :type storage_access_persistant: DbAccess
        :param storage_access_transient: The database access object for transient parameters.
        :type storage_access_transient: DbAccess | None
        :raises ValueError: If the storage_access is not of type DbAccess.
        """
        self.storage_access_persistant = storage_access_persistant

        self.persistant_manipulator = DbManipulator(
            storage_access_persistant,
            DbParameter
        )

        self.parameter_base_types = parameter_base_types
        self.update_param_event_ident = "param_update_event_feeder"

        self.update_parameter_speaker: VyraSpeaker = create_vyra_speaker(
            type=parameter_base_types['UpdateParamEvent'], 
            node=node, 
            description="Parameter update event feeder.",
            ident_name=self.update_param_event_ident
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
    async def get_param(self, request: Any, response: Any) -> None:
        """
        Get a parameter value by its key (api).

        :param key: The key of the parameter to retrieve.
        :return: The value of the parameter.
        :raises ValueError: If the parameter does not exist or if multiple parameters are found.
        :rtype: None
        """
        key = request.key

        response.json_value = "[]"

        all_params: DBReturnValue = await self.persistant_manipulator.get_all(
            filters={"name": key})

        if all_params.status != DBSTATUS.SUCCESS:
            response.success = False
            response.message = f"Failed to retrieve parameter with key '{key}'." 
            Logger.warn(response.message)
            return None

        if not isinstance(all_params.value, list):
            response.success = False
            response.message = "Internal error: all_params.value is not a list."
            Logger.warn(response.message)
            return None

        all_params.value = self.persistant_manipulator.to_dict(all_params.value[0])
        response.message = f"Parameter '{key}' retrieved successfully."
        response.success = True
        response.json_value = json.dumps(all_params.value)

        return None


    @ErrorTraceback.w_check_error_exist
    @remote_callable
    async def set_param(self, request: Any, response: Any) -> None:
        """
        Set a parameter value by its key (api).

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
            response.success = False
            response.message = (f"Parameter with key '{key}' does not exist.")
            Logger.warn(response.message)
            return None
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
    async def read_all_params(self, request: Any, response: Any) -> None:
        """
        Read all parameters (api). Return a JSON string containing all parameters.
        Example:
        response.all_params_json = '[{"name": "param1", "value": "value1"}, ...]'
        
        :return: None
        """
        ret_obj: DBReturnValue = await self.persistant_manipulator.get_all()
        if not isinstance(ret_obj.value, list):
            raise ValueError("ret_obj.value must be a list.")

        all_params = [
            self.persistant_manipulator.to_dict(p) for p in ret_obj.value]

        response.all_params_json = json.dumps(all_params)

        return None

    @ErrorTraceback.w_check_error_exist
    def after_update_param_callback(self, mapper, connection, target) -> None:
        """
        Callback function that is called after a parameter is updated. Will shout
        the parameter change event.

        :param mapper: The mapper.
        :param connection: The database connection.
        :param target: The target object that was updated.
        """
        Logger.info(f"Parameter '{target.name}' updated to '{target.value}'")
        insp = inspect(target)

        for attr in insp.attrs:
            if attr.history.has_changes():
                old_value = attr.history.deleted
                new_value = attr.history.added
                Logger.debug(f"Feld {attr.key} changed: {old_value} -> {new_value}")
                
        self.update_parameter_speaker.shout(
            message={
                "param_key": target.name,
                "changed_from": old_value,
                "changed_to": new_value,
                "timestamp": Ros2TypeConverter.time_to_ros2buildintime(
                    datetime.now())
            }
        )

    @ErrorTraceback.w_check_error_exist
    @remote_callable
    async def get_update_param_event_topic(self, request: Any, response: Any) -> None:
        """
        Get the topic for parameter change events (api).

        :return: The topic for parameter change events.
        """
        pub_server = self.update_parameter_speaker.publisher_server
        if pub_server is None:
            raise RuntimeError("Publisher server is not initialized.")
        else:
            response.topic = pub_server.publisher_info.name
        return None