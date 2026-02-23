from datetime import datetime
import json
import logging
from typing import Any, Optional

from sqlalchemy import event
from sqlalchemy import inspect

from vyra_base.com import InterfaceFactory, ProtocolType, remote_service
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.storage.db_access import DBSTATUS, DbAccess
from vyra_base.storage.db_manipulator import DBReturnValue, DbManipulator
from vyra_base.storage.tb_params import Parameter as DbParameter
from vyra_base.storage.tb_params import TypeEnum

logger = logging.getLogger(__name__)


class Parameter:
    """
    Managing module parameters.
    """
    def __init__(
            self, 
            parameter_base_types: dict[str, Any],
            node: Any,
            storage_access_persistant: Any,
            storage_access_persistant_default: Any=None,
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
        self.storage_access_persistant_default = storage_access_persistant_default

        self.persistant_manipulator = DbManipulator(
            storage_access_persistant,
            DbParameter
        )

        self.parameter_base_types = parameter_base_types
        self.update_param_event_ident = "param_update_event_publisher"

        # Store node reference for lazy publisher initialization
        self._node = node
        self.update_parameter_publisher: Optional[Any] = None  # Will be ROS2Publisher

        self.storage_access_transient: Any|None = storage_access_transient

    async def _init_publisher(self) -> None:
        """Initialize the parameter update publisher lazily."""
        if self.update_parameter_publisher is None:
            self.update_parameter_publisher = await InterfaceFactory.create_publisher(
                name=self.update_param_event_ident,
                protocols=[ProtocolType.ROS2],
                message_type=self.parameter_base_types['UpdateParamEvent'],
                node=self._node,
                is_publisher=True
            )

    @ErrorTraceback.w_check_error_exist
    async def load_defaults(
            self, 
            storage_access_default: Optional[DbAccess] = None, 
            override_exist: bool = False) -> bool:
        """
        Initialize parameters. Load default parameters if they does not exist. 
        Also add them to the transient storage.
        This method should be implemented to set up initial parameters.
        :param storage_access_default: The database access object for default parameters.
        :param override_exist: If True, existing parameters will be overridden with defaults.
        :return: None
        :raises ValueError: If the storage_access_default is not of type DbAccess.
        """

        if storage_access_default is not None and not isinstance(storage_access_default, DbAccess):
            raise ValueError("Invalid storage_access_default. Must be of type DbAccess.")

        if storage_access_default is None:
            if self.storage_access_persistant_default is None:
                logger.warning("No default storage access provided. Skipping default "
                            "parameter loading.")
                return False
            storage_access_default = self.storage_access_persistant_default

        if not isinstance(storage_access_default, DbAccess):
            raise ValueError("Invalid storage_access_default. Must be of type DbAccess.")

        default_manipulator = DbManipulator(
            storage_access_default, 
            DbParameter
        )

        if not await storage_access_default.check_table_exists(DbParameter):
            logger.warn(
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
                logger.info("Loading default parameter "
                            f"'{default_item['name']}: {default_item['value']}'")

                param_obj: dict[str, Any] = {
                    key: default_item[key] for key in default_item if key != "id"
                }
                await self.persistant_manipulator.add(param_obj)
            elif override_exist:
                logger.info("Overriding existing parameter "
                            f"'{default_item['name']}: {default_item['value']}'")

                param_obj: dict[str, Any] = {
                    key: default_item[key] for key in default_item if key != "id"
                }

                await self.persistant_manipulator.update(param_obj)

        event.listen(DbParameter, "after_update", self.after_update_param_callback)

        return True

    @remote_service()
    async def get_parameter(self, request: Any, response: Any) -> bool:
        """
        Get a parameter value by its key (ROS2 service interface).
        
        This is the external ROS2 service endpoint. For internal calls,
        use :meth:`get_parameter_impl` instead.

        :param request: The request object containing the key of the parameter to retrieve.
        :type request: Any
        :param response: The response object to store the result.
        :type response: Any
        :return: True if successful, False otherwise.
        :rtype: bool
        
        **ROS2 Service Usage:**
        
        .. code-block:: bash
        
            ros2 service call /module_name/get_parameter \\
                vyra_base_interfaces/srv/GetParameter \\
                "{key: 'param_name'}"
        
        **Internal Usage:**
        
        .. code-block:: python
        
            result = await param_manager.get_parameter_impl("param_name")
            if result:
                print(result["value"])
        """
        key = request.key
        
        param_return = await self.get_parameter_impl(key)
        if param_return is None:
            response.json_value = "[]"
            response.success = False
            response.message = f"Failed to retrieve parameter with key '{key}'."
            return False
        
        response.json_value = param_return["value"]
        response.success = param_return["success"]
        response.message = param_return["message"]
        
        return True
    
    async def get_parameter_impl(self, key: str) -> Optional[dict]:
        """
        Get a parameter value by its key (internal implementation).
        
        This method contains the actual business logic for retrieving a parameter.
        It can be called directly from Python code without going through the ROS2
        service layer.

        :param key: The key of the parameter to retrieve.
        :type key: str
        :return: Dictionary with parameter data or None on error.
        :rtype: Optional[dict]
        
        **Return format:**
        
        .. code-block:: python
        
            {
                "value": "<json_serialized_parameter>",
                "success": True,
                "message": "Parameter 'key' retrieved successfully."
            }
        
        **Example:**
        
        .. code-block:: python
        
            result = await param_manager.get_parameter_impl("robot_speed")
            if result and result["success"]:
                param_data = json.loads(result["value"])
                print(f"Speed: {param_data['value']}")
        """
        response: Any = {}
        response['value'] = "[]"

        all_params: DBReturnValue = await self.persistant_manipulator.get_all(
            filters={"name": key})

        # Treat NOT_FOUND the same as a real DB error for get (parameter doesn't exist)
        if all_params.status == DBSTATUS.NOT_FOUND:
            all_params.status = DBSTATUS.ERROR  # map to failure so caller gets proper error

        if all_params.status != DBSTATUS.SUCCESS:
            response['success'] = False
            response['message'] = f"Failed to retrieve parameter with key '{key}'." 
            logger.warn(response['message'])
            return None

        if not isinstance(all_params.value, list):
            response['success'] = False
            response['message'] = "Internal error: all_params.value is not a list."
            logger.warn(response['message'])
            return None

        all_params.value = self.persistant_manipulator.to_dict(all_params.value[0])
        response['message'] = f"Parameter '{key}' retrieved successfully."
        response['success'] = True
        response['value'] = json.dumps(all_params.value)

        return response

    @remote_service()
    async def set_parameter(self, request: Any, response: Any) -> None:
        """
        Set a parameter value by its key (ROS2 service interface).
        
        This is the external ROS2 service endpoint. For internal calls,
        use :meth:`set_parameter_impl` instead.

        :param request: The request object containing key and value.
        :type request: Any
        :param response: The response object to update with the result.
        :type response: Any
        :return: None
        :rtype: None
        
        **ROS2 Service Usage:**
        
        .. code-block:: bash
        
            ros2 service call /module_name/set_parameter \\
                vyra_base_interfaces/srv/SetParameter \\
                "{key: 'robot_speed', value: '2.5'}"
        
        **Internal Usage:**
        
        .. code-block:: python
        
            result = await param_manager.set_parameter_impl("robot_speed", "2.5")
            if result and result["success"]:
                print("Parameter updated!")
        """
        key = request.key
        value = request.value
        
        param_return = await self.set_parameter_impl(key, value)
        if param_return is None:
            response.success = False
            response.message = "Internal error occurred"
            return None
        
        response.success = param_return["success"]
        response.message = param_return["message"]
        return None
    
    async def set_parameter_impl(self, key: str, value: Any) -> Optional[dict]:
        """
        Set a parameter value by its key (internal implementation).
        
        This method contains the actual business logic for updating a parameter.
        It validates the input, performs type conversion, and updates the database.

        :param key: The key of the parameter to set.
        :type key: str
        :param value: The value to set for the parameter.
        :type value: Any
        :return: Dictionary with success status and message, or None on error.
        :rtype: Optional[dict]
        
        **Return format:**
        
        .. code-block:: python
        
            {
                "success": True,
                "message": "Parameter 'key' updated successfully."
            }
        
        **Example:**
        
        .. code-block:: python
        
            result = await param_manager.set_parameter_impl("robot_speed", "3.0")
            if result and result["success"]:
                print(result["message"])
            else:
                print(f"Error: {result['message'] if result else 'Unknown'}")
        
        **Error handling:**
        
        - Returns success=False if parameter doesn't exist
        - Returns success=False if type conversion fails
        - Returns None on internal errors
        """
        response: dict = {}

        param_ret: DBReturnValue = await self.persistant_manipulator.get_all(
            filters={"name": key})

        # DBSTATUS.NOT_FOUND means the query ran fine but no rows matched (empty table/filter).
        # Treat NOT_FOUND the same as SUCCESS+empty-list: fall through to the upsert path.
        if param_ret.status == DBSTATUS.NOT_FOUND:
            param_ret.status = DBSTATUS.SUCCESS
            param_ret.value = []

        if param_ret.status != DBSTATUS.SUCCESS:
            response['success'] = False
            response['message'] = f"Failed to retrieve parameter with key '{key}' (DB error)."
            logger.warning(response['message'])
            return response

        if not isinstance(param_ret.value, list):
            response['success'] = False
            response['message'] = "Internal error: param_ret.value is not a list."
            logger.error(response['message'])
            return response
            
        if len(param_ret.value) == 0:
            # Parameter does not exist — create it as a string parameter (upsert behavior)
            try:
                new_param: dict[str, Any] = {
                    "name": key,
                    "value": str(value),
                    "default_value": str(value),
                    "type": TypeEnum.string.value,
                    "displayname": key,
                }
                add_ret: DBReturnValue = await self.persistant_manipulator.add(new_param)
                if add_ret.status != DBSTATUS.SUCCESS:
                    response['success'] = False
                    response['message'] = f"Failed to create parameter with key '{key}'."
                    logger.error(response['message'])
                    return response
                response['success'] = True
                response['message'] = f"Parameter '{key}' created successfully."
                logger.info(response['message'])
                return response
            except Exception as e:
                response['success'] = False
                response['message'] = f"Failed to create parameter '{key}': {e}"
                logger.error(response['message'])
                return response
            
        if len(param_ret.value) > 1:
            logger.warn(
                f"Multiple parameters found with key '{key}'. "
                "Updating the first one found.")

        try:
            param_obj: dict[str, Any] = {
                "value": (eval(param_ret.value[0].type.value))(value)
            }
        except ValueError as ve:
            response['success'] = False
            response['message'] = (
                f"Failed to convert value '{value}' to type "
                f"'{param_ret.value[0].type.value}': {ve}")
            logger.error(response['message'])
            return response

        update_ret: DBReturnValue = await self.persistant_manipulator.update(
            param_obj, filters={"name": key})

        if update_ret.status != DBSTATUS.SUCCESS:
            response['success'] = False
            response['message'] = f"Failed to update parameter with key '{key}'."
            logger.error(response['message'])
            return response

        response['success'] = True
        response['message'] = f"Parameter '{key}' updated successfully."
        logger.info(response['message'])
        return response

    @remote_service()
    async def read_all_params(self, request: Any, response: Any) -> None:
        """
        Read all parameters (ROS2 service interface).
        
        This is the external ROS2 service endpoint. For internal calls,
        use :meth:`read_all_params_impl` instead.
        
        Returns a JSON string containing all parameters with their metadata.
        
        :param request: The request object (unused).
        :type request: Any
        :param response: The response object to update with the result.
        :type response: Any
        :return: None
        :rtype: None
        
        **Response format:**
        
        .. code-block:: python
        
            response.all_params_json = '[{"name": "param1", "value": "value1", ...}, ...]'
        
        **ROS2 Service Usage:**
        
        .. code-block:: bash
        
            ros2 service call /module_name/read_all_params \\
                vyra_base_interfaces/srv/ReadAllParams "{}"
        
        **Internal Usage:**
        
        .. code-block:: python
        
            result = await param_manager.read_all_params_impl()
            if result:
                params = json.loads(result["all_params_json"])
                for param in params:
                    print(f"{param['name']}: {param['value']}")
        """
        param_return = await self.read_all_params_impl()
        if param_return is None:
            response.all_params_json = "[]"
            return None
        
        response.all_params_json = param_return["all_params_json"]
        return None
    
    async def read_all_params_impl(self) -> Optional[dict]:
        """
        Read all parameters (internal implementation).
        
        This method retrieves all parameters from the database and serializes
        them to JSON format.
        
        :return: Dictionary containing all parameters as JSON string, or None on error.
        :rtype: Optional[dict]
        
        **Return format:**
        
        .. code-block:: python
        
            {
                "all_params_json": '[{"name": "...", "value": "...", "type": "..."}, ...]'
            }
        
        **Example:**
        
        .. code-block:: python
        
            result = await param_manager.read_all_params_impl()
            if result:
                params = json.loads(result["all_params_json"])
                for param in params:
                    print(f"Parameter: {param['name']}")
                    print(f"  Value: {param['value']}")
                    print(f"  Type: {param['type']}")
        """
        response: dict = {}
        
        ret_obj: DBReturnValue = await self.persistant_manipulator.get_all()
        
        if not isinstance(ret_obj.value, list):
            logger.error("Internal error: ret_obj.value is not a list.")
            return None

        all_params = [
            self.persistant_manipulator.to_dict(p) for p in ret_obj.value]

        response['all_params_json'] = json.dumps(all_params)
        return response

    @ErrorTraceback.w_check_error_exist
    def after_update_param_callback(self, mapper, connection, target) -> None:
        """
        Callback function that is called after a parameter is updated. Will shout
        the parameter change event.

        :param mapper: The mapper.
        :param connection: The database connection.
        :param target: The target object that was updated.
        """
        logger.info(f"Parameter '{target.name}' updated to '{target.value}'")
        insp = inspect(target)

        for attr in insp.attrs:
            if attr.history.has_changes():
                old_value = attr.history.deleted
                new_value = attr.history.added
                logger.debug(f"Feld {attr.key} changed: {old_value} -> {new_value}")
        
        # NOTE: SQLAlchemy event callbacks must be synchronous - cannot use await
        # TODO: Implement async event queue for parameter change notifications via ROS2
        logger.debug(f"Parameter '{target.name}' updated (ROS2 change event not published)")

    @remote_service()
    async def param_changed_topic(self, request: Any, response: Any) -> None:
        """
        Get the ROS2 topic name for parameter change events (ROS2 service interface).
        
        This is the external ROS2 service endpoint. For internal calls,
        use :meth:`param_changed_topic_impl` instead.
        
        Returns the topic name where parameter change notifications are published.
        Other modules can subscribe to this topic to receive updates.

        :param request: The request object (unused).
        :type request: Any
        :param response: The response object to update with the result.
        :type response: Any
        :return: None
        :rtype: None
        
        **ROS2 Service Usage:**
        
        .. code-block:: bash
        
            ros2 service call /module_name/param_changed_topic \\
                vyra_base_interfaces/srv/ParamChangedTopic "{}"
        
        **Internal Usage:**
        
        .. code-block:: python
        
            result = await param_manager.param_changed_topic_impl()
            if result:
                topic = result["topic"]
                print(f"Subscribe to parameter changes: {topic}")
        """
        param_return = await self.param_changed_topic_impl()
        if param_return is None:
            response.topic = ""
            return None
        
        response.topic = param_return["topic"]
        return None
    
    async def param_changed_topic_impl(self) -> Optional[dict]:
        """
        Get the ROS2 topic name for parameter change events (internal implementation).
        
        This method retrieves the topic name from the internal publisher server.
        The topic is used to broadcast parameter changes to other modules.

        :return: Dictionary containing the topic name, or None on error.
        :rtype: Optional[dict]
        
        **Return format:**
        
        .. code-block:: python
        
            {
                "topic": "/module_name/param_update_event"
            }
        
        **Example:**
        
        .. code-block:: python
        
            result = await param_manager.param_changed_topic_impl()
            if result:
                # Subscribe to parameter change events
                node.create_subscription(
                    UpdateParamEvent,
                    result["topic"],
                    callback=on_param_changed
                )
        """
        response: dict = {}
        
        # Ensure publisher is initialized
        if self.update_parameter_publisher is None:
            await self._init_publisher()
        
        # Access internal publisher from ROS2Publisher
        pub_server = getattr(self.update_parameter_publisher, '_publisher', None)
        if pub_server is None:
            logger.error("Publisher server is not initialized.")
            return None
        
        response['topic'] = pub_server.publisher_info.name
        return response