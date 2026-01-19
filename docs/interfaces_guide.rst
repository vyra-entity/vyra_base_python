Interfaces
==========

Interface Configuration Guide
------------------------------

VYRA uses JSON configuration files to define ROS2 interfaces (services, topics, and actions) for modules. These configuration files are located in ``src/<module_name>_interfaces/config/`` and automatically generate the corresponding ROS2 message types and service endpoints.

Overview
--------

Each module requires interface definitions for:

- **Services/Callables**: ROS2 services for synchronous request-response communication
- **Topics/Speakers**: ROS2 topics for asynchronous publish-subscribe communication  
- **Actions/Jobs**: ROS2 actions for long-running tasks with feedback (future feature)

File Structure
--------------

Interface configuration files are JSON arrays containing interface definitions:

.. code-block:: text

   src/
   └── <module_name>_interfaces/
       └── config/
           ├── module_core_meta.json      # Core services (required)
           ├── module_state_meta.json     # State management services
           ├── module_custom_meta.json    # Module-specific interfaces
           └── ...

Each JSON file contains an array of interface definitions that share common characteristics.

Services/Callables
------------------

Services provide synchronous request-response communication between modules. They are defined with the ``callable`` type.

Structure
^^^^^^^^^

.. list-table:: Service/Callable Fields
   :widths: 20 15 65
   :header-rows: 1

   * - Field
     - Required
     - Description
   * - ``tags``
     - Yes
     - Array containing ``["ros2_srv"]``
   * - ``type``
     - Yes
     - Must be ``"callable"``
   * - ``functionname``
     - Yes
     - Python method name (snake_case)
   * - ``displayname``
     - Yes
     - Human-readable name for UI
   * - ``description``
     - Yes
     - Detailed description of the service
   * - ``filetype``
     - Yes
     - ROS2 service type name (ends with ``.srv``)
   * - ``access_level``
     - Yes
     - Security level required (1-5)
   * - ``params``
     - Yes
     - Array of input parameters
   * - ``returns``
     - Yes
     - Array of return values
   * - ``displaystyle``
     - No
     - UI display configuration

Parameter/Return Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^

Each parameter and return value has:

.. list-table::
   :widths: 20 15 65
   :header-rows: 1

   * - Field
     - Required
     - Description
   * - ``name``
     - Yes
     - Parameter name (snake_case)
   * - ``datatype``
     - Yes
     - ROS2 data type (e.g., ``string``, ``int32``, ``bool``)
   * - ``displayname``
     - Yes
     - Human-readable name
   * - ``description``
     - Yes
     - Parameter description

Example: Service Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
       "tags": ["ros2_srv"],
       "type": "callable",
       "functionname": "set_parameter",
       "displayname": "Set Parameter",
       "description": "Set a configuration parameter in the system",
       "filetype": "VBASESetParam.srv",
       "access_level": 3,
       "params": [
           {
               "name": "key",
               "datatype": "string",
               "displayname": "Parameter Key",
               "description": "The key of the parameter to set"
           },
           {
               "name": "value",
               "datatype": "string",
               "displayname": "Parameter Value",
               "description": "The value to set for the parameter"
           }
       ],
       "returns": [
           {
               "name": "success",
               "datatype": "bool",
               "displayname": "Success",
               "description": "True if parameter was set successfully"
           },
           {
               "name": "message",
               "datatype": "string",
               "displayname": "Message",
               "description": "Status message or error description"
           }
       ],
       "displaystyle": {
           "visible": true,
           "published": false
       }
   }

Implementation
^^^^^^^^^^^^^^

After defining the service in JSON, implement it in your application:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import remote_callable
   from vyra_base.security import security_required, SecurityLevel

   class Application(OperationalStateMachine):
       @remote_callable
       @security_required(security_level=SecurityLevel.EXTENDED_AUTH)
       def set_parameter(self, request=None, response=None):
           """
           Implementation matching the interface definition.
           Access level 3 (EXTENDED_AUTH) required.
           """
           try:
               self.entity.parameters[request.key] = request.value
               response.success = True
               response.message = f"Parameter '{request.key}' set successfully"
           except Exception as e:
               response.success = False
               response.message = str(e)
           
           return response

Topics/Speakers
---------------

Topics provide asynchronous publish-subscribe communication. Publishers send messages without waiting for responses, and multiple subscribers can listen to the same topic.

Structure
^^^^^^^^^

.. list-table:: Topic/Speaker Fields
   :widths: 20 15 65
   :header-rows: 1

   * - Field
     - Required
     - Description
   * - ``tags``
     - Yes
     - Array containing ``["ros2_topic"]`` or ``["ros2_pub"]``
   * - ``type``
     - Yes
     - Must be ``"speaker"``
   * - ``functionname``
     - Yes
     - Topic name (snake_case)
   * - ``displayname``
     - Yes
     - Human-readable name
   * - ``description``
     - Yes
     - Topic purpose description
   * - ``filetype``
     - Yes
     - ROS2 message type name (ends with ``.msg``)
   * - ``frequency``
     - No
     - Publishing frequency in Hz (default: 10.0)
   * - ``params``
     - Yes
     - Array of message fields
   * - ``displaystyle``
     - No
     - UI display configuration

Example: Topic Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
       "tags": ["ros2_topic"],
       "type": "speaker",
       "functionname": "status_updates",
       "displayname": "Status Updates",
       "description": "Publishes module status updates periodically",
       "filetype": "ModuleStatus.msg",
       "frequency": 1.0,
       "params": [
           {
               "name": "module_name",
               "datatype": "string",
               "displayname": "Module Name",
               "description": "Name of the module"
           },
           {
               "name": "state",
               "datatype": "string",
               "displayname": "State",
               "description": "Current operational state"
           },
           {
               "name": "cpu_usage",
               "datatype": "float32",
               "displayname": "CPU Usage",
               "description": "CPU usage percentage"
           },
           {
               "name": "timestamp",
               "datatype": "builtin_interfaces/Time",
               "displayname": "Timestamp",
               "description": "Time of status update"
           }
       ],
       "displaystyle": {
           "visible": true,
           "published": true
       }
   }

Implementation
^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.speaker import VyraSpeaker
   from vyra_base.com.datalayer.interface_factory import create_vyra_speaker

   class Application(OperationalStateMachine):
       def __init__(self, entity, *args, **kwargs):
           super().__init__(entity.unified_state_machine)
           self.entity = entity
           
           # Create speaker from interface definition
           self.status_speaker = create_vyra_speaker(
               node=entity.node,
               name="status_updates",
               msg_type=ModuleStatus,
               frequency=1.0
           )
       
       async def publish_status(self):
           """Publish status update."""
           msg = ModuleStatus()
           msg.module_name = self.entity.module_entry.name
           msg.state = self.entity.state_machine.operational_state.name
           msg.cpu_usage = self.get_cpu_usage()
           msg.timestamp = self.entity.node.get_clock().now().to_msg()
           
           self.status_speaker.publish(msg)

Actions/Jobs
------------

Actions provide long-running operations with feedback. They are useful for tasks that take time to complete and need to report progress.

.. note::
   Action support is planned for future releases. Current implementation uses simplified job pattern.

Structure
^^^^^^^^^

.. list-table:: Action/Job Fields
   :widths: 20 15 65
   :header-rows: 1

   * - Field
     - Required
     - Description
   * - ``tags``
     - Yes
     - Array containing ``["ros2_action"]`` or ``["job"]``
   * - ``type``
     - Yes
     - Must be ``"job"``
   * - ``functionname``
     - Yes
     - Action name (snake_case)
   * - ``displayname``
     - Yes
     - Human-readable name
   * - ``description``
     - Yes
     - Action purpose description
   * - ``filetype``
     - Yes
     - ROS2 action type name (ends with ``.action``)
   * - ``access_level``
     - Yes
     - Security level required (1-5)
   * - ``goal``
     - Yes
     - Array of goal parameters
   * - ``result``
     - Yes
     - Array of result fields
   * - ``feedback``
     - Yes
     - Array of feedback fields
   * - ``displaystyle``
     - No
     - UI display configuration

Example: Action Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
       "tags": ["ros2_action"],
       "type": "job",
       "functionname": "execute_task",
       "displayname": "Execute Task",
       "description": "Execute a long-running task with progress feedback",
       "filetype": "ExecuteTask.action",
       "access_level": 3,
       "goal": [
           {
               "name": "task_id",
               "datatype": "string",
               "displayname": "Task ID",
               "description": "Unique identifier for the task"
           },
           {
               "name": "parameters",
               "datatype": "string",
               "displayname": "Parameters",
               "description": "JSON-encoded task parameters"
           }
       ],
       "result": [
           {
               "name": "success",
               "datatype": "bool",
               "displayname": "Success",
               "description": "True if task completed successfully"
           },
           {
               "name": "result_data",
               "datatype": "string",
               "displayname": "Result Data",
               "description": "JSON-encoded result data"
           }
       ],
       "feedback": [
           {
               "name": "progress",
               "datatype": "float32",
               "displayname": "Progress",
               "description": "Completion percentage (0.0-100.0)"
           },
           {
               "name": "status_message",
               "datatype": "string",
               "displayname": "Status Message",
               "description": "Current operation status"
           }
       ],
       "displaystyle": {
           "visible": true,
           "published": false
       }
   }

Implementation (Simplified)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.job import VyraJob
   from vyra_base.com.datalayer.interface_factory import create_vyra_job

   class Application(OperationalStateMachine):
       @remote_callable
       @security_required(security_level=SecurityLevel.EXTENDED_AUTH)
       async def execute_task(self, request=None, response=None):
           """
           Execute long-running task.
           In full action implementation, this would return action handle.
           """
           task_id = request.task_id
           
           # Create job for async execution
           job = create_vyra_job(
               name=f"task_{task_id}",
               callback=self._execute_task_impl,
               args=(task_id, request.parameters)
           )
           
           response.success = True
           response.message = f"Task {task_id} started"
           return response
       
       async def _execute_task_impl(self, task_id, parameters):
           """Internal task implementation with progress updates."""
           for progress in range(0, 101, 10):
               await asyncio.sleep(0.5)
               # Publish feedback (simplified)
               self.entity.publish_news(
                   f"Task {task_id}: {progress}% complete"
               )
           
           return {"success": True, "result": "Task completed"}

Common Data Types
-----------------

ROS2 Standard Types
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Type
     - Description
   * - ``bool``
     - Boolean (true/false)
   * - ``int8``, ``int16``, ``int32``, ``int64``
     - Signed integers
   * - ``uint8``, ``uint16``, ``uint32``, ``uint64``
     - Unsigned integers
   * - ``float32``, ``float64``
     - Floating point numbers
   * - ``string``
     - UTF-8 string
   * - ``byte``
     - Raw byte data
   * - ``type[]``
     - Unbounded array (e.g., ``string[]``)
   * - ``type[n]``
     - Fixed-size array (e.g., ``float32[3]``)

Common ROS2 Message Types
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - Type
     - Description
   * - ``builtin_interfaces/Time``
     - ROS timestamp (seconds + nanoseconds)
   * - ``builtin_interfaces/Duration``
     - Time duration
   * - ``std_msgs/Header``
     - Message header with timestamp and frame_id
   * - ``geometry_msgs/Point``
     - 3D point (x, y, z)
   * - ``geometry_msgs/Pose``
     - Position and orientation
   * - ``unique_identifier_msgs/UUID``
     - UUID (16 bytes)

Access Levels
-------------

Every interface must specify an ``access_level`` (1-5) that determines which modules can call it:

.. list-table::
   :widths: 10 20 70
   :header-rows: 1

   * - Level
     - Name
     - Description
   * - 1
     - NONE
     - Public access, no authentication required
   * - 2
     - BASIC_AUTH
     - Module ID verification required
   * - 3
     - EXTENDED_AUTH
     - Module ID + password authentication
   * - 4
     - HMAC
     - HMAC-SHA256 message integrity check
   * - 5
     - DIGITAL_SIGNATURE
     - Certificate-based PKI authentication

See :doc:`security` for detailed security documentation.

Best Practices
--------------

Naming Conventions
^^^^^^^^^^^^^^^^^^

- **Function names**: Use ``snake_case`` (e.g., ``set_parameter``, ``get_status``)
- **Display names**: Use Title Case (e.g., ``"Set Parameter"``, ``"Get Status"``)
- **File types**: Use PascalCase with suffix (e.g., ``VBASESetParam.srv``, ``ModuleStatus.msg``)
- **Parameter names**: Use ``snake_case``

Organization
^^^^^^^^^^^^

1. **Group related interfaces**: Put core services in ``*_core_meta.json``, state management in ``*_state_meta.json``
2. **Consistent naming**: Use module prefix for custom types (e.g., ``V2MMSetConfig.srv`` for v2_modulemanager)
3. **Access levels**: Start with level 2 (BASIC_AUTH) as minimum for non-public services
4. **Documentation**: Provide clear descriptions for all interfaces and parameters

Interface Versioning
^^^^^^^^^^^^^^^^^^^^

When modifying existing interfaces:

1. **Non-breaking changes**: Add optional parameters at the end
2. **Breaking changes**: Create new interface with version suffix (e.g., ``SetParamV2.srv``)
3. **Deprecation**: Mark old interfaces as deprecated in description
4. **Migration**: Provide migration guide in module documentation

Example: Complete Module Interfaces
------------------------------------

Complete example for a module with multiple interface types:

.. code-block:: json

   [
       {
           "tags": ["ros2_srv"],
           "type": "callable",
           "functionname": "get_info",
           "displayname": "Get Module Info",
           "description": "Retrieve module information and capabilities",
           "filetype": "GetInfo.srv",
           "access_level": 1,
           "params": [],
           "returns": [
               {
                   "name": "module_name",
                   "datatype": "string",
                   "displayname": "Module Name"
               },
               {
                   "name": "version",
                   "datatype": "string",
                   "displayname": "Version"
               },
               {
                   "name": "capabilities",
                   "datatype": "string[]",
                   "displayname": "Capabilities"
               }
           ]
       },
       {
           "tags": ["ros2_topic"],
           "type": "speaker",
           "functionname": "heartbeat",
           "displayname": "Heartbeat",
           "description": "Module alive signal",
           "filetype": "Heartbeat.msg",
           "frequency": 1.0,
           "params": [
               {
                   "name": "timestamp",
                   "datatype": "builtin_interfaces/Time"
               },
               {
                   "name": "sequence",
                   "datatype": "uint64"
               }
           ]
       },
       {
           "tags": ["ros2_srv"],
           "type": "callable",
           "functionname": "emergency_stop",
           "displayname": "Emergency Stop",
           "description": "Immediately halt all operations",
           "filetype": "EmergencyStop.srv",
           "access_level": 5,
           "params": [
               {
                   "name": "reason",
                   "datatype": "string",
                   "displayname": "Reason"
               }
           ],
           "returns": [
               {
                   "name": "success",
                   "datatype": "bool"
               }
           ]
       }
   ]

See Also
--------

- :doc:`security`: Security framework and access control
- :doc:`com/overview`: Communication overview
- :doc:`core/entity`: Entity initialization and interface registration
- :doc:`MIGRATION_ACCESS_LEVEL`: Access level integration guide
