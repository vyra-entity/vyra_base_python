IPC - Inter Process Communication
==================================

VYRA supports IPC (Inter Process Communication) via **gRPC with Unix Domain Sockets**.
This enables fast inter process communication **within a module**.

Concept
-------

IPC used for:

✅ **Use Cases:**

* Communication between processes in the same container/module
* Fast internal API calls without network overhead
* Decoupling of frontend and backend within the same module
* Python ↔ Python communication within a module

❌ **Not suitable for:**

* Inter-module communication (use ROS2)
* Communication across container boundaries
* External API access

.. note::
   For communication **between modules**, always use ROS2 (Job/Callable).
   IPC is only intended for communication **within a module**.

gRPC via Unix Domain Socket
-----------------------------

VYRA uses **Unix Domain Sockets** instead of TCP for gRPC communication:

**Advantages:**

* ⚡ Very fast (no network latency)
* 🔒 Secure (local processes only)
* 🎯 No port conflicts
* 💾 Less overhead than TCP/IP

**Socket Path:** ``/tmp/<module_name>_ipc.sock``

Setup
-----------

Server-Side (Backend)
^^^^^^^^^^^^^^^^^^^^^^

Create a gRPC server with Unix Domain Socket:

.. code-block:: python

   import grpc
   from concurrent import futures
   from vyra_base.com.handler.ipc import IPCHandler
   
   # gRPC Server with Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
   
   # Service register (Your gRPC Service)
   MyServiceServicer.add_to_server(MyServiceImpl(), server)
   
   # Bind Unix Domain Socket
   server.add_insecure_port(f'unix://{socket_path}')
   server.start()
   print(f"IPC Server running on {socket_path}")

Client-Side (Frontend/and other Processes)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Connect to the gRPC server:

.. code-block:: python

   import grpc
   
   # Connect via Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   channel = grpc.insecure_channel(f'unix://{socket_path}')
   
   # Client-Stub create
   stub = MyServiceStub(channel)
   
   # RPC call
   request = MyRequest(data="Hello")
   response = stub.CallMethod(request)
   print(f"Response: {response.result}")

Practical Example
-----------------
Scenario: Frontend-Backend Communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Backend (Python gRPC Server)**:

.. code-block:: python

   # backend_service.proto (gRPC Definition)
   # service BackendService {
   #     rpc GetStatus(StatusRequest) returns (StatusResponse);
   #     rpc ProcessData(DataRequest) returns (DataResponse);
   # }
   
   from backend_service_pb2_grpc import BackendServiceServicer
   import grpc
   from concurrent import futures
   
   class BackendServiceImpl(BackendServiceServicer):
       def GetStatus(self, request, context):
           # Backend logic
           return StatusResponse(
               status="running",
               uptime_seconds=12345
           )
       
       def ProcessData(self, request, context):
           # Data processing
           result = process(request.data)
           return DataResponse(result=result)
   
   # Start server
   def start_ipc_server():
       socket_path = "/tmp/my_module_ipc.sock"
       server = grpc.server(futures.ThreadPoolExecuto(max_workers=10))
       BackendServiceServicer.add_to_server(BackendServiceImpl(), server)
       server.add_insecure_port(f'unix://{socket_path}')
       server.start()
       server.wait_for_termination()

**Frontend (Python gRPC Client)**:

.. code-block:: python

   from backend_service_pb2 import StatusRequest, DataRequest
   from backend_service_pb2_grpc import BackendServiceStub
   import grpc
   
   class FrontendClient:
       def __init__(self):
           socket_path = "/tmp/my_module_ipc.sock"
           self.channel = grpc.insecure_channel(f'unix://{socket_path}')
           self.stub = BackendServiceStub(self.channel)
       
       def get_backend_status(self):
           request = StatusRequest()
           response = self.stub.GetStatus(request)
           return {
               "status": response.status,
               "uptime": response.uptime_seconds
           }
       
       def process_data(self, data):
           request = DataRequest(data=data)
           response = self.stub.ProcessData(request)
           return response.result
   
   # Usage
   client = FrontendClient()
   status = client.get_backend_status()
   print(f"Backend-Status: {status}")

gRPC Proto-Definitionen
-----------------------

Define your gRPC services in ``.proto`` files:

.. code-block:: protobuf

   syntax = "proto3";
   
   package vyra.ipc;
   
   // Service-Definition
   service ModuleService {
       rpc Initialize(InitRequest) returns (InitResponse);
       rpc ExecuteCommand(CommandRequest) returns (CommandResponse);
       rpc GetData(DataRequest) returns (DataResponse);
   }
   
   // Message-Definitions
   message InitRequest {
       string config_path = 1;
   }
   
   message InitResponse {
       bool success = 1;
       string message = 2;
   }
   
   message CommandRequest {
       string command = 1;
       map<string, string> parameters = 2;
   }
   
   message CommandResponse {
       int32 status_code = 1;
       string result = 2;
   }

Compiling the Proto Files:

.. code-block:: bash

   # Generate Python gRPC code
   python -m grpc_tools.protoc \
       -I. \
       --python_out=. \
       --grpc_python_out=. \
       module_service.proto

IPCHandler Class
-----------------

VYRA provides a helper class for IPC connections:

.. code-block:: python

   from vyra_base.com.handler.ipc import IPCHandler
   
   # IPC-Handler initialize
   ipc = IPCHandler(
       socket_path="/tmp/my_module_ipc.sock",
       service_stub=MyServiceStub
   )
   
   # Establish connection
   await ipc.connect()
   
   # RPC call
   response = await ipc.call_method("GetStatus", request)
   
   # Close connection
   await ipc.disconnect()

.. note::
   Further details to IPCHandler implementation can be found in the
   :class:`~vyra_base.com.handler.ipc.IPCHandler` API Reference.

Performance
-----------

**Typical Latencies:**

* Unix Domain Socket: ~0.05 - 0.2 ms
* TCP localhost: ~0.2 - 1 ms
* ROS2 Service: ~1 - 5 ms

➡️ IPC is **10-100x faster** than ROS2 for local communication.

Best Practices
--------------

✅ **Recommended:**

* Use IPC only within a module
* Define clear Proto definitions
* Implement error handling and timeouts
* Clean up socket files on shutdown
* Use connection pooling for frequent calls

❌ **Avoid:**

* IPC for inter-module communication (use ROS2)
* Very large messages (> 10 MB, use shared memory)
* Blocking calls without timeout
* Hardcoded socket paths (use your configuration)

Error Handling
--------------

.. code-block:: python

   import grpc
   
   try:
       channel = grpc.insecure_channel(f'unix://{socket_path}')
       stub = MyServiceStub(channel)
       
       # With timeout
       response = stub.CallMethod(
           request,
           timeout=5.0  # 5 seconds Timeout
       )
   except grpc.RpcError as e:
       if e.code() == grpc.StatusCode.UNAVAILABLE:
           print("Server not reachable")
       elif e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
           print("Timeout")
       else:
           print(f"gRPC-Error: {e}")
   except Exception as e:
       print(f"Unexpected error: {e}")

Cleanup
-------

Socket files should be deleted on shutdown:

.. code-block:: python

   import os
   import signal
   
   def cleanup_socket(socket_path):
       if os.path.exists(socket_path):
           os.remove(socket_path)
           print(f"Socket {socket_path} removed")
   
   # Clean up on shutdown
   def shutdown_handler(signum, frame):
       cleanup_socket("/tmp/my_module_ipc.sock")
       server.stop(grace_period=5)
       sys.exit(0)
   
   signal.signal(signal.SIGINT, shutdown_handler)
   signal.signal(signal.SIGTERM, shutdown_handler)

Further Information
-------------------

* :doc:`ros2_communication` - ROS2 for Inter-Module Communication
* :doc:`../vyra_base.com` - API Reference
* gRPC Documentation: https://grpc.io/docs/languages/python/
* Protocol Buffers: https://protobuf.dev/
