IPC - Inter-Process Communication
==================================

VYRA supports IPC (Inter-Process Communication) via **gRPC with Unix Domain Sockets**.
This enables fast inter-process communication **within a module**.

Concept
-------

IPC eignet sich for:

✅ **Use Cases:**

* Communication between processes in the same container/module
* Fast internal API calls without network overhead
* Decoupling of frontend and backend within the same module
* Python ↔ Python communication within a module

❌ **Not suitable for:**

* Inter-module communication (use ROS2)
* Communication across container boandaries
* External API access

.. note::
   For communication **between modules**, always use ROS2 (Job/Callable).
   IPC is only intended for communication **within a module**.

gRPC via Unix Domain Socket
-----------------------------

VYRA nutzt **Unix Domain Sockets** statt TCP for gRPC-Kommunikation:

**Advantages:**

* ⚡ Very fast (no network latency)
* 🔒 Secure (local processes only)
* 🎯 No port conflicts
* 💾 Less overhead than TCP/IP

**Socket-Pfad:** ``/tmp/<module_name>_ipc.sock``

Setup
-----------

Server-Seite (Backend)
^^^^^^^^^^^^^^^^^^^^^^

Erstellen you einen gRPC-Server with Unix Domain Socket:

.. code-block:: python

   import grpc
   from concurrent import futures
   from vyra_base.com.handler.ipc import IPCHandler
   
   # gRPC Server with Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   server = grpc.server(futures.ThreadPoolExecuto(max_workers=10))
   
   # Service register (Ihr gRPC Service)
   MyServiceServicer.add_to_server(MyServiceImpl(), server)
   
   # Unix Domain Socket binden
   server.add_insecure_port(f'unix://{socket_path}')
   server.start()
   print(f"IPC Server läuft on {socket_path}")

Client-Seite (Frontend/undere Prozesse)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Verbinden you sich with dem gRPC-Server:

.. code-block:: python

   import grpc
   
   # Verbindung via Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   channel = grpc.insecure_channel(f'unix://{socket_path}')
   
   # Client-Stub create
   stub = MyServiceStub(channel)
   
   # RPC-Aufruf
   request = MyRequest(data="Hello")
   response = stub.CallMethod(request)
   print(f"Antwort: {response.result}")

Praktisches Beispiel
--------------------

Szenario: Frontend-Backend-Kommunikation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
           # Backend-Logik
           return StatusResponse(
               status="running",
               uptime_seconds=12345
           )
       
       def ProcessData(self, request, context):
           # Datenverarattung
           result = process(request.data)
           return DataResponse(result=result)
   
   # Server starten
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

Definieren you Ihre gRPC-Services in ``.proto``-Dateien:

.. code-block:: protobuf

   syntax = "proto3";
   
   package vyra.ipc;
   
   // Service-Definition
   service ModuleService {
       rpc Initialize(InitRequest) returns (InitResponse);
       rpc ExecuteCommand(CommandRequest) returns (CommandResponse);
       rpc GetData(DataRequest) returns (DataResponse);
   }
   
   // Message-Definitionen
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

Kompilieren der Proto-Dateien:

.. code-block:: bash

   # Python gRPC Code generieren
   python -m grpc_tools.protoc \
       -I. \
       --python_out=. \
       --grpc_python_out=. \
       module_service.proto

IPCHandler-Class
-----------------

VYRA bietet eine Helper-Class for IPC-Verbindungen:

.. code-block:: python

   from vyra_base.com.handler.ipc import IPCHandler
   
   # IPC-Handler initialize
   ipc = IPCHandler(
       socket_path="/tmp/my_module_ipc.sock",
       service_stub=MyServiceStub
   )
   
   # Establish connection
   await ipc.connect()
   
   # RPC-Aufruf
   response = await ipc.call_method("GetStatus", request)
   
   # Verbindung schließen
   await ipc.disconnect()

.. note::
   Further Details to IPCHandler-Implementierung can be found you in der
   :class:`~vyra_base.com.handler.ipc.IPCHandler` API-Reference.

Performance
-----------

**Typische Latenzzeiten:**

* Unix Domain Socket: ~0.05 - 0.2 ms
* TCP localhost: ~0.2 - 1 ms
* ROS2 Service: ~1 - 5 ms

➡️ IPC is **10-100x fastr** als ROS2 for lokale Kommunikation.

Best Practices
--------------

✅ **Empfohlen:**

* Use you IPC nur innerhalb eines Moduls
* Definieren you klare Proto-Definitionen
* Implementieren you Error Handling and Timeouts
* Räumen you Socket-Dateien on Shutdown on
* Use you Connection-Pooling for frequent Calls

❌ **Vermeiden:**

* IPC for Inter-module communication (use ROS2)
* Sehr große messages (> 10 MB, use you Shared Memory)
* Blocking-Calls without Timeout
* Hardcodierte Socket-Pfade (use you Konfiguration)

Error Handling
----------------

.. code-block:: python

   import grpc
   
   try:
       channel = grpc.insecure_channel(f'unix://{socket_path}')
       stub = MyServiceStub(channel)
       
       # Mit Timeout
       response = stub.CallMethod(
           request,
           timeout=5.0  # 5 seconds Timeout
       )
   except grpc.RpcError as e:
       if e.code() == grpc.StatusCode.UNAVAILABLE:
           print("Server not erreichbar")
       elif e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
           print("Timeout")
       else:
           print(f"gRPC-Fehler: {e}")
   except Exception as e:
       print(f"Unerwarteter Fehler: {e}")

Cleanup
-------

Socket-Dateien should on Shutdown gelöscht are:

.. code-block:: python

   import os
   import signal
   
   def cleanup_socket(socket_path):
       if os.path.exists(socket_path):
           os.remove(socket_path)
           print(f"Socket {socket_path} entfernt")
   
   # Beim Shutdown onräumen
   def shutdown_handler(signum, frame):
       cleanup_socket("/tmp/my_module_ipc.sock")
       server.stop(grace_period=5)
       sys.exit(0)
   
   signal.signal(signal.SIGINT, shutdown_handler)
   signal.signal(signal.SIGTERM, shutdown_handler)

Further Information
-----------------------------

* :doc:`ros2_communication` - ROS2 for Inter-Modul-Kommunikation
* :doc:`../vyra_base.com` - API-Reference
* gRPC Dokumentation: https://grpc.io/docs/languages/python/
* Protocol Buffers: https://protobuf.dev/
