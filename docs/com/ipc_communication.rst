IPC - Inter-Process Communication
==================================

VYRA unterstützt IPC (Inter-Process Communication) über **gRPC mit Unix Domain Sockets**.
Dies ermöglicht schnelle prozessübergreifende Kommunikation **innerhalb eines Moduls**.

Konzept
-------

IPC eignet sich für:

✅ **Anwendungsfälle:**

* Kommunikation zwischen Prozessen im selben Container/Modul
* Schnelle interne API-Calls ohne Netzwerk-Overhead
* Entkopplung von Frontend und Backend im selben Modul
* Python ↔ Python Kommunikation innerhalb eines Moduls

❌ **Nicht geeignet für:**

* Inter-Modul-Kommunikation (nutzen Sie ROS2)
* Kommunikation über Container-Grenzen hinweg
* Externe API-Zugriffe

.. note::
   Für Kommunikation **zwischen Modulen** nutzen Sie immer ROS2 (Job/Callable).
   IPC ist nur für Kommunikation **innerhalb eines Moduls** gedacht.

gRPC über Unix Domain Socket
-----------------------------

VYRA nutzt **Unix Domain Sockets** statt TCP für gRPC-Kommunikation:

**Vorteile:**

* ⚡ Sehr schnell (keine Netzwerk-Latenz)
* 🔒 Sicher (nur lokale Prozesse)
* 🎯 Keine Port-Konflikte
* 💾 Weniger Overhead als TCP/IP

**Socket-Pfad:** ``/tmp/<module_name>_ipc.sock``

Einrichtung
-----------

Server-Seite (Backend)
^^^^^^^^^^^^^^^^^^^^^^

Erstellen Sie einen gRPC-Server mit Unix Domain Socket:

.. code-block:: python

   import grpc
   from concurrent import futures
   from vyra_base.com.handler.ipc import IPCHandler
   
   # gRPC Server mit Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
   
   # Service registrieren (Ihr gRPC Service)
   MyServiceServicer.add_to_server(MyServiceImpl(), server)
   
   # Unix Domain Socket binden
   server.add_insecure_port(f'unix://{socket_path}')
   server.start()
   print(f"IPC Server läuft auf {socket_path}")

Client-Seite (Frontend/andere Prozesse)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Verbinden Sie sich mit dem gRPC-Server:

.. code-block:: python

   import grpc
   
   # Verbindung über Unix Domain Socket
   socket_path = "/tmp/my_module_ipc.sock"
   channel = grpc.insecure_channel(f'unix://{socket_path}')
   
   # Client-Stub erstellen
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
           # Datenverarbeitung
           result = process(request.data)
           return DataResponse(result=result)
   
   # Server starten
   def start_ipc_server():
       socket_path = "/tmp/my_module_ipc.sock"
       server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
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
   
   # Verwendung
   client = FrontendClient()
   status = client.get_backend_status()
   print(f"Backend-Status: {status}")

gRPC Proto-Definitionen
-----------------------

Definieren Sie Ihre gRPC-Services in ``.proto``-Dateien:

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

IPCHandler-Klasse
-----------------

VYRA bietet eine Helper-Klasse für IPC-Verbindungen:

.. code-block:: python

   from vyra_base.com.handler.ipc import IPCHandler
   
   # IPC-Handler initialisieren
   ipc = IPCHandler(
       socket_path="/tmp/my_module_ipc.sock",
       service_stub=MyServiceStub
   )
   
   # Verbindung aufbauen
   await ipc.connect()
   
   # RPC-Aufruf
   response = await ipc.call_method("GetStatus", request)
   
   # Verbindung schließen
   await ipc.disconnect()

.. note::
   Weitere Details zur IPCHandler-Implementierung finden Sie in der
   :class:`~vyra_base.com.handler.ipc.IPCHandler` API-Referenz.

Performance
-----------

**Typische Latenzzeiten:**

* Unix Domain Socket: ~0.05 - 0.2 ms
* TCP localhost: ~0.2 - 1 ms
* ROS2 Service: ~1 - 5 ms

➡️ IPC ist **10-100x schneller** als ROS2 für lokale Kommunikation.

Best Practices
--------------

✅ **Empfohlen:**

* Verwenden Sie IPC nur innerhalb eines Moduls
* Definieren Sie klare Proto-Definitionen
* Implementieren Sie Fehlerbehandlung und Timeouts
* Räumen Sie Socket-Dateien beim Shutdown auf
* Nutzen Sie Connection-Pooling für häufige Calls

❌ **Vermeiden:**

* IPC für Inter-Modul-Kommunikation (nutzen Sie ROS2)
* Sehr große Nachrichten (> 10 MB, nutzen Sie Shared Memory)
* Blocking-Calls ohne Timeout
* Hardcodierte Socket-Pfade (nutzen Sie Konfiguration)

Fehlerbehandlung
----------------

.. code-block:: python

   import grpc
   
   try:
       channel = grpc.insecure_channel(f'unix://{socket_path}')
       stub = MyServiceStub(channel)
       
       # Mit Timeout
       response = stub.CallMethod(
           request,
           timeout=5.0  # 5 Sekunden Timeout
       )
   except grpc.RpcError as e:
       if e.code() == grpc.StatusCode.UNAVAILABLE:
           print("Server nicht erreichbar")
       elif e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
           print("Timeout")
       else:
           print(f"gRPC-Fehler: {e}")
   except Exception as e:
       print(f"Unerwarteter Fehler: {e}")

Cleanup
-------

Socket-Dateien sollten beim Shutdown gelöscht werden:

.. code-block:: python

   import os
   import signal
   
   def cleanup_socket(socket_path):
       if os.path.exists(socket_path):
           os.remove(socket_path)
           print(f"Socket {socket_path} entfernt")
   
   # Beim Shutdown aufräumen
   def shutdown_handler(signum, frame):
       cleanup_socket("/tmp/my_module_ipc.sock")
       server.stop(grace_period=5)
       sys.exit(0)
   
   signal.signal(signal.SIGINT, shutdown_handler)
   signal.signal(signal.SIGTERM, shutdown_handler)

Weiterführende Informationen
-----------------------------

* :doc:`ros2_communication` - ROS2 für Inter-Modul-Kommunikation
* :doc:`../vyra_base.com` - API-Referenz
* gRPC Dokumentation: https://grpc.io/docs/languages/python/
* Protocol Buffers: https://protobuf.dev/
