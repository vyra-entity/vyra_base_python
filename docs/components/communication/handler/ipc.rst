gRPC over Unix Domain Socket (UDS) Base Handler
===============================================

This module provides the generic foundation for gRPC communication over Unix Domain Sockets.
It supports all gRPC communication patterns:
- Unary RPC (request-response)
- Server-Side Streaming (one request, multiple responses)
- Client-Side Streaming (multiple requests, one response)
- Bidirectional Streaming (multiple requests, multiple responses)

**Usage:**

Server:
   server = GrpcServer(target="/tmp/my_service.sock")
   server.add_service(MyServiceServicer())
   await server.start()

Client:
   client = GrpcClient(target="/tmp/my_service.sock")
   await client.connect()
   response = await client.call_unary("method_name", request)
