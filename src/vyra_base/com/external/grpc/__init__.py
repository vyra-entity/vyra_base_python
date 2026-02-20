"""
gRPC External Communication

Professional gRPC integration over Unix Domain Sockets.
Provides client and server implementations.
"""
from vyra_base.com.external.grpc.grpc_client import GrpcClient
from vyra_base.com.external.grpc.grpc_client import GRPC_AVAILABLE as GRPC_CLIENT_AVAILABLE
from vyra_base.com.external.grpc.grpc_server import GrpcServer
from vyra_base.com.external.grpc.grpc_server import GRPC_AVAILABLE as GRPC_SERVER_AVAILABLE

__all__ = [
    "GrpcClient",
    "GrpcServer",
    "GRPC_CLIENT_AVAILABLE",
    "GRPC_SERVER_AVAILABLE",
]
