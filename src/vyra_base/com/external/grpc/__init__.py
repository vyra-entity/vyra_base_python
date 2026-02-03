"""
gRPC External Communication

Professional gRPC integration over Unix Domain Sockets.
Provides client and server implementations.
"""
from vyra_base.com.external.grpc.grpc_client import GrpcClient
from vyra_base.com.external.grpc.grpc_server import GrpcServer

__all__ = [
    "GrpcClient",
    "GrpcServer"
]
