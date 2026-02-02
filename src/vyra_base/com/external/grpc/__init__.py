"""
gRPC External Communication

Professional gRPC integration over Unix Domain Sockets.
Wraps existing com/handler/ipc.py with provider pattern.
"""
from vyra_base.com.external.grpc.provider import GrpcProvider

__all__ = [
    "GrpcProvider",
]
