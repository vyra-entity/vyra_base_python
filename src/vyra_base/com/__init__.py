"""
VYRA Base Communication Module (COM)

Provides ROS2 communication (Services, Topics, Actions),
IPC over gRPC/Unix Domain Sockets, and automatic Feeders.

Public API for external developers
"""

# ROS2 Communication - Datalayer
from vyra_base.com.datalayer.interface_factory import (
    create_vyra_job,          # Service Client (calls services)
    create_vyra_callable,      # Service Server (provides services)
    create_vyra_speaker,       # Publisher (publishes topics)
    remote_callable,           # Decorator for automatic service registration
    DataSpace                  # Interface registry
)

from vyra_base.com.datalayer.callable import VyraCallable, VyraCallableExecutor
from vyra_base.com.datalayer.speaker import VyraSpeaker, VyraSpeakerListener
from vyra_base.com.datalayer.publisher import VyraPublisher
from vyra_base.com.datalayer.subscriber import VyraSubscriber
from vyra_base.com.datalayer.node import VyraNode, CheckerNode, NodeSettings
from vyra_base.com.datalayer.action_client import VyraActionClient
from vyra_base.com.datalayer.action_server import VyraActionServer

# Feeders - Automatic data publication
from vyra_base.com.feeder.feeder import BaseFeeder
from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.com.feeder.news_feeder import NewsFeeder
from vyra_base.com.feeder.error_feeder import ErrorFeeder

# IPC - Inter-Process Communication
from vyra_base.com.handler.ipc import GrpcUdsServer, GrpcUdsClient

__all__ = [
    # Factory functions
    "create_vyra_job",
    "create_vyra_callable",
    "create_vyra_speaker",
    "remote_callable",
    "DataSpace",
    
    # ROS2 Classes
    "VyraCallable",
    "VyraCallableExecutor",
    "VyraSpeaker",
    "VyraSpeakerListener",
    "VyraPublisher",
    "VyraSubscriber",
    "VyraNode",
    "CheckerNode",
    "NodeSettings",
    "VyraActionClient",
    "VyraActionServer",
    
    # Feeders
    "BaseFeeder",
    "StateFeeder",
    "NewsFeeder",
    "ErrorFeeder",
    
    # IPC
    "GrpcUdsServer",
    "GrpcUdsClient",
]
