"""
UDS Action Server Implementation

Uses Unix Stream Sockets with state tracking for long-running goals.
"""

import logging
import socket
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable, Dict
from pathlib import Path
from enum import Enum

from vyra_base.com.core.types import VyraActionServer, ProtocolType, GoalHandle
from vyra_base.com.core.topic_builder import InterfaceType, TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR

logger = logging.getLogger(__name__)


class GoalStatus(Enum):
    """Action goal execution status."""
    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    SUCCEEDED = 3
    ABORTED = 4
    CANCELED = 5


class VyraActionServerImpl(VyraActionServer):
    """
    Unix Domain Socket action server using stream sockets.
    
    Pattern:
    - Main socket: {VYRA_SOCKET_DIR}/{action}.sock
    - Receives: goal_request | cancel_request | feedback_subscribe
    - Per Goal: Creates dedicated feedback socket at {module}_{action}_{goal_id}_feedback.sock
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        handle_goal_request: Callable[[Any], Awaitable[bool]],
        handle_cancel_request: Callable[[Any], Awaitable[bool]],
        execution_callback: Callable[[Any], Awaitable[Any]],
        action_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            handle_goal_request, handle_cancel_request, execution_callback,
            ProtocolType.UDS, **kwargs
        )
        self._action_type = action_type
        self._module_name = module_name
        self._socket: Optional[socket.socket] = None
        self._socket_dir = UDS_SOCKET_DIR
        self._socket_path = None  # Set in initialize()
        self._server_task: Optional[asyncio.Task] = None
        self._active_goals: Dict[str, asyncio.Task] = {}  # goal_id -> execution_task
        self._goal_handles: Dict[str, GoalHandle] = {}   # goal_id -> GoalHandle
        self._goal_states: Dict[str, GoalStatus] = {}  # goal_id -> status
        
    async def initialize(self) -> bool:
        """Initialize UDS action server."""
        try:
            self.action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            # Sanitize for filesystem use (topic names may contain '/')
            self._safe_name = self.action_name.replace("/", "_")
            self._socket_path = self._socket_dir / f"{self._safe_name}.sock"
            
            # Create socket directory
            self._socket_dir.mkdir(parents=True, exist_ok=True)
            
            # Create stream socket
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.setblocking(False)
            
            # Remove old socket file if exists
            if self._socket_path.exists():
                self._socket_path.unlink()
                
            # Bind and listen
            self._socket.bind(str(self._socket_path))
            self._socket.listen(10)
            
            self._initialized = True
            logger.info(f"✅ VyraActionServer '{self.name}' initialized: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize VyraActionServer '{self.name}': {e}")
            raise InterfaceError(f"ActionServer initialization failed: {e}")
    
    async def serve(self) -> bool:
        """Start serving action requests."""
        if not self._socket:
            logger.error("Server not initialized")
            return False
            
        try:
            self._server_task = asyncio.create_task(self._accept_loop())
            logger.info(f"📡 UdsActionServer serving on: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Serve failed: {e}")
            return False
    
    async def _accept_loop(self):
        """Accept incoming connections."""
        try:
            loop = asyncio.get_event_loop()
            
            if not self._socket:
                logger.error("Server socket not initialized")
                return
            
            while True:
                try:
                    client_socket, _ = await loop.sock_accept(self._socket)
                    asyncio.create_task(self._handle_client(client_socket))
                except Exception as e:
                    logger.error(f"❌ Accept failed: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"❌ Accept loop failed: {e}")
    
    async def _handle_client(self, client_socket: socket.socket):
        """Handle client request (goal/cancel/feedback_subscribe)."""
        loop = asyncio.get_event_loop()
        
        try:
            # Receive message (length prefix + data)
            length_bytes = await loop.sock_recv(client_socket, 4)
            if not length_bytes:
                return
                
            message_length = int.from_bytes(length_bytes, byteorder='big')
            message_data = await loop.sock_recv(client_socket, message_length)
            
            # Parse message
            message = json.loads(message_data.decode('utf-8'))
            msg_type = message.get('type')
            
            if msg_type == 'goal_request':
                await self._handle_goal_request(client_socket, message)
            elif msg_type == 'cancel_request':
                await self._handle_cancel_request(client_socket, message)
            elif msg_type == 'status_query':
                await self._handle_status_query(client_socket, message)
            else:
                logger.warning(f"Unknown message type: {msg_type}")
                
        except Exception as e:
            logger.error(f"❌ Client handling failed: {e}")
        finally:
            client_socket.close()
    
    async def _handle_goal_request(self, client_socket: socket.socket, message: dict):
        """Handle incoming goal request."""
        try:
            goal_data = message.get('goal')
            goal_id = message.get('goal_id', str(uuid.uuid4()))
            
            # TODO: Deserialize goal to action_type.Goal
            
            # Call handle_goal callback
            if self.handle_goal_request:
                accepted = await self.handle_goal_request(goal_data)
            else:
                accepted = True
            
            if accepted:
                # Set initial state
                self._goal_states[goal_id] = GoalStatus.ACCEPTED
                
                # Start execution task
                execution_task = asyncio.create_task(
                    self._execute_goal(goal_id, goal_data)
                )
                self._active_goals[goal_id] = execution_task
                
                # Send acceptance response
                response = {
                    'goal_id': goal_id,
                    'accepted': True,
                    'feedback_socket': str(self._socket_dir / f"{self._module_name}_{self._safe_name}_{goal_id}_feedback.sock")
                }
            else:
                response = {'goal_id': goal_id, 'accepted': False}
                
            # Send response
            response_bytes = json.dumps(response).encode('utf-8')
            length_prefix = len(response_bytes).to_bytes(4, byteorder='big')
            await asyncio.get_event_loop().sock_sendall(
                client_socket, length_prefix + response_bytes
            )
            
        except Exception as e:
            logger.error(f"❌ Goal request handling failed: {e}")
    
    async def _handle_cancel_request(self, client_socket: socket.socket, message: dict):
        """Handle cancel request."""
        try:
            goal_id = message.get('goal_id')
            
            if goal_id in self._active_goals:
                goal_handle = self._goal_handles.get(goal_id)
                # Signal the executing coroutine that cancel was requested
                if goal_handle:
                    goal_handle.request_cancel()
                # TODO: Call handle_cancel callback
                if self.handle_cancel_request:
                    arg = goal_handle if goal_handle else goal_id
                    canceled = await self.handle_cancel_request(arg)
                else:
                    canceled = True
                
                if canceled:
                    task = self._active_goals[goal_id]
                    task.cancel()
                    self._goal_states[goal_id] = GoalStatus.CANCELED
                    response = {'goal_id': goal_id, 'canceled': True}
                else:
                    response = {'goal_id': goal_id, 'canceled': False}
            else:
                response = {'goal_id': goal_id, 'canceled': False, 'error': 'Goal not found'}
                
            # Send response
            response_bytes = json.dumps(response).encode('utf-8')
            length_prefix = len(response_bytes).to_bytes(4, byteorder='big')
            await asyncio.get_event_loop().sock_sendall(
                client_socket, length_prefix + response_bytes
            )
            
        except Exception as e:
            logger.error(f"❌ Cancel request handling failed: {e}")
    
    async def _handle_status_query(self, client_socket: socket.socket, message: dict):
        """Handle status query."""
        try:
            goal_id = message.get('goal_id')

            if isinstance(goal_id, str):
                status = self._goal_states.get(goal_id, GoalStatus.UNKNOWN)
            else:
                status = GoalStatus.UNKNOWN
            
            response = {
                'goal_id': goal_id,
                'status': status.name
            }
            
            response_bytes = json.dumps(response).encode('utf-8')
            length_prefix = len(response_bytes).to_bytes(4, byteorder='big')
            await asyncio.get_event_loop().sock_sendall(
                client_socket, length_prefix + response_bytes
            )
            
        except Exception as e:
            logger.error(f"❌ Status query handling failed: {e}")
    
    async def _execute_goal(self, goal_id: str, goal: Any):
        """Execute goal in background."""
        try:
            self._goal_states[goal_id] = GoalStatus.EXECUTING
            
            if self.execution_callback is None:
                logger.warning("No execution callback defined, marking goal as succeeded")
                self._goal_states[goal_id] = GoalStatus.SUCCEEDED
                return
            
            # Create GoalHandle so the execution_callback can publish feedback
            goal_handle = GoalHandle(
                goal_id=goal_id,
                goal=goal,
                feedback_fn=self.publish_feedback
            )
            self._goal_handles[goal_id] = goal_handle
            result = await self.execution_callback(goal_handle)
            
            self._goal_states[goal_id] = GoalStatus.SUCCEEDED
            
            # TODO: Broadcast result via feedback socket
            
        except asyncio.CancelledError:
            self._goal_states[goal_id] = GoalStatus.CANCELED
            raise
        except Exception as e:
            logger.error(f"❌ Goal execution failed: {e}")
            self._goal_states[goal_id] = GoalStatus.ABORTED
        finally:
            self._active_goals.pop(goal_id, None)
            self._goal_handles.pop(goal_id, None)
    
    async def publish_feedback(self, goal_id: str, feedback: Any):
        """
        Publish feedback for active goal.
        
        TODO: Broadcast to subscribers via dedicated feedback socket.
        """
        logger.debug(f"Feedback for {goal_id}: {feedback}")
    
    async def cleanup(self):
        """Cleanup UDS resources."""
        # Cancel all active goals
        for task in self._active_goals.values():
            task.cancel()
        self._active_goals.clear()
        self._goal_handles.clear()
        
        if self._server_task:
            self._server_task.cancel()
            
        if self._socket:
            self._socket.close()
            
        if self._socket_path and self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except Exception as e:
                logger.warning(f"Failed to remove socket file: {e}")
                
        logger.info(f"🔄 UdsActionServer cleaned up: {self._socket_path}")
    
    @property
    def interface_type(self) -> InterfaceType:
        return InterfaceType.ACTION_SERVER
