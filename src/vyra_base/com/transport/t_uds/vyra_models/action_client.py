"""
UDS Action Client Implementation

Uses Unix Stream Sockets with feedback subscription for long-running goals.
"""

import logging
import socket
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable
from pathlib import Path

from vyra_base.com.core.types import VyraActionClient, ProtocolType, InterfaceType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError

logger = logging.getLogger(__name__)


class VyraActionClientImpl(VyraActionClient):
    """
    Unix Domain Socket action client using stream sockets.
    
    Pattern:
    - Connects to server at /tmp/vyra_sockets/act_{module}_{action}.sock
    - Sends goal_request, receives goal_id and feedback_socket_path
    - Optionally connects to feedback_socket for streaming updates
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        direct_response: Optional[Callable[[Any], Awaitable[None]]],
        feedback_callback: Optional[Callable[[Any], Awaitable[None]]],
        goal_response_callback: Optional[Callable[[Any], Awaitable[None]]],
        action_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            direct_response, feedback_callback, goal_response_callback,
            ProtocolType.UDS, **kwargs
        )
        self.action_type = action_type
        self._module_name = module_name
        self._socket_dir = Path("/tmp/vyra_sockets")
        self._server_socket_path = None  # Set in initialize()
        self._active_goal_id: Optional[str] = None
        self._feedback_listen_task: Optional[asyncio.Task] = None
        
    async def initialize(self) -> bool:
        """Initialize UDS action client."""
        try:
            action_name = self.topic_builder.build(self.name)
            self._server_socket_path = self._socket_dir / f"act_{self._module_name}_{action_name}.sock"
            
            if not self._server_socket_path.exists():
                logger.warning(f"Server socket not found: {self._server_socket_path}")
            
            self._initialized = True
            logger.info(f"✅ VyraActionClient '{self.name}' initialized: {self._server_socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize VyraActionClient '{self.name}': {e}")
            raise InterfaceError(f"ActionClient initialization failed: {e}")
    
    async def send_goal(self, goal: Any, timeout: float = 5.0) -> Optional[str]:
        """
        Send goal to action server.
        
        Args:
            goal: Goal message instance or dict
            timeout: Goal acceptance timeout
            
        Returns:
            goal_id on success, None on failure
        """
        loop = asyncio.get_event_loop()
        client_socket = None
        
        try:
            # Create socket
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.setblocking(False)
            
            # Connect to server
            try:
                await asyncio.wait_for(
                    loop.sock_connect(client_socket, str(self._server_socket_path)),
                    timeout=timeout
                )
            except (asyncio.TimeoutError, FileNotFoundError) as e:
                logger.error(f"❌ Connection failed: {e}")
                return None
                
            # Serialize goal
            # TODO: Support action_type.Goal serialization
            if hasattr(goal, 'to_dict'):
                goal_data = goal.to_dict()
            elif isinstance(goal, dict):
                goal_data = goal
            else:
                goal_data = {'goal': str(goal)}
                
            # Build goal request
            goal_id = str(uuid.uuid4())
            request = {
                'type': 'goal_request',
                'goal_id': goal_id,
                'goal': goal_data
            }
            
            request_bytes = json.dumps(request).encode('utf-8')
            length_prefix = len(request_bytes).to_bytes(4, byteorder='big')
            
            # Send request
            await loop.sock_sendall(client_socket, length_prefix + request_bytes)
            
            # Receive response
            try:
                length_bytes = await asyncio.wait_for(
                    loop.sock_recv(client_socket, 4),
                    timeout=timeout
                )
                if not length_bytes:
                    return None
                    
                response_length = int.from_bytes(length_bytes, byteorder='big')
                response_data = await asyncio.wait_for(
                    loop.sock_recv(client_socket, response_length),
                    timeout=timeout
                )
                
                response = json.loads(response_data.decode('utf-8'))
                
                if response.get('accepted'):
                    self._active_goal_id = response.get('goal_id', goal_id)
                    feedback_socket_path = response.get('feedback_socket')
                    
                    # Start feedback listener if callback provided
                    if self.feedback_callback and feedback_socket_path:
                        self._feedback_listen_task = asyncio.create_task(
                            self._listen_feedback(Path(feedback_socket_path))
                        )
                    
                    # Call goal_response_callback
                    if self.goal_callback:
                        await self.goal_callback(response)
                    
                    return self._active_goal_id
                else:
                    logger.warning("Goal rejected by server")
                    return None
                    
            except asyncio.TimeoutError:
                logger.error(f"❌ Goal acceptance timeout")
                return None
                
        except Exception as e:
            logger.error(f"❌ Send goal failed: {e}")
            return None
            
        finally:
            if client_socket:
                client_socket.close()
    
    async def _listen_feedback(self, feedback_socket_path: Path):
        """Listen for feedback on dedicated socket."""
        loop = asyncio.get_event_loop()
        feedback_socket = None
        
        try:
            # Wait for feedback socket to be created by server
            for _ in range(10):
                if feedback_socket_path.exists():
                    break
                await asyncio.sleep(0.1)
            else:
                logger.warning(f"Feedback socket not created: {feedback_socket_path}")
                return
                
            # Connect to feedback socket
            feedback_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            feedback_socket.setblocking(False)
            await loop.sock_connect(feedback_socket, str(feedback_socket_path))
            
            # Listen for feedback messages
            while True:
                try:
                    # Receive feedback (length prefix + data)
                    length_bytes = await loop.sock_recv(feedback_socket, 4)
                    if not length_bytes:
                        break
                        
                    message_length = int.from_bytes(length_bytes, byteorder='big')
                    message_data = await loop.sock_recv(feedback_socket, message_length)
                    
                    message = json.loads(message_data.decode('utf-8'))
                    msg_type = message.get('type')
                    
                    if msg_type == 'feedback' and self.feedback_callback:
                        await self.feedback_callback(message.get('feedback'))
                    elif msg_type == 'result' and self.direct_response_callback:
                        await self.direct_response_callback(message.get('result'))
                        break  # Goal completed
                        
                except Exception as e:
                    logger.error(f"❌ Feedback handling failed: {e}")
                    break
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"❌ Feedback listen failed: {e}")
        finally:
            if feedback_socket:
                feedback_socket.close()
    
    async def cancel_goal(self, goal_id: str, timeout: float = 5.0) -> bool:
        """
        Cancel active goal.
        
        Args:
            goal_id: ID of goal to cancel
            timeout: Cancel timeout
            
        Returns:
            True on successful cancellation
        """
        loop = asyncio.get_event_loop()
        client_socket = None
        
        try:
            # Connect to server
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.setblocking(False)
            await asyncio.wait_for(
                loop.sock_connect(client_socket, str(self._server_socket_path)),
                timeout=timeout
            )
            
            # Send cancel request
            request = {
                'type': 'cancel_request',
                'goal_id': goal_id
            }
            
            request_bytes = json.dumps(request).encode('utf-8')
            length_prefix = len(request_bytes).to_bytes(4, byteorder='big')
            await loop.sock_sendall(client_socket, length_prefix + request_bytes)
            
            # Receive response
            length_bytes = await asyncio.wait_for(
                loop.sock_recv(client_socket, 4),
                timeout=timeout
            )
            if not length_bytes:
                return False
                
            response_length = int.from_bytes(length_bytes, byteorder='big')
            response_data = await asyncio.wait_for(
                loop.sock_recv(client_socket, response_length),
                timeout=timeout
            )
            
            response = json.loads(response_data.decode('utf-8'))
            return response.get('canceled', False)
            
        except Exception as e:
            logger.error(f"❌ Cancel goal failed: {e}")
            return False
            
        finally:
            if client_socket:
                client_socket.close()
    
    async def cleanup(self):
        """Cleanup UDS resources."""
        if self._feedback_listen_task:
            self._feedback_listen_task.cancel()
            try:
                await self._feedback_listen_task
            except asyncio.CancelledError:
                pass
                
        logger.info(f"🔄 UdsActionClient cleaned up: {self.name}")
    
    @property
    def interface_type(self) -> InterfaceType:
        return InterfaceType.ACTION_CLIENT
