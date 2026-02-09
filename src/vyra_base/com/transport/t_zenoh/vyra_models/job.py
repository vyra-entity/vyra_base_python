"""
Zenoh Job Implementation

Concrete implementation of VyraJob for Zenoh long-running task communication.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Optional
from enum import Enum

from vyra_base.com.core.types import VyraJob, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
from vyra_base.com.transport.t_zenoh.session import ZenohSession
from vyra_base.com.transport.t_zenoh.vyra_models.speaker import ZenohSpeaker
from vyra_base.com.transport.t_zenoh.vyra_models.callable import ZenohCallable
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType

logger = logging.getLogger(__name__)


class JobStatus(str, Enum):
    """Job execution status."""
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ZenohJob(VyraJob):
    """
    Zenoh-specific implementation of VyraJob for long-running tasks.
    
    Implements job pattern using:
    - Callable for starting/cancelling jobs
    - Speaker for status/progress updates
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Example: v2_modulemanager_abc123/execute_task
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        session: Optional[ZenohSession] = None,
        result_callback: Optional[Callable[[Any], Any]] = None,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        format: SerializationFormat = SerializationFormat.JSON,
        is_server: bool = True,
        **kwargs
    ):
        """
        Initialize Zenoh job.
        
        Args:
            name: Job name (base key expression)
            session: Zenoh session
            result_callback: Async callback executed when job completes, returns result
            feedback_callback: Async callback executed during job execution for progress updates
            format: Serialization format
            is_server: Whether this is a server (True) or client (False)
            topic_builder: Optional TopicBuilder for naming convention
            **kwargs: Additional parameters
        """
        # Apply topic builder if provided
        if topic_builder:
            name = topic_builder.build(name, interface_type=InterfaceType.JOB)
            logger.debug(f"Applied TopicBuilder: {name}")
        
        super().__init__(
            name, topic_builder, result_callback, feedback_callback, 
            ProtocolType.ZENOH, **kwargs)
        
        self.session = session
        self.format = format
        self.is_server = is_server
        
        # Job control via callable (start/cancel)
        self._control_callable: Optional[ZenohCallable] = None
        
        # Feedback updates via speaker (published during execution)
        self._feedback_speaker: Optional[ZenohSpeaker] = None
        
        # Result updates via speaker (published when completed)
        self._result_speaker: Optional[ZenohSpeaker] = None
        
        # Internal state
        self._status = JobStatus.IDLE
        self._current_task: Optional[asyncio.Task] = None
        self._result: Any = None
    
    async def initialize(self) -> bool:
        """
        Initialize Zenoh job interfaces.
        
        Creates control callable and status speaker.
        """
        if self._initialized:
            logger.warning(f"ZenohJob '{self.name}' already initialized")
            return True
        
        if not self.session or not self.session.is_open:
            raise InterfaceError("Zenoh session is required and must be open")
        
        try:
            if self.is_server:
                # Server side - create control callable
                logger.info(f"🔧 Creating Zenoh job server: {self.name}")
                
                if not self.result_callback:
                    raise InterfaceError("result_callback is required for Zenoh job server")
                
                # Control callable for start/cancel
                self._control_callable = ZenohCallable(
                    name=f"{self.name}/control",
                    session=self.session,
                    callback=self._handle_control,
                    format=self.format,
                    is_server=True
                )
                await self._control_callable.initialize()
                
                # Feedback speaker for progress updates (published during execution)
                self._feedback_speaker = ZenohSpeaker(
                    name=f"{self.name}/feedback",
                    session=self.session,
                    format=self.format,
                    is_publisher=True
                )
                await self._feedback_speaker.initialize()
                
                # Result speaker for final result (published when completed)
                self._result_speaker = ZenohSpeaker(
                    name=f"{self.name}/result",
                    session=self.session,
                    format=self.format,
                    is_publisher=True
                )
                await self._result_speaker.initialize()
                
                logger.info(f"✅ Zenoh job server created: {self.name}")
            else:
                # Client side
                logger.info(f"🔧 Creating Zenoh job client: {self.name}")
                
                # Control callable for sending commands
                self._control_callable = ZenohCallable(
                    name=f"{self.name}/control",
                    session=self.session,
                    format=self.format,
                    is_server=False
                )
                await self._control_callable.initialize()
                
                # Feedback speaker for receiving progress updates (subscribed during execution)
                self._feedback_speaker = ZenohSpeaker(
                    name=f"{self.name}/feedback",
                    session=self.session,
                    format=self.format,
                    is_publisher=False
                )
                await self._feedback_speaker.initialize()
                
                # Result speaker for receiving final result (subscribed when completed)
                self._result_speaker = ZenohSpeaker(
                    name=f"{self.name}/result",
                    session=self.session,
                    format=self.format,
                    is_publisher=False
                )
                await self._result_speaker.initialize()
                
                logger.info(f"✅ Zenoh job client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohJob '{self.name}': {e}")
            
            # Cleanup on error
            if self._control_callable:
                await self._control_callable.shutdown()
                self._control_callable = None
            
            if self._feedback_speaker:
                await self._feedback_speaker.shutdown()
                self._feedback_speaker = None
            
            if self._result_speaker:
                await self._result_speaker.shutdown()
                self._result_speaker = None
            
            raise
    
    async def shutdown(self) -> None:
        """Cleanup Zenoh job resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down ZenohJob: {self.name}")
        
        # Cancel running task
        if self._current_task and not self._current_task.done():
            self._current_task.cancel()
            try:
                await self._current_task
            except asyncio.CancelledError:
                pass
        
        # Cleanup callable and speakers
        if self._control_callable:
            await self._control_callable.shutdown()
            self._control_callable = None
        
        if self._feedback_speaker:
            await self._feedback_speaker.shutdown()
            self._feedback_speaker = None
        
        if self._result_speaker:
            await self._result_speaker.shutdown()
            self._result_speaker = None
        
        self._initialized = False
        logger.info(f"✅ ZenohJob '{self.name}' shutdown complete")
    
    async def _handle_control(self, request: Any) -> Any:
        """
        Handle job control requests (server-side).
        
        Args:
            request: Control request {"command": "start"|"cancel", "params": ...}
            
        Returns:
            Control response
        """
        try:
            command = request.get("command")
            params = request.get("params", {})
            
            if command == "start":
                return await self._start_job(params)
            elif command == "cancel":
                return await self._cancel_job()
            else:
                return {"status": "error", "message": f"Unknown command: {command}"}
                
        except Exception as e:
            logger.error(f"Error handling job control: {e}")
            return {"status": "error", "message": str(e)}
    
    async def _start_job(self, params: Any) -> Any:
        """Start job execution (server-side)."""
        if self._status == JobStatus.RUNNING:
            return {"status": "error", "message": "Job already running"}
        
        logger.info(f"▶️  Starting Zenoh job: {self.name}")
        self._status = JobStatus.RUNNING
        
        # Execute job in background task
        self._current_task = asyncio.create_task(self._execute_job(params))
        
        return {"status": "started", "job": self.name}
    
    async def _cancel_job(self) -> Any:
        """Cancel job execution (server-side)."""
        if self._status != JobStatus.RUNNING:
            return {"status": "error", "message": "Job not running"}
        
        logger.info(f"⏸️  Cancelling Zenoh job: {self.name}")
        
        if self._current_task:
            self._current_task.cancel()
        
        self._status = JobStatus.CANCELLED
        await self._publish_result()
        
        return {"status": "cancelled", "job": self.name}
    
    async def _execute_job(self, params: Any) -> None:
        """Execute job callback and handle result."""
        try:
            # Execute job callback with feedback support
            if self.result_callback:
                if asyncio.iscoroutinefunction(self.result_callback):
                    self._result = await self.result_callback(params, self._publish_feedback)
                else:
                    self._result = self.result_callback(params, self._publish_feedback)
            else:
                logger.warning(f"No result_callback defined for job: {self.name}")
                self._result = {"error": "No callback defined"}
            
            self._status = JobStatus.SUCCEEDED
            logger.info(f"✅ Zenoh job completed: {self.name}")
            
        except asyncio.CancelledError:
            self._status = JobStatus.CANCELLED
            logger.info(f"⏸️  Zenoh job cancelled: {self.name}")
            
        except Exception as e:
            self._status = JobStatus.FAILED
            self._result = {"error": str(e)}
            logger.error(f"❌ Zenoh job failed: {self.name}: {e}")
        
        finally:
            # Publish final result
            await self._publish_result()
    
    async def _publish_feedback(self, feedback_data: Any) -> None:
        """
        Publish job feedback during execution.
        
        Called by result_callback to provide progress updates.
        
        Args:
            feedback_data: Feedback data (e.g., progress percentage, status message)
        """
        if self._feedback_speaker and self._feedback_speaker._initialized:
            feedback_msg = {
                "job": self.name,
                "feedback": feedback_data,
                "status": self._status.value
            }
            await self._feedback_speaker.shout(feedback_msg)
            logger.debug(f"📊 Published feedback for job: {self.name}")
            
            # Call user-provided feedback_callback if exists
            if self.feedback_callback:
                if asyncio.iscoroutinefunction(self.feedback_callback):
                    await self.feedback_callback(feedback_data)
                else:
                    self.feedback_callback(feedback_data)
    
    async def _publish_result(self) -> None:
        """Publish final job result when completed."""
        if self._result_speaker and self._result_speaker._initialized:
            result_msg = {
                "job": self.name,
                "status": self._status.value,
                "result": self._result
            }
            await self._result_speaker.shout(result_msg)
            logger.info(f"✅ Published result for job: {self.name}")
    
    async def start(self, params: Any = None) -> Any:
        """
        Start job execution (client-side).
        
        Args:
            params: Job parameters
            
        Returns:
            Start response
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohJob '{self.name}' not initialized")
        
        if not self._control_callable:
            raise InterfaceError("Control callable not available")
        
        logger.info(f"▶️  Starting job via Zenoh: {self.name}")
        
        request = {"command": "start", "params": params or {}}
        return await self._control_callable.call(request)
    
    async def cancel(self) -> Any:
        """
        Cancel job execution (client-side).
        
        Returns:
            Cancel response
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohJob '{self.name}' not initialized")
        
        if not self._control_callable:
            raise InterfaceError("Control callable not available")
        
        logger.info(f"⏸️  Cancelling job via Zenoh: {self.name}")
        
        request = {"command": "cancel"}
        return await self._control_callable.call(request)
    
    async def listen_feedback(self, callback: Callable[[Any], None]) -> None:
        """
        Listen for job feedback updates during execution (client-side).
        
        Args:
            callback: Callback for feedback updates
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohJob '{self.name}' not initialized")
        
        if not self._feedback_speaker:
            raise InterfaceError("Feedback speaker not available")
        
        await self._feedback_speaker.listen(callback)
        logger.info(f"👂 Listening to feedback for job: {self.name}")
    
    async def listen_result(self, callback: Callable[[Any], None]) -> None:
        """
        Listen for job result when completed (client-side).
        
        Args:
            callback: Callback for result updates
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohJob '{self.name}' not initialized")
        
        if not self._result_speaker:
            raise InterfaceError("Result speaker not available")
        
        await self._result_speaker.listen(callback)
        logger.info(f"👂 Listening to result for job: {self.name}")
    
    def get_status(self) -> JobStatus:
        """Get current job status (server-side)."""
        return self._status
    
    def get_result(self) -> Any:
        """Get job result (server-side)."""
        return self._result
