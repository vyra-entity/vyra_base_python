"""
UDS Job Implementation

Long-running task communication over Unix Domain Sockets.
Provides async job execution with progress feedback and cancellation support.
"""
import asyncio
import json
import uuid
from pathlib import Path
from typing import Any, Callable, Optional
from enum import Enum
from datetime import datetime

from vyra_base.com.core.types import VyraJob, ProtocolType
from vyra_base.com.core.exceptions import JobError, TransportError
from vyra_base.helper.logger import Logger
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR
from vyra_base.com.transport.t_uds.vyra_models.callable import UDSCallable
from vyra_base.com.transport.t_uds.vyra_models.speaker import UDSSpeaker
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType


class JobStatus(str, Enum):
    """Job execution status."""
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class UDSJob(VyraJob):
    """
    Job interface over Unix Domain Sockets.
    
    Implements job pattern using:
    - Callable for starting/cancelling jobs (control channel)
    - Speaker for status/progress updates (feedback channel)
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Control: /tmp/vyra_uds/<module_name>_<module_id>_<function_name>_control.sock
        Feedback: /tmp/vyra_uds/<module_name>_<module_id>_<function_name>_feedback.sock
    
    Example:
        >>> # Server side
        >>> async def execute_task(goal):
        ...     # Long-running task
        ...     for i in range(10):
        ...         await asyncio.sleep(1)
        ...         # Feedback is handled automatically
        ...     return {"result": "success"}
        >>> 
        >>> job = UDSJob(
        ...     "process_data",
        ...     topic_builder=builder,
        ...     result_callback=execute_task,
        ...     is_server=True
        ... )
        >>> await job.initialize()
        >>> 
        >>> # Client side
        >>> job = UDSJob(
        ...     "process_data",
        ...     topic_builder=builder,
        ...     is_server=False
        ... )
        >>> result = await job.execute({"data": "test"})
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        result_callback: Optional[Callable[[Any], Any]] = None,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        is_server: bool = True,
        **kwargs
    ):
        """Initialize UDS Job."""
        # Apply topic builder if provided
        
        super().__init__(
            name, topic_builder, result_callback, feedback_callback,
            ProtocolType.UDS, **kwargs)
        
        self.is_server = is_server
        
        # Control and feedback channels
        self._control_callable: Optional[UDSCallable] = None
        self._feedback_speaker: Optional[UDSSpeaker] = None
        
        # Server-side state
        self._active_jobs: dict[str, asyncio.Task] = {}
        self._job_status: dict[str, JobStatus] = {}
        
        # Client-side state
        self._current_job_id: Optional[str] = None
        self._result_future: Optional[asyncio.Future] = None
    
    async def initialize(self) -> bool:
        """Initialize UDS job (server or client)."""
        if self._initialized:
            Logger.warning(f"UDSJob '{self.name}' already initialized")
            return True
        
        try:
            if self.is_server:
                # Server: create control callable and feedback speaker
                Logger.info(f"🔧 Creating UDS job server: {self.name}")
                
                if not self.result_callback:
                    raise JobError(f"result_callback required for job server '{self.name}'")
                
                # Control channel for start/cancel commands
                self._control_callable = UDSCallable(
                    f"{self.name}_control",
                    topic_builder=self.topic_builder,  # Already applied
                    callback=self._handle_control_request,
                    module_name=self.name
                )
                await self._control_callable.initialize()
                
                # Feedback channel for progress updates
                self._feedback_speaker = UDSSpeaker(
                    f"{self.name}_feedback",
                    topic_builder=self.topic_builder,  # Already applied
                    is_publisher=True
                )
                await self._feedback_speaker.initialize()
                
                Logger.info(f"✅ UDS job server created: {self.name}")
            else:
                # Client: create control callable and feedback subscriber
                Logger.info(f"🔧 Creating UDS job client: {self.name}")
                
                # Control channel for sending commands
                self._control_callable = UDSCallable(
                    f"{self.name}_control",
                    topic_builder=self.topic_builder,  # Already applied
                    module_name=self.name
                )
                await self._control_callable.initialize()
                
                # Feedback channel for receiving progress
                if self.feedback_callback:
                    self._feedback_speaker = UDSSpeaker(
                        f"{self.name}_feedback",
                        topic_builder=self.topic_builder,  # Already applied
                        callback=self._handle_feedback, # pyright: ignore[reportArgumentType]
                        is_publisher=False
                    )
                    await self._feedback_speaker.initialize()
                
                Logger.info(f"✅ UDS job client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            Logger.error(f"❌ Failed to initialize UDSJob '{self.name}': {e}")
            raise JobError(f"Failed to initialize UDSJob: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup UDS job resources."""
        if not self._initialized:
            return
        
        Logger.info(f"🛑 Shutting down UDSJob: {self.name}")
        
        # Cancel active jobs (server-side)
        for job_id, task in list(self._active_jobs.items()):
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._active_jobs.clear()
        
        # Shutdown channels
        if self._control_callable:
            await self._control_callable.shutdown()
            self._control_callable = None
        
        if self._feedback_speaker:
            await self._feedback_speaker.shutdown()
            self._feedback_speaker = None
        
        self._initialized = False
        Logger.info(f"✅ UDSJob '{self.name}' shutdown complete")
    
    async def execute(
        self,
        goal: Any,
        feedback_callback: Optional[Callable[[Any], None]] = None
    ) -> Any:
        """
        Execute the job (client-side).
        
        Args:
            goal: Job goal data
            feedback_callback: Optional callback for progress updates
            
        Returns:
            Job result
            
        Raises:
            JobError: If not initialized or execution fails
        """
        if not self._initialized:
            raise JobError(f"UDSJob '{self.name}' not initialized")
        
        if self.is_server:
            raise JobError(f"Cannot execute on server-side job '{self.name}'")
        
        if not self._control_callable:
            raise JobError("Control channel not available for job execution")

        try:
            Logger.info(f"🚀 Executing UDS job: {self.name}")
            
            # Generate job ID
            job_id = str(uuid.uuid4())
            self._current_job_id = job_id
            
            # Override feedback callback if provided
            if feedback_callback:
                self.feedback_callback = feedback_callback
            
            # Send start command via control channel
            request = {
                "command": "start",
                "job_id": job_id,
                "goal": goal,
                "timestamp": datetime.now().isoformat()
            }
            
            response = await self._control_callable.call(request, timeout=60.0)
            
            if response.get("status") != "success":
                raise JobError(f"Job start failed: {response.get('error', 'Unknown error')}")
            
            result = response.get("result")
            Logger.info(f"✅ UDS job completed: {self.name}")
            return result
            
        except Exception as e:
            Logger.error(f"❌ UDS job execution failed '{self.name}': {e}")
            raise JobError(f"Job execution failed: {e}")
    
    async def cancel(self, job_id: Optional[str] = None) -> bool:
        """
        Cancel a running job.
        
        Args:
            job_id: Job ID to cancel (uses current job if None)
            
        Returns:
            True if cancelled successfully
        """
        if not self._initialized:
            raise JobError(f"UDSJob '{self.name}' not initialized")
        
        job_id = job_id or self._current_job_id
        if not job_id:
            raise JobError("No active job to cancel")
        
        if not self._control_callable:
            raise JobError("Control channel not available for cancellation")
        
        try:
            request = {
                "command": "cancel",
                "job_id": job_id,
                "timestamp": datetime.now().isoformat()
            }
            
            response = await self._control_callable.call(request, timeout=5.0)
            return response.get("status") == "cancelled"
            
        except Exception as e:
            Logger.error(f"❌ Failed to cancel job '{job_id}': {e}")
            raise JobError(f"Cancel failed: {e}")
    
    async def _handle_control_request(self, request: dict) -> dict:
        """Handle control requests (server-side)."""
        command = request.get("command", None)
        job_id = request.get("job_id", None)

        if not command or not job_id:
            Logger.warning("Received invalid control request: missing command or job_id")
            return {"status": "error", "error": "Invalid control request: missing command or job_id"}
        
        if command == "start":
            return await self._start_job(job_id, request.get("goal"))
        elif command == "cancel":
            return await self._cancel_job(job_id)
        else:
            return {"status": "error", "error": f"Unknown command: {command}"}
    
    async def _start_job(self, job_id: str, goal: Any) -> dict:
        """Start a new job (server-side)."""
        try:
            Logger.info(f"🏃 Starting job {job_id}: {self.name}")
            
            # Mark as running
            self._job_status[job_id] = JobStatus.RUNNING
            
            # Execute job in background task
            task = asyncio.create_task(self._execute_job(job_id, goal))
            self._active_jobs[job_id] = task
            
            # Wait for result
            result = await task
            
            # Cleanup
            self._job_status[job_id] = JobStatus.SUCCEEDED
            del self._active_jobs[job_id]
            
            return {"status": "success", "result": result}
            
        except asyncio.CancelledError:
            self._job_status[job_id] = JobStatus.CANCELLED
            return {"status": "cancelled"}
        except Exception as e:
            Logger.error(f"❌ Job {job_id} failed: {e}")
            self._job_status[job_id] = JobStatus.FAILED
            return {"status": "error", "error": str(e)}
    
    async def _execute_job(self, job_id: str, goal: Any) -> Any:
        """Execute job and publish feedback."""
        # Execute user callback

        if not self.result_callback:
            raise JobError("No result_callback defined for job execution")
        
        if asyncio.iscoroutinefunction(self.result_callback):
            result = await self.result_callback(goal)
        else:
            result = self.result_callback(goal)
        
        # Publish final status
        await self._publish_feedback(job_id, {
            "status": JobStatus.SUCCEEDED,
            "progress": 100,
            "message": "Job completed"
        })
        
        return result
    
    async def _cancel_job(self, job_id: str) -> dict:
        """Cancel a running job (server-side)."""
        if job_id not in self._active_jobs:
            return {"status": "error", "error": "Job not found"}
        
        task = self._active_jobs[job_id]
        task.cancel()
        
        try:
            await task
        except asyncio.CancelledError:
            pass
        
        self._job_status[job_id] = JobStatus.CANCELLED
        del self._active_jobs[job_id]
        
        return {"status": "cancelled"}
    
    async def _publish_feedback(self, job_id: str, feedback: dict) -> None:
        """Publish job feedback (server-side)."""
        if self._feedback_speaker:
            await self._feedback_speaker.publish({
                "job_id": job_id,
                "timestamp": datetime.now().isoformat(),
                **feedback
            })
    
    async def _handle_feedback(self, feedback: dict) -> None:
        """Handle feedback messages (client-side)."""
        if self.feedback_callback:
            if asyncio.iscoroutinefunction(self.feedback_callback):
                await self.feedback_callback(feedback)
            else:
                self.feedback_callback(feedback)
    
    def get_status(self, job_id: str) -> Optional[JobStatus]:
        """Get status of a job (server-side only)."""
        return self._job_status.get(job_id)
