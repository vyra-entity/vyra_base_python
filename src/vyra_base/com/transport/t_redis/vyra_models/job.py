"""
Redis Job Implementation

Long-running task communication via Redis Pub/Sub and key-value storage.
Provides async job execution with progress feedback and cancellation support.
"""
import asyncio
import json
import logging
import uuid
from typing import Any, Callable, Optional
from enum import Enum
from datetime import datetime

from vyra_base.com.core.types import VyraJob, ProtocolType
from vyra_base.com.core.exceptions import JobError, TransportError
from vyra_base.helper.logger import Logger
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient
from vyra_base.com.transport.t_redis.vyra_models.callable import RedisCallable
from vyra_base.com.transport.t_redis.vyra_models.speaker import RedisSpeaker
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType


class JobStatus(str, Enum):
    """Job execution status."""
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class RedisJob(VyraJob):
    """
    Redis-specific implementation of VyraJob for long-running tasks.
    
    Implements job pattern using:
    - Callable for starting/cancelling jobs (control channel via Redis keys)
    - Speaker for status/progress updates (feedback channel via Pub/Sub)
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Control: job:control:<module_name>_<module_id>/<function_name>
        Feedback: job:feedback:<module_name>_<module_id>/<function_name>
        Status: job:status:<module_name>_<module_id>/<function_name>:<job_id>
    
    Example:
        >>> # Server side
        >>> async def execute_task(goal):
        ...     # Long-running task
        ...     for i in range(10):
        ...         await asyncio.sleep(1)
        ...         # Feedback is handled automatically
        ...     return {"result": "success"}
        >>> 
        >>> job = RedisJob(
        ...     "process_data",
        ...     topic_builder=builder,
        ...     redis_client=client,
        ...     result_callback=execute_task,
        ...     is_server=True
        ... )
        >>> await job.initialize()
        >>> 
        >>> # Client side
        >>> job = RedisJob(
        ...     "process_data",
        ...     topic_builder=builder,
        ...     redis_client=client,
        ...     is_server=False
        ... )
        >>> result = await job.execute({"data": "test"})
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        redis_client: RedisClient,
        result_callback: Optional[Callable[[Any], Any]] = None,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        is_server: bool = True,
        **kwargs
    ):
        """Initialize Redis Job."""
        # Apply topic builder if provided
        if topic_builder:
            name = topic_builder.build(name, interface_type=InterfaceType.JOB)
            Logger.debug(f"Applied TopicBuilder: {name}")
        
        super().__init__(
            name, topic_builder, result_callback, feedback_callback,
            ProtocolType.REDIS, **kwargs)
        
        self.redis_client = redis_client
        self.is_server = is_server
        self.topic_builder = topic_builder
        
        # Control and feedback channels
        self._control_callable: Optional[RedisCallable] = None
        self._feedback_speaker: Optional[RedisSpeaker] = None
        
        # Server-side state
        self._active_jobs: dict[str, asyncio.Task] = {}
        self._job_status: dict[str, JobStatus] = {}
        
        # Client-side state
        self._current_job_id: Optional[str] = None
        self._result_future: Optional[asyncio.Future] = None
    
    async def initialize(self) -> bool:
        """Initialize Redis job (server or client)."""
        if self._initialized:
            Logger.warning(f"RedisJob '{self.name}' already initialized")
            return True
        
        try:
            if self.is_server:
                # Server: create control callable and feedback speaker
                Logger.info(f"🔧 Creating Redis job server: {self.name}")
                
                if not self.result_callback:
                    raise JobError(f"result_callback required for job server '{self.name}'")
                
                # Control channel for start/cancel commands
                self._control_callable = RedisCallable(
                    f"job_control_{self.name}",
                    topic_builder=self.topic_builder,  # Already applied
                    callback=self._handle_control_request,
                    redis_client=self.redis_client
                )
                await self._control_callable.initialize()
                
                # Feedback channel for progress updates
                self._feedback_speaker = RedisSpeaker(
                    f"job_feedback_{self.name}",
                    topic_builder=self.topic_builder,  # Already applied
                    redis_client=self.redis_client,
                    is_publisher=True
                )
                await self._feedback_speaker.initialize()
                
                Logger.info(f"✅ Redis job server created: {self.name}")
            else:
                # Client: create control callable and feedback subscriber
                Logger.info(f"🔧 Creating Redis job client: {self.name}")
                
                # Control channel for sending commands
                self._control_callable = RedisCallable(
                    f"job_control_{self.name}",
                    topic_builder=self.topic_builder,  # Already applied
                    redis_client=self.redis_client
                )
                await self._control_callable.initialize()
                
                # Feedback channel for receiving progress
                if self.feedback_callback:
                    self._feedback_speaker = RedisSpeaker(
                        f"job_feedback_{self.name}",
                        topic_builder=self.topic_builder,  # Already applied
                        redis_client=self.redis_client,
                        callback=self._handle_feedback,
                        is_publisher=False
                    )
                    await self._feedback_speaker.initialize()
                
                Logger.info(f"✅ Redis job client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            Logger.error(f"❌ Failed to initialize RedisJob '{self.name}': {e}")
            raise JobError(f"Failed to initialize RedisJob: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup Redis job resources."""
        if not self._initialized:
            return
        
        Logger.info(f"🛑 Shutting down RedisJob: {self.name}")
        
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
        Logger.info(f"✅ RedisJob '{self.name}' shutdown complete")
    
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
            raise JobError(f"RedisJob '{self.name}' not initialized")
        
        if self.is_server:
            raise JobError(f"Cannot execute on server-side job '{self.name}'")
        
        if not self._control_callable:
            raise JobError(f"Control channel not available for job '{self.name}'")

        try:
            Logger.info(f"🚀 Executing Redis job: {self.name}")
            
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
            Logger.info(f"✅ Redis job completed: {self.name}")
            return result
            
        except Exception as e:
            Logger.error(f"❌ Redis job execution failed '{self.name}': {e}")
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
            raise JobError(f"RedisJob '{self.name}' not initialized")
        
        job_id = job_id or self._current_job_id
        if not job_id:
            raise JobError("No active job to cancel")
        
        if not self._control_callable:
            raise JobError(f"Control channel not available for job '{self.name}'")

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
            return {"status": "error", "error": "Invalid request"}
        
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
            
            # Mark as running in Redis
            status_key = f"job:status:{self.name}:{job_id}"
            await self._set_job_status(status_key, JobStatus.RUNNING)
            self._job_status[job_id] = JobStatus.RUNNING
            
            # Execute job in background task
            task = asyncio.create_task(self._execute_job(job_id, goal, status_key))
            self._active_jobs[job_id] = task
            
            # Wait for result
            result = await task
            
            # Cleanup
            await self._set_job_status(status_key, JobStatus.SUCCEEDED)
            self._job_status[job_id] = JobStatus.SUCCEEDED
            del self._active_jobs[job_id]
            
            return {"status": "success", "result": result}
            
        except asyncio.CancelledError:
            await self._set_job_status(status_key, JobStatus.CANCELLED)
            self._job_status[job_id] = JobStatus.CANCELLED
            return {"status": "cancelled"}
        except Exception as e:
            Logger.error(f"❌ Job {job_id} failed: {e}")
            await self._set_job_status(status_key, JobStatus.FAILED)
            self._job_status[job_id] = JobStatus.FAILED
            return {"status": "error", "error": str(e)}
    
    async def _execute_job(self, job_id: str, goal: Any, status_key: str) -> Any:
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
        
        status_key = f"job:status:{self.name}:{job_id}"
        await self._set_job_status(status_key, JobStatus.CANCELLED)
        self._job_status[job_id] = JobStatus.CANCELLED
        del self._active_jobs[job_id]
        
        return {"status": "cancelled"}
    
    async def _set_job_status(self, key: str, status: JobStatus) -> None:
        """Store job status in Redis."""
        data = {
            "status": status.value,
            "timestamp": datetime.now().isoformat()
        }
        await self.redis_client.set(key, json.dumps(data), ex=3600)  # Expire after 1 hour
    
    async def _publish_feedback(self, job_id: str, feedback: dict) -> None:
        """Publish job feedback (server-side)."""
        if self._feedback_speaker:
            await self._feedback_speaker.shout({
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
    
    async def get_status_from_redis(self, job_id: str) -> Optional[dict]:
        """Get job status from Redis."""
        status_key = f"job:status:{self.name}:{job_id}"
        data = await self.redis_client.get(status_key)
        if data:
            return json.loads(data)
        return None
