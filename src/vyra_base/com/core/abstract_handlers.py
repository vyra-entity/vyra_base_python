"""
Abstract Handler Interfaces

REQUIRED base classes for structured callback implementations in VYRA.
All complex handlers MUST inherit from these interfaces to ensure consistency.

These interfaces define the contract for different communication patterns:
- IServiceHandler: Request-Response handlers
- IActionHandler: Long-running task handlers with goal lifecycle
- IGoalHandle: Goal handle abstraction for action execution
"""

from abc import ABC, abstractmethod
from typing import Any, Dict


class ActionStatus:
    """
    Action goal status values (imported from types for convenience).
    Re-exported here for handler implementations.
    """
    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


class IGoalHandle(ABC):
    """
    Abstract interface for action goal handles.
    
    Provides methods for:
    - Publishing feedback during execution
    - Checking cancellation requests
    - Setting goal status (succeed/abort/cancel)
    """
    
    @abstractmethod
    def publish_feedback(self, feedback: Dict[str, Any]) -> None:
        """
        Publish feedback about goal progress.
        
        Args:
            feedback: Dictionary with progress information
        
        Example:
            goal_handle.publish_feedback({
                "progress": 50,
                "total": 100,
                "message": "Processing items..."
            })
        """
        pass
    
    @abstractmethod
    def is_cancel_requested(self) -> bool:
        """
        Check if cancellation has been requested.
        
        Returns:
            bool: True if goal should be canceled
        
        Example:
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"status": "canceled"}
        """
        pass
    
    @abstractmethod
    def succeed(self) -> None:
        """Mark goal as succeeded."""
        pass
    
    @abstractmethod
    def abort(self) -> None:
        """Mark goal as aborted (error occurred)."""
        pass
    
    @abstractmethod
    def canceled(self) -> None:
        """Mark goal as canceled (by request)."""
        pass
    
    @property
    @abstractmethod
    def goal(self) -> Any:
        """
        Get the goal request data.
        
        Returns:
            Goal request object with parameters
        """
        pass


class IServiceHandler(ABC):
    """
    Abstract interface for service (request-response) handlers.
    
    Implement this interface for services that handle single request-response
    interactions like ROS2 Services, gRPC unary calls, REST endpoints.
    
    Example:
        class CalculateHandler(IServiceHandler):
            async def handle_request(self, request):
                result = request.x + request.y
                return {"result": result}
    """
    
    @abstractmethod
    async def handle_request(self, request: Any) -> Any:
        """
        Handle incoming service request.
        
        Args:
            request: Request data/object
        
        Returns:
            Response data (dict or response object)
        
        Example:
            async def handle_request(self, request):
                # Process request
                result = process(request.data)
                # Return response
                return {"result": result, "status": "success"}
        """
        pass


class IActionHandler(ABC):
    """
    Abstract interface for action (long-running task) handlers.
    
    REQUIRED for all ActionServer implementations. Defines the three-phase
    lifecycle of action goals:
    
    1. on_goal: Accept or reject incoming goals
    2. execute: Main execution logic with feedback
    3. on_cancel: Handle cancellation requests
    
    Example:
        class BatchProcessorHandler(IActionHandler):
            async def on_goal(self, goal):
                return goal.count <= 100
            
            async def execute(self, handle):
                for i in range(handle.goal.count):
                    if handle.is_cancel_requested():
                        handle.canceled()
                        return {"processed": i}
                    process_item(i)
                    handle.publish_feedback({"progress": i+1})
                handle.succeed()
                return {"processed": handle.goal.count}
            
            async def on_cancel(self):
                return True
    """
    
    @abstractmethod
    async def on_goal(self, goal: Any) -> bool:
        """
        Called when a new goal is received.
        
        Decide whether to accept or reject the goal based on its parameters,
        current system state, resource availability, etc.
        
        Args:
            goal: Goal request data
        
        Returns:
            bool: True to accept, False to reject
        
        Example:
            async def on_goal(self, goal):
                # Check if goal is valid
                if goal.count <= 0 or goal.count > 1000:
                    logger.warning(f"Rejecting invalid goal count: {goal.count}")
                    return False
                
                # Check system resources
                if self.is_busy():
                    logger.info("Rejecting goal: system busy")
                    return False
                
                return True
        """
        pass
    
    @abstractmethod
    async def execute(self, handle: IGoalHandle) -> Dict[str, Any]:
        """
        Execute the goal and return result.
        
        This is the main execution logic. Use the goal_handle to:
        - Access goal parameters via handle.goal
        - Publish feedback via handle.publish_feedback()
        - Check cancellation via handle.is_cancel_requested()
        - Set final status via handle.succeed()/abort()/canceled()
        
        Args:
            handle: Goal handle for accessing goal data and publishing feedback
        
        Returns:
            dict: Result data to return to client
        
        Example:
            async def execute(self, handle):
                total = handle.goal.count
                
                for i in range(total):
                    # Check for cancellation
                    if handle.is_cancel_requested():
                        handle.canceled()
                        return {
                            "processed": i,
                            "status": ActionStatus.CANCELED
                        }
                    
                    # Do work
                    await self.process_item(i)
                    
                    # Publish feedback
                    handle.publish_feedback({
                        "progress": i + 1,
                        "total": total,
                        "percent": (i + 1) / total * 100
                    })
                
                # Success
                handle.succeed()
                return {
                    "processed": total,
                    "status": ActionStatus.SUCCEEDED
                }
        """
        pass
    
    @abstractmethod
    async def on_cancel(self) -> bool:
        """
        Called when cancellation is requested.
        
        Decide whether to accept the cancellation request. Some goals may not
        be cancellable (e.g., already near completion, critical operations).
        
        Returns:
            bool: True to accept cancellation, False to reject
        
        Example:
            async def on_cancel(self):
                # Check if safe to cancel
                if self.in_critical_section():
                    logger.warning("Cannot cancel: in critical section")
                    return False
                
                logger.info("Accepting cancellation request")
                return True
        """
        pass


# Type exports for convenience
__all__ = [
    'IServiceHandler',
    'IActionHandler',
    'IGoalHandle',
    'ActionStatus',
]
