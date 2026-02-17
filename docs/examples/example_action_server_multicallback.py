"""
Example: ActionServer with Multi-Callback Pattern

This example demonstrates the new multi-callback pattern for ActionServer,
which separates goal acceptance, cancellation handling, and execution logic
into distinct methods for better code organization.

Key Features:
- Separate callbacks for on_goal, on_cancel, and execute
- Clean separation of concerns
- Better testability and maintainability
- Follows IActionHandler interface (REQUIRED)

Usage Pattern:
    @remote_actionServer.on_goal(name="action_name")
    async def accept_goal(self, goal_request) -> bool: ...
    
    @remote_actionServer.on_cancel(name="action_name")
    async def handle_cancel(self, goal_handle) -> bool: ...
    
    @remote_actionServer.execute(name="action_name")
    async def execute(self, goal_handle) -> Dict: ...
"""

import asyncio
import logging
from typing import Dict, Any

from vyra_base.state import OperationalStateMachine
from vyra_base.com.core import remote_actionServer, IActionHandler, IGoalHandle
from vyra_base.com.core.decorators import bind_decorated_callbacks

logger = logging.getLogger(__name__)


class BatchProcessor(OperationalStateMachine, IActionHandler):
    """
    Example component with ActionServer using multi-callback pattern.
    
    This component processes batches of items with:
    - Goal validation (reject if batch too large)
    - Cancellation support (clean shutdown)
    - Progress feedback (10% increments)
    """
    
    def __init__(self):
        super().__init__()
        self.max_batch_size = 100
        self.processing = False
        
        # Bind decorated callbacks (Phase 2 of two-phase init)
        bind_decorated_callbacks(self)
    
    # ============================================================
    # GOAL ACCEPTANCE CALLBACK
    # ============================================================
    
    @remote_actionServer.on_goal(
        name="process_batch",
        auto_register=True  # First decorator should register blueprint
    )
    async def accept_goal(self, goal_request: Any) -> bool:
        """
        Validate and accept/reject incoming goal requests.
        
        Args:
            goal_request: The goal data (typically a ROS2 message or dict)
            
        Returns:
            bool: True to accept, False to reject
            
        Note:
            This is called BEFORE execution starts.
            Use this for validation, resource checks, etc.
        """
        # Extract batch size from goal request
        batch_size = getattr(goal_request, 'batch_size', 0)
        
        logger.info(f"🎯 Goal request received: batch_size={batch_size}")
        
        # Validate batch size
        if batch_size <= 0:
            logger.warning(f"❌ Rejecting goal: invalid batch size {batch_size}")
            return False
        
        if batch_size > self.max_batch_size:
            logger.warning(
                f"❌ Rejecting goal: batch size {batch_size} "
                f"exceeds maximum {self.max_batch_size}"
            )
            return False
        
        if self.processing:
            logger.warning("❌ Rejecting goal: already processing another batch")
            return False
        
        logger.info(f"✅ Goal accepted: batch_size={batch_size}")
        return True
    
    # ============================================================
    # CANCELLATION CALLBACK
    # ============================================================
    
    @remote_actionServer.on_cancel(
        name="process_batch",
        auto_register=False  # Blueprint already registered by on_goal
    )
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        """
        Handle cancellation requests.
        
        Args:
            goal_handle: The goal handle (provides access to goal data)
            
        Returns:
            bool: True to accept cancellation, False to reject
            
        Note:
            Accepting cancellation doesn't immediately stop execution.
            The execute callback must check is_cancel_requested() and
            call goal_handle.canceled() to finalize cancellation.
        """
        logger.warning("⚠️  Cancellation requested for batch processing")
        
        # You could reject cancellation during critical sections:
        # if self.in_critical_section:
        #     logger.warning("❌ Rejecting cancellation: in critical section")
        #     return False
        
        logger.info("✅ Cancellation accepted")
        return True
    
    # ============================================================
    # EXECUTION CALLBACK (Main Logic)
    # ============================================================
    
    @remote_actionServer.execute(
        name="process_batch",
        auto_register=False  # Blueprint already registered by on_goal
    )
    async def execute_batch(self, goal_handle: IGoalHandle) -> Dict[str, Any]:
        """
        Execute the batch processing action.
        
        Args:
            goal_handle: Provides:
                - goal_handle.goal: Goal request data
                - goal_handle.publish_feedback(dict): Send progress updates
                - goal_handle.is_cancel_requested(): Check for cancellation
                - goal_handle.succeed(): Mark as successful
                - goal_handle.abort(): Mark as failed
                - goal_handle.canceled(): Mark as canceled
                
        Returns:
            Dict[str, Any]: Result data (available to action client)
            
        Note:
            You MUST call succeed(), abort(), or canceled() before returning!
        """
        # Extract goal parameters
        batch_size = getattr(goal_handle.goal, 'batch_size', 0)
        logger.info(f"🚀 Starting batch processing: {batch_size} items")
        
        self.processing = True
        processed_count = 0
        
        try:
            # Process items with periodic feedback
            for i in range(batch_size):
                # Check for cancellation request
                if goal_handle.is_cancel_requested():
                    logger.warning(
                        f"⚠️  Cancellation detected at item {i}/{batch_size}"
                    )
                    goal_handle.canceled()
                    return {
                        "status": "canceled",
                        "processed": processed_count,
                        "total": batch_size
                    }
                
                # Simulate processing
                await asyncio.sleep(0.1)
                processed_count += 1
                
                # Publish feedback every 10%
                progress_percent = int((processed_count / batch_size) * 100)
                if progress_percent % 10 == 0:
                    goal_handle.publish_feedback({
                        "progress": progress_percent,
                        "processed": processed_count,
                        "remaining": batch_size - processed_count
                    })
                    logger.debug(
                        f"📊 Progress: {progress_percent}% "
                        f"({processed_count}/{batch_size})"
                    )
            
            # Success!
            logger.info(f"✅ Batch processing complete: {processed_count} items")
            goal_handle.succeed()
            return {
                "status": "success",
                "processed": processed_count,
                "total": batch_size
            }
            
        except Exception as e:
            # Handle errors
            logger.error(f"❌ Batch processing failed: {e}")
            goal_handle.abort()
            return {
                "status": "error",
                "error": str(e),
                "processed": processed_count,
                "total": batch_size
            }
            
        finally:
            self.processing = False


# ============================================================
# ALTERNATIVE: Minimal Implementation
# ============================================================

class MinimalActionServer(OperationalStateMachine, IActionHandler):
    """
    Minimal example with default goal/cancel handlers.
    
    If you only need custom execution logic, you can:
    1. Only decorate @remote_actionServer.execute()
    2. Provide empty on_goal and on_cancel (default: accept always)
    """
    
    def __init__(self):
        super().__init__()
        bind_decorated_callbacks(self)
    
    @remote_actionServer.on_goal(name="simple_action", auto_register=True)
    async def on_goal(self, goal_request: Any) -> bool:
        """Default: Accept all goals."""
        return True
    
    @remote_actionServer.on_cancel(name="simple_action")
    async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
        """Default: Accept all cancellations."""
        return True
    
    @remote_actionServer.execute(name="simple_action")
    async def execute(self, goal_handle: IGoalHandle) -> Dict[str, Any]:
        """Main execution logic."""
        # Do work
        await asyncio.sleep(1.0)
        
        # Check cancellation
        if goal_handle.is_cancel_requested():
            goal_handle.canceled()
            return {"status": "canceled"}
        
        # Success
        goal_handle.succeed()
        return {"status": "done"}


# ============================================================
# TESTING
# ============================================================

async def test_batch_processor():
    """Simple test of BatchProcessor component."""
    logger.info("=" * 60)
    logger.info("Testing BatchProcessor with Multi-Callback Pattern")
    logger.info("=" * 60)
    
    # Create component
    processor = BatchProcessor()
    
    # Verify decorated methods are bound
    from vyra_base.com.core.decorators import get_decorated_methods
    decorated = get_decorated_methods(processor)
    
    logger.info(f"\n📋 Decorated actions: {len(decorated['actions'])}")
    for action in decorated['actions']:
        callback_type = action.get('callback_type', 'unknown')
        logger.info(
            f"  - {action['name']} ({callback_type}) "
            f"[bound: {action.get('blueprint').is_bound(callback_type) if action.get('blueprint') else False}]"
        )
    
    # Verify all callbacks are bound
    from vyra_base.com.core import CallbackRegistry
    CallbackRegistry.debug_print()
    
    logger.info("\n✅ Multi-callback pattern validated successfully!")


if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s'
    )
    
    # Run test
    asyncio.run(test_batch_processor())
