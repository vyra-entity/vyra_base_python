"""
ROS2 Action Server Implementation

Async-first action server for ROS2 with goal/cancel/execution callbacks.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraActionServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication.action_server import (
    ROS2ActionServer as ROS2ActionServer,
    ActionServerInfo
)

logger = logging.getLogger(__name__)


class _ROS2GoalHandleWrapper:
    """
    Wraps a ROS2 ServerGoalHandle to match the VYRA async goal_handle interface.

    Translates attribute names and converts sync ROS2 methods to the async
    signatures expected by Vyra application callbacks:

    - ``.goal`` maps to ``goal_handle.request``
    - ``await publish_feedback(dict)`` converts the dict to a ROS2 Feedback message
      and calls the synchronous ``goal_handle.publish_feedback()``
    - ``succeed()``, ``abort()``, ``canceled()`` record the desired final state so
      the execute_callback wrapper can apply them from the ROS2 executor thread,
      which is required for rclpy state-changing operations to work correctly.
    """

    def __init__(self, ros2_handle: Any, action_type: Any) -> None:
        self._handle = ros2_handle
        self._action_type = action_type
        self._finalized = False
        self._final_state: Optional[str] = None  # 'succeeded' | 'aborted' | 'canceled'

    @property
    def goal(self) -> Any:
        """Return the goal request object (ROS2 attribute is `.request`)."""
        return self._handle.request

    def is_cancel_requested(self) -> bool:
        """Return whether a cancel has been requested."""
        return self._handle.is_cancel_requested

    def succeed(self) -> None:
        """Record 'succeeded' state — applied from executor thread after coroutine returns."""
        self._finalized = True
        self._final_state = 'succeeded'

    def abort(self) -> None:
        """Record 'aborted' state — applied from executor thread after coroutine returns."""
        self._finalized = True
        self._final_state = 'aborted'

    def canceled(self) -> None:
        """Record 'canceled' state — applied from executor thread after coroutine returns."""
        self._finalized = True
        self._final_state = 'canceled'

    async def publish_feedback(self, feedback: Any) -> None:
        """Convert feedback dict to ROS2 Feedback message and publish (sync call)."""
        import time
        logger.debug("⚙️  publish_feedback: building msg")
        feedback_msg = self._action_type.Feedback()
        if isinstance(feedback, dict):
            for key, value in feedback.items():
                if hasattr(feedback_msg, key):
                    setattr(feedback_msg, key, value)
        t0 = time.monotonic()
        logger.debug("⚙️  publish_feedback: calling handle.publish_feedback")
        self._handle.publish_feedback(feedback_msg)
        elapsed = time.monotonic() - t0
        if elapsed > 0.05:
            logger.warning(f"⚠️  publish_feedback SLOW: {elapsed:.3f}s")
        else:
            logger.debug(f"⚙️  publish_feedback: done in {elapsed:.4f}s")


class VyraActionServerImpl(VyraActionServer):
    """
    Vyra Action Server implementation.

    Wraps a ROS2 action server with async callback support for goal handling.
    All three callbacks (goal, cancel, execute) are bridged from the ROS2
    executor thread back onto the main asyncio event loop using
    ``asyncio.run_coroutine_threadsafe`` to avoid deadlocking on a running loop.
    """

    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        action_type: Any,
        handle_goal_request: Optional[Callable] = None,
        handle_cancel_request: Optional[Callable] = None,
        execution_callback: Optional[Callable] = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            handle_goal_request, handle_cancel_request, execution_callback,
            ProtocolType.ROS2, **kwargs
        )
        self.node = node
        self.action_type = action_type
        self._ros2_action_server: Optional[ROS2ActionServer] = None

    async def initialize(self) -> bool:
        """Initialize ROS2 action server."""
        try:
            action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)

            # Capture the running event loop NOW (initialize() runs in the main
            # asyncio context).  The sync wrappers below are called from the ROS2
            # executor thread, so we use run_coroutine_threadsafe instead of
            # run_until_complete (which would deadlock on an already-running loop).
            main_loop = asyncio.get_event_loop()

            def goal_callback(goal_request):
                """Sync wrapper for handle_goal_request (called from ROS2 executor thread)."""
                from rclpy.action.server import GoalResponse
                try:
                    if self.handle_goal_request:
                        if asyncio.iscoroutinefunction(self.handle_goal_request):
                            future = asyncio.run_coroutine_threadsafe(
                                self.handle_goal_request(goal_request), main_loop
                            )
                            accepted = future.result(timeout=10.0)
                        else:
                            accepted = self.handle_goal_request(goal_request)
                        if isinstance(accepted, bool):
                            return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT
                        return accepted
                    return GoalResponse.ACCEPT
                except Exception as e:
                    logger.error(f"❌ Goal callback error: {e}")
                    return GoalResponse.REJECT

            def cancel_callback(goal_handle):
                """Sync wrapper for handle_cancel_request (called from ROS2 executor thread)."""
                from rclpy.action.server import CancelResponse
                try:
                    if self.handle_cancel_request:
                        if asyncio.iscoroutinefunction(self.handle_cancel_request):
                            future = asyncio.run_coroutine_threadsafe(
                                self.handle_cancel_request(goal_handle), main_loop
                            )
                            accepted = future.result(timeout=10.0)
                        else:
                            accepted = self.handle_cancel_request(goal_handle)
                        if isinstance(accepted, bool):
                            return CancelResponse.ACCEPT if accepted else CancelResponse.REJECT
                        return accepted
                    return CancelResponse.ACCEPT
                except Exception as e:
                    logger.error(f"❌ Cancel callback error: {e}")
                    return CancelResponse.ACCEPT

            def execute_callback(goal_handle):
                """Sync wrapper for execution_callback (called from ROS2 executor thread)."""
                import threading
                logger.debug(
                    f"⚙️  execute_callback: START thread={threading.current_thread().name}"
                )
                # Wrap ROS2 ServerGoalHandle so the application sees the VYRA interface
                wrapper = _ROS2GoalHandleWrapper(goal_handle, self.action_type)
                try:
                    if self.execution_callback:
                        if asyncio.iscoroutinefunction(self.execution_callback):
                            logger.debug("⚙️  execute_callback: submitting coro to asyncio loop")
                            future = asyncio.run_coroutine_threadsafe(
                                self.execution_callback(wrapper), main_loop
                            )
                            logger.debug("⚙️  execute_callback: waiting for coro result (timeout=30s)...")
                            result = future.result(timeout=30.0)  # blocking wait with timeout
                            logger.debug(f"⚙️  execute_callback: coro done, result={result}")
                        else:
                            result = self.execution_callback(wrapper)

                        # Map result dict to ROS2 Result message
                        result_msg = self.action_type.Result()
                        if isinstance(result, dict):
                            for key, value in result.items():
                                if hasattr(result_msg, key):
                                    setattr(result_msg, key, value)

                        # Apply the final state from the executor thread — rclpy requires
                        # succeed/abort/canceled to be called from the executor thread, not
                        # from the asyncio thread (where the application callback ran).
                        if wrapper._final_state == 'succeeded':
                            goal_handle.succeed()
                        elif wrapper._final_state == 'aborted':
                            goal_handle.abort()
                        elif wrapper._final_state == 'canceled':
                            goal_handle.canceled()
                        else:
                            goal_handle.succeed()  # default if application didn't call finalize

                        return result_msg

                    result_msg = self.action_type.Result()
                    goal_handle.succeed()
                    return result_msg
                except Exception as e:
                    logger.error(f"❌ Execution callback error: {e}")
                    try:
                        if not wrapper._finalized:
                            goal_handle.abort()
                    except Exception:
                        pass
                    return self.action_type.Result()

            # Create action server via ROS2ActionServer
            action_info = ActionServerInfo(
                name=action_name,
                type=self.action_type,
                goal_callback=goal_callback,
                cancel_callback=cancel_callback,
                execute_callback=execute_callback
            )

            self._ros2_action_server = ROS2ActionServer(
                actionInfo=action_info,
                node=self.node
            )
            self._ros2_action_server.create_action_server()

            self._transport_handle = self._ros2_action_server
            self._initialized = True

            logger.info(f"✅ ROS2 ActionServer '{self.name}' initialized on '{action_name}'")
            return True

        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 ActionServer '{self.name}': {e}")
            raise InterfaceError(f"ActionServer initialization failed: {e}")

    async def shutdown(self) -> None:
        """Shutdown ROS2 action server."""
        if self._ros2_action_server:
            self._ros2_action_server.destroy()
            self._ros2_action_server = None
        self._initialized = False
        self._transport_handle = None

    async def start(self) -> None:
        """
        Start action server (already active after initialize for ROS2).
        """
        if not self._initialized:
            raise InterfaceError(f"ActionServer '{self.name}' not initialized")

        logger.debug(f"ActionServer '{self.name}' is active")
