from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Union

from vyra_base.com.datalayer.action_client import VyraActionClient, ActionInfo
from vyra_base.com.datalayer.action_server import VyraActionServer, Action
from vyra_base.helper.logger import Logger


@dataclass
class VyraJob:
    """
    Represents and stores all information about a Vyra job.

    A job is a long-running operation that can provide feedback during execution.
    It is based on ROS2 actions and is used for operations that take time to complete,
    such as navigation, manipulation tasks, or other asynchronous operations.

    Jobs are analogous to Callables but for long-running tasks:
    - VyraCallable -> ROS2 Service (quick request/response)
    - VyraJob -> ROS2 Action (long-running with feedback)

    :param name: Name of the job.
    :type name: str
    :param type: Type of the job, typically a ROS2 action type.
    :type type: Any
    :param description: Description of the job.
    :type description: str
    :param last_return: The last return value of the job.
    :type last_return: Any
    :param action_server: The associated VyraActionServer instance (for server-side).
    :type action_server: VyraActionServer or None
    :param goal_callback: Callback function for goal requests.
    :type goal_callback: Callable or None
    :param feedback_callback: Callback function for feedback updates.
    :type feedback_callback: Callable or None
    :param result_callback: Callback function for final results.
    :type result_callback: Callable or None
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None
    action_server: Union[VyraActionServer, None] = None
    goal_callback: Union[Callable, None] = None
    feedback_callback: Union[Callable, None] = None
    result_callback: Union[Callable, None] = None

    def __repr__(self) -> str:
        """
        Returns the string representation of the VyraJob.

        :return: The name of the job.
        :rtype: str
        """
        return self.name
    
    def merge(self, other: Any) -> VyraJob:
        """
        Merge another Job into this one, combining their attributes.

        :param other: Another Job instance to merge with.
        :type other: Any
        :return: A new Job instance with merged attributes.
        :rtype: VyraJob
        """
        Logger.debug(f"[MERGE DEBUG] Starting merge for job: {self.name}\n")
        Logger.debug(f"[MERGE DEBUG] self.action_server BEFORE: {self.action_server}")
        Logger.debug(f"[MERGE DEBUG] other.action_server BEFORE: {other.action_server}")
        
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return
        
        # Transfer action_server if self doesn't have one
        if other.action_server:
            if not self.action_server:
                Logger.debug(f"[MERGE DEBUG] Transferring action_server from other to self")
                self.action_server = other.action_server
            else:
                Logger.debug(f"[MERGE DEBUG] self already has action_server, NOT transferring")
            
            # Clear reference from 'other' to prevent __del__ from destroying it
            Logger.debug(f"[MERGE DEBUG] Clearing other.action_server")
            other.action_server = None
        
        Logger.debug(f"[MERGE DEBUG] self.action_server AFTER: {self.action_server}")
        Logger.debug(f"[MERGE DEBUG] other.action_server AFTER: {other.action_server}")
        
        self.goal_callback = other.goal_callback or self.goal_callback
        self.feedback_callback = other.feedback_callback or self.feedback_callback
        self.result_callback = other.result_callback or self.result_callback

        return self
    
    def __del__(self):
        """
        Destructor to clean up the job.
        If the job has an action server, it will be destroyed.
        """
        Logger.debug(f"[DEL DEBUG] __del__ called for job: {self.name}")
        Logger.debug(f"[DEL DEBUG] action_server value: {self.action_server}")
        
        if self.action_server:
            Logger.debug(f"[DEL DEBUG] DESTROYING action server for: {self.name}")
            # Action servers are automatically cleaned up by ROS2
            self.action_server = None
        else:
            Logger.debug(f"[DEL DEBUG] NO action_server to destroy for: {self.name}")


@dataclass
class VyraJobRunner:
    """
    Represents a job runner (action client) in the data layer.

    A job runner is used to send goals to action servers and receive feedback/results.
    This is the client-side counterpart to VyraJob (server-side).

    JobRunner is analogous to CallableExecutor:
    - VyraCallableExecutor -> ROS2 Service Client (calls services)
    - VyraJobRunner -> ROS2 Action Client (sends goals to actions)

    :param name: The name of the job runner.
    :type name: str
    :param type: The type of the action.
    :type type: Any
    :param description: The description of the job runner.
    :type description: str
    :param last_return: The last return value of the job runner.
    :type last_return: Any
    :param action_client: The associated VyraActionClient instance.
    :type action_client: VyraActionClient or None
    :param feedback_callback: Callback function for feedback updates.
    :type feedback_callback: Callable or None
    :param result_callback: Callback function for final results.
    :type result_callback: Callable or None
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None
    action_client: Union[VyraActionClient, None] = None
    feedback_callback: Union[Callable, None] = None
    result_callback: Union[Callable, None] = None

    def __repr__(self) -> str:
        """
        Returns the string representation of the VyraJobRunner.

        :return: The name of the job runner.
        :rtype: str
        """
        return self.name

    def merge(self, other: Any) -> VyraJobRunner:
        """
        Merges another JobRunner into this one, combining their attributes.

        :param other: Another JobRunner instance to merge with.
        :type other: Any
        :return: A new JobRunner instance with merged attributes.
        :rtype: VyraJobRunner
        """
        Logger.debug(f"[MERGE DEBUG] Starting merge for job runner: {self.name}\n")
        Logger.debug(f"[MERGE DEBUG] self.action_client BEFORE: {self.action_client}")
        Logger.debug(f"[MERGE DEBUG] other.action_client BEFORE: {other.action_client}")
        
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return
        
        # Transfer action_client if self doesn't have one
        if other.action_client:
            if not self.action_client:
                Logger.debug(f"[MERGE DEBUG] Transferring action_client from other to self")
                self.action_client = other.action_client
            else:
                Logger.debug(f"[MERGE DEBUG] self already has action_client, NOT transferring")
            
            # Clear reference from 'other' to prevent __del__ from destroying it
            Logger.debug(f"[MERGE DEBUG] Clearing other.action_client")
            other.action_client = None
        
        Logger.debug(f"[MERGE DEBUG] self.action_client AFTER: {self.action_client}")
        Logger.debug(f"[MERGE DEBUG] other.action_client AFTER: {other.action_client}")
        
        self.feedback_callback = other.feedback_callback or self.feedback_callback
        self.result_callback = other.result_callback or self.result_callback

        return self
    
    def __del__(self):
        """
        Destructor to clean up the job runner.
        If the job runner has an action client, it will be destroyed.
        """
        Logger.debug(f"[DEL DEBUG] __del__ called for job runner: {self.name}")
        Logger.debug(f"[DEL DEBUG] action_client value: {self.action_client}")
        
        if self.action_client:
            Logger.debug(f"[DEL DEBUG] DESTROYING action client for: {self.name}")
            # Action clients are automatically cleaned up by ROS2
            self.action_client = None
        else:
            Logger.debug(f"[DEL DEBUG] NO action_client to destroy for: {self.name}")

    async def send_goal(self, goal_msg: Any) -> Any:
        """
        Send a goal to the action server.

        :param goal_msg: The goal message to send.
        :type goal_msg: Any
        :return: The result of the goal.
        :rtype: Any
        :raises ValueError: If the action client has not been created.
        """
        if not self.action_client:
            raise ValueError(f"Action client for job runner '{self.name}' has not been created.")
        
        # Send goal and wait for acceptance
        send_goal_future = self.action_client.send_goal(goal_msg)
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            Logger.warning(f"Goal rejected by action server '{self.name}'")
            return None
        
        Logger.info(f"Goal accepted by action server '{self.name}'")
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        result = await get_result_future
        
        self.last_return = result.result
        
        if self.result_callback:
            self.result_callback(result)
        
        return result.result
