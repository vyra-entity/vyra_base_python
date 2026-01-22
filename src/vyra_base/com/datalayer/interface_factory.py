from __future__ import annotations

from csv import Error
from functools import wraps
from inspect import iscoroutinefunction
from typing import Any, Callable, Union

from rclpy.qos import QoSProfile

from vyra_base.com.datalayer.action_server import Action, VyraActionServer
from vyra_base.com.datalayer.callable import VyraCallable, VyraCallableExecutor
from vyra_base.com.datalayer.job import VyraJob, VyraJobRunner
from vyra_base.com.datalayer.node import NodeSettings, VyraNode, CheckerNode
from vyra_base.com.datalayer.publisher import PeriodicCaller, PublisherInfo, VyraPublisher
from vyra_base.com.datalayer.service_client import ServiceClientInfo, VyraServiceClient
from vyra_base.com.datalayer.service_server import ServiceServerInfo, VyraServiceServer
from vyra_base.com.datalayer.speaker import VyraSpeaker, VyraSpeakerListener
from vyra_base.com.datalayer.subscriber import SubscriptionInfo, VyraSubscriber
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger


class DataSpace:
    """
    Represents a data space in the V.Y.R.A. system.

    This class is used to create and manage various types of data layers such as callables, jobs, observables, and speakers.

    :cvar callables: List of registered V.Y.R.A. callables.
    :cvar jobs: List of registered V.Y.R.A. jobs.
    :cvar speakers: List of registered V.Y.R.A. speakers.
    """

    speakers: list[VyraSpeaker] = []
    callables: list[VyraCallable] = []
    jobs: list[VyraJob] = []
    
    speakers_listener: list[VyraSpeakerListener] = []
    callables_executor: list[VyraCallableExecutor] = []
    jobs_runner: list[VyraJobRunner] = []


    @classmethod
    def kill(cls, obj: Any) -> None:
        """
        Remove a specific object from the DataSpace and clean up any associated resources.

        :param obj: The V.Y.R.A. callable, job, or speaker to remove from the DataSpace.
        :type obj: Any
        :raises ValueError: If the object type is not recognized.
        :return: None
        """
        if isinstance(obj, VyraCallable):
            cls.callables.remove(obj)
        elif isinstance(obj, VyraJob):
            cls.jobs.remove(obj)
        elif isinstance(obj, VyraSpeaker):
            cls.speakers.remove(obj)
        elif isinstance(obj, VyraSpeakerListener):
            cls.speakers_listener.remove(obj)
        elif isinstance(obj, VyraCallableExecutor):
            cls.callables_executor.remove(obj)
        elif isinstance(obj, VyraJobRunner):
            cls.jobs_runner.remove(obj)
        else:
            raise ValueError("Object type not recognized for killing.")
        
        # Additional cleanup logic can be added here if needed
    
    @classmethod
    def get_speaker(cls, name: str) -> VyraSpeaker:
        """
        Get a speaker by its name.

        :param name: Name of the speaker to retrieve.
        :type name: str
        :return: The VyraSpeaker object if found, otherwise None.
        :rtype: VyraSpeaker or None
        """
        for speaker in cls.speakers:
            if speaker.name == name:
                return speaker
        raise ValueError(f"Speaker with name {name} not found in DataSpace.")
    
    @classmethod
    def get_callable(cls, name: str) -> VyraCallable:
        """
        Get a callable by its name.

        :param name: Name of the callable to retrieve.
        :type name: str
        :return: The VyraCallable object if found, otherwise None.
        :rtype: VyraCallable or None
        """
        for callable_ in cls.callables:
            if callable_.name == name:
                return callable_
        raise ValueError(f"Callable with name {name} not found in DataSpace.")

    @classmethod
    def get_job(cls, name: str) -> VyraJob:
        """
        Get a job by its name.

        :param name: Name of the job to retrieve.
        :type name: str
        :return: The VyraJob object if found, otherwise None.
        :rtype: VyraJob or None
        """
        for job in cls.jobs:
            if job.name == name:
                return job
        raise ValueError(f"Job with name {name} not found in DataSpace.")

    @classmethod
    def add_speaker(cls, obj: VyraSpeaker) -> VyraSpeaker:
        """
        Add a speaker object to the DataSpace.

        If a speaker with the same publisher name already exists, 
        merges the new object with the existing one.

        :param obj: The V.Y.R.A. speaker to add or merge.
        :type obj: VyraSpeaker
        :return: The added or merged VyraSpeaker object.
        :rtype: VyraSpeaker
        """

        index: int | None = next(
            (i for i, ele in enumerate(cls.speakers) if 
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing speaker {obj.name}<->"
                f"{cls.speakers[index].name}."
            )
            if obj.publisher_server is None:
                Logger.warn(
                    f"Speaker {obj.name} has no publisher server. "
                    "It will nevertheless be added to the DataSpace."
                )

            return cls.speakers[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new speaker {obj.name}.")
            cls.speakers.append(obj)
            return obj
    
    @classmethod
    def add_callable(cls, obj: VyraCallable) -> VyraCallable:
        """
        Add a callable object to the DataSpace.

        If a callable with the same name already exists, merges the new object with the existing one.

        :param obj: The V.Y.R.A. callable to add or merge.
        :type obj: VyraCallable
        :return: The added or merged VyraCallable object.
        :rtype: VyraCallable
        """
        
        index: int | None = next(
            (i for i, ele in enumerate(cls.callables) if 
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing callable {obj.name}<->"
                f"{cls.callables[index].name}."
            )

            if obj.service_server is None:
                Logger.warn(
                    f"Callable {obj.name} has no service server. "
                    "It will still be added to the DataSpace."
                )

            return cls.callables[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new callable {obj.name}.")
            cls.callables.append(obj)
            return obj
    
    @classmethod
    def add_job(cls, obj: VyraJob) -> VyraJob:
        """
        Add a job object to the DataSpace.

        If a job with the same name already exists, merges the new object with the existing one.

        :param obj: The V.Y.R.A. job to add or merge.
        :type obj: VyraJob
        :return: The added or merged VyraJob object.
        :rtype: VyraJob
        """
        # if obj.service_server is None:
        #     Logger.warn(
        #         f"Job {obj.name} has no service server. "
        #         "It will still be added to the DataSpace."
        #     )
        
        index: int | None = next(
            (i for i, ele in enumerate(cls.jobs) if 
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing job {obj.name}<->"
                f"{cls.jobs[index].name}."
            )
            return cls.jobs[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new job {obj.name}.")
            cls.jobs.append(obj)
            return obj
        
    @classmethod
    def get_speaker_listener(cls, name: str) -> VyraSpeakerListener:
        """
        Get a speaker listener by its name.

        :param name: Name of the speaker to retrieve.
        :type name: str
        :return: The VyraSpeaker object if found, otherwise None.
        :rtype: VyraSpeaker or None
        """
        for speaker in cls.speakers_listener:
            if speaker.name == name:
                return speaker
        raise ValueError(f"Speaker listener with name {name} not found in DataSpace.")

    @classmethod
    def get_callable_executor(cls, name: str) -> VyraCallableExecutor:
        """
        Get a callable executor by its name.

        :param name: Name of the callable to retrieve.
        :type name: str
        :return: The VyraCallableExecutor object if found, otherwise None.
        :rtype: VyraCallableExecutor or None
        """
        for callable_ in cls.callables_executor:
            if callable_.name == name:
                return callable_
        raise ValueError(f"Callable executor with name {name} not found in DataSpace.")

    @classmethod
    def get_job_runner(cls, name: str) -> VyraJobRunner:
        """
        Get a job runner by its name.

        :param name: Name of the job to retrieve.
        :type name: str
        :return: The VyraJobRunner object if found, otherwise None.
        :rtype: VyraJobRunner or None
        """
        for job in cls.jobs_runner:
            if job.name == name:
                return job
        raise ValueError(f"Job runner with name {name} not found in DataSpace.")

    @classmethod
    def add_speaker_listener(cls, obj: VyraSpeakerListener) -> VyraSpeakerListener:
        """
        Add a speaker listener object to the DataSpace.

        If a speaker listener with the same name already exists, 
        merges the new object with the existing one.

        :param obj: The V.Y.R.A. speaker listener to add or merge.
        :type obj: VyraSpeakerListener
        :return: The added or merged VyraSpeakerListener object.
        :rtype: VyraSpeakerListener
        """

        index: int | None = next(
            (i for i, ele in enumerate(cls.speakers_listener) if 
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing speaker listener {obj.name}<->"
                f"{cls.speakers_listener[index].name}."
            )
            if obj.subscriber_server is None:
                Logger.warn(
                    f"Speaker listener {obj.name} has no subscriber server. "
                    "It will nevertheless be added to the DataSpace."
                )

            return cls.speakers_listener[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new speaker listener {obj.name}.")
            cls.speakers_listener.append(obj)
            return obj
    
    @classmethod
    def add_callable_executor(cls, obj: VyraCallableExecutor) -> VyraCallableExecutor:
        """
        Add a callable executor object to the DataSpace.

        If a callable executor with the same name already exists, merges the new object with the existing one.

        :param obj: The V.Y.R.A. callable executor to add or merge.
        :type obj: VyraCallableExecutor
        :return: The added or merged VyraCallableExecutor object.
        :rtype: VyraCallableExecutor
        """

        index: int | None = next(
            (i for i, ele in enumerate(cls.callables_executor) if
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing callable executor {obj.name}<->"
                f"{cls.callables_executor[index].name}."
            )
        
        index: int | None = next(
            (i for i, ele in enumerate(cls.callables_executor) if 
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing callable {obj.name}<->"
                f"{cls.callables_executor[index].name}."
            )

            if obj.service_client is None:
                Logger.warn(
                    f"Callable {obj.name} has no service client. "
                    "It will still be added to the DataSpace."
                )

            return cls.callables_executor[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new callable executor {obj.name}.")
            cls.callables_executor.append(obj)
            return obj
    
    @classmethod
    def add_job_runner(cls, obj: VyraJobRunner) -> VyraJobRunner:
        """
        Add a job runner object to the DataSpace.

        If a job runner with the same name already exists, merges the new object with the existing one.

        :param obj: The V.Y.R.A. job runner to add or merge.
        :type obj: VyraJobRunner
        :return: The added or merged VyraJob object.
        :rtype: VyraJob
        """
        
        index: int | None = next(
            (i for i, ele in enumerate(cls.jobs_runner) if
                ele.name == obj.name), None
        )
        if index is not None:
            Logger.debug(
                f"{cls}Merging existing job runner {obj.name}<->"
                f"{cls.jobs_runner[index].name}."
            )
            return cls.jobs_runner[index].merge(obj)
        else:
            Logger.debug(f"{cls}Adding new job runner {obj.name}.")
            cls.jobs_runner.append(obj)
            return obj


@ErrorTraceback.w_check_error_exist
def create_vyra_speaker(
        type: Any, 
        node: VyraNode,
        description: str,
        periodic: bool = False,
        interval_time: Union[float, None] = None,
        periodic_caller: Union[Callable, None] = None,
        qos_profile: Union[int, QoSProfile] = 10,
        ident_name: str = "global_speaker",
        domain_name: str = "speaker",
        async_loop = None
        ) -> VyraSpeaker:
    """
    Create a speaker for a V.Y.R.A. service.

    A speaker is a publisher that sends messages to a topic and is used to publish simple data types to other V.Y.R.A. OS modules.
    :param type: The ROS2 message datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param description: Description of the speaker.
    :type description: str
    :param periodic: Whether the speaker should publish periodically.
    :type periodic: bool
    :param interval_time: Interval time for periodic publishing.
    :type interval_time: float or None
    :param periodic_caller: Callable for periodic publishing.
    :type periodic_caller: Callable or None
    :param qos_profile: Quality of Service profile.
    :type qos_profile: int or QoSProfile
    :param ident_name: Identifier name for the speaker.
    :type ident_name: str
    :param domain_name: Domain name of the speaker to categorize.
    :type domain_name: str
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :return: The created VyraSpeaker object.
    :rtype: VyraSpeaker
    """

    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a speaker."
        )
    
    name: str = _name_parser(base_name, domain_name, ident_name)

    publisher = PublisherInfo(
        name=name,
        type=type
    )

    if periodic and periodic_caller is not None:
        publisher.periodic_caller = PeriodicCaller(
            interval_time=interval_time,
            caller=periodic_caller,
        )
    
    if qos_profile is not None:
        publisher.qos_profile = qos_profile

    publisher_server = VyraPublisher(
        publisherInfo=publisher,
        node=node
    )

    publisher_server.create_publisher()

    vyra_speaker: VyraSpeaker = VyraSpeaker(
        name=ident_name,
        type=type,
        description=description,
        publisher_server=publisher_server,
    )

    vyra_speaker: VyraSpeaker = DataSpace.add_speaker(vyra_speaker)

    Logger.log(f'VyraSpeaker created: {name}')
    return vyra_speaker

@ErrorTraceback.w_check_error_exist
def create_vyra_callable(
        type: Any, 
        node: VyraNode,
        callback: Union[Callable, None] = None,
        ident_name: str = "global_callable",
        domain_name: str = "callable",
        async_loop = None
    ) -> VyraCallable:
    """
    Create a callable for a V.Y.R.A. (V.Y.R.A. Operating System) service.

    A callable is a function that provides a quick response to the request and 
    does not block the caller. The callable must return a value within a given 
    time limit, otherwise it is considered a failure.

    :param type: The ROS2 service datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param callback: Callback function to be called when the service is invoked.
    :type callback: Callable or None
    :param ident_name: Identifier name for the callable.
    :type ident_name: str
    :param domain_name: Domain name of the speaker to categorize.
    :type domain_name: str
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :raises ValueError: If no callback function is provided.
    :return: The created VyraCallable object.
    :rtype: VyraCallable
    """

    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a callable."
        )

    name: str = _name_parser(base_name, domain_name, ident_name)

    service = ServiceServerInfo(
        name=name,
        type=type
    )

    server = VyraServiceServer(
        serviceInfo=service,
        node=node,
        async_loop=async_loop
    )

    vyra_callable: VyraCallable = VyraCallable(
        name=ident_name,
        type=type,
        description="A callable for the V.Y.R.A. Operating System.",
        service_server=server
    )
    vyra_callable: VyraCallable = DataSpace.add_callable(vyra_callable)

    if callback is not None:
        Logger.info((f"Overwrite callback function {callback.__name__}"
                     f" over callable {vyra_callable.name}.")
        )
        vyra_callable.connected_callback = callback

    if not vyra_callable.connected_callback:
        Logger.error(
            f"Callable <{name}> has no callback function. "
        )
        raise ValueError(
            "A callback function must be provided before "
            f"creation of the callable: {name}."
        )

    server.create_service(vyra_callable.connected_callback)

    Logger.log(f'VyraCallable created: {name}')

    return vyra_callable

@ErrorTraceback.w_check_error_exist
def create_vyra_job(
        type: Any,
        node: VyraNode,
        goal_callback: Callable,
        feedback_callback: Union[Callable, None] = None,
        result_callback: Union[Callable, None] = None,
        ident_name: str = "global_job",
        domain_name: str = "job",
        async_loop = None) -> VyraJob:
    """
    Create a job (action server) for a V.Y.R.A. service.

    A job is a long-running operation that can provide feedback during execution.
    Jobs are based on ROS2 actions and are used for operations that take time to complete,
    such as navigation, manipulation tasks, or other asynchronous operations.

    Jobs provide three types of callbacks:
    - goal_callback: Handles incoming goals (required)
    - feedback_callback: Provides progress updates (optional)
    - result_callback: Returns final result (optional)

    Example::
    
        def execute_task(goal_handle):
            # Long-running task
            for i in range(10):
                feedback_msg = MyAction.Feedback()
                feedback_msg.progress = i * 10
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
            
            result = MyAction.Result()
            result.success = True
            return result
        
        job = create_vyra_job(
            type=MyAction,
            node=entity.node,
            goal_callback=execute_task,
            ident_name="my_task"
        )

    :param type: The ROS2 action datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param goal_callback: Callback function for executing the goal.
    :type goal_callback: Callable
    :param feedback_callback: Optional callback for feedback updates.
    :type feedback_callback: Callable or None
    :param result_callback: Optional callback for result processing.
    :type result_callback: Callable or None
    :param ident_name: Identifier name for the job.
    :type ident_name: str
    :param domain_name: Domain name of the speaker to categorize.
    :type domain_name: str
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :return: The created VyraJob instance.
    :rtype: VyraJob
    """    
    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a job."
        )

    name: str = _name_parser(base_name, domain_name, ident_name)

    action = Action(
        name=name,
        type=type,
        callback=goal_callback
    )

    action_server = VyraActionServer(
        action=action,
        node=node
    )

    vyra_job: VyraJob = VyraJob(
        name=ident_name,
        type=type,
        description="A job (action server) for the V.Y.R.A. Operating System.",
        action_server=action_server,
        goal_callback=goal_callback,
        feedback_callback=feedback_callback,
        result_callback=result_callback
    )
    vyra_job: VyraJob = DataSpace.add_job(vyra_job)

    if not vyra_job.goal_callback:
        Logger.error(
            f"Job <{name}> has no goal callback function. "
        )
        raise ValueError(
            "A goal callback function must be provided before "
            f"creation of the job: {name}."
        )

    action_server.create_action_server()

    Logger.log(f'VyraJob created: {name}')

    return vyra_job


@ErrorTraceback.w_check_error_exist
def create_vyra_job_runner(
        type: Any,
        node: VyraNode,
        feedback_callback: Union[Callable, None] = None,
        result_callback: Union[Callable, None] = None,
        ident_name: str = "global_job_runner",
        timeout_sec: float = 5.0) -> VyraJobRunner:
    """
    Create a job runner (action client) for sending goals to V.Y.R.A. jobs.

    A job runner is used to send goals to action servers and receive feedback/results.
    This is the client-side counterpart to create_vyra_job (server-side).

    Example::
    
        def on_feedback(feedback_msg):
            print(f"Progress: {feedback_msg.feedback.progress}%")
        
        def on_result(result):
            print(f"Task completed: {result.result.success}")
        
        job_runner = create_vyra_job_runner(
            type=MyAction,
            node=entity.node,
            feedback_callback=on_feedback,
            result_callback=on_result,
            ident_name="my_task_client"
        )
        
        # Send goal
        goal = MyAction.Goal()
        goal.target = "destination"
        result = await job_runner.send_goal(goal)

    :param type: The ROS2 action datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param feedback_callback: Optional callback for feedback updates.
    :type feedback_callback: Callable or None
    :param result_callback: Optional callback for result processing.
    :type result_callback: Callable or None
    :param ident_name: Identifier name for the job runner.
    :type ident_name: str
    :param timeout_sec: Timeout for waiting for action server (seconds).
    :type timeout_sec: float
    :return: The created VyraJobRunner instance.
    :rtype: VyraJobRunner
    """
    from vyra_base.com.datalayer.action_client import ActionInfo, VyraActionClient
    
    domain_name = "job"
    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a job runner."
        )

    name: str = _name_parser(base_name, domain_name, ident_name)

    action_info = ActionInfo(
        name=name,
        type=type,
        feedback_callable=feedback_callback if feedback_callback else lambda x: None,
        result_callable=result_callback if result_callback else lambda x: None,
        TIMEOUT_SEC=timeout_sec
    )

    action_client = VyraActionClient(
        actionInfo=action_info,
        node=node
    )

    vyra_job_runner: VyraJobRunner = VyraJobRunner(
        name=ident_name,
        type=type,
        description="A job runner (action client) for the V.Y.R.A. Operating System.",
        action_client=action_client,
        feedback_callback=feedback_callback,
        result_callback=result_callback
    )
    vyra_job_runner: VyraJobRunner = DataSpace.add_job_runner(vyra_job_runner)

    action_client.create_action_client()

    Logger.log(f'VyraJobRunner created: {name}')

    return vyra_job_runner


@ErrorTraceback.w_check_error_exist
def remove_vyra_speaker(name: str= "", speaker: VyraSpeaker= None) -> None:
    """
    Remove a V.Y.R.A. speaker by name.

    :param name: Name of the speaker to remove.
    :type name: str
    :return: None
    """
    if speaker is None and name == "":
        Logger.error(
            "Either name or speaker must be provided."
        )
        raise ValueError("Either name or speaker must be provided.")
    
    if speaker is None:
        speaker = DataSpace.get_speaker(name)

    DataSpace.kill(speaker)
    Logger.info(f"VyraSpeaker removed: {name}")

@ErrorTraceback.w_check_error_exist
def remove_vyra_callable(name: str, callable: VyraCallable = None) -> None:
    """
    Remove a V.Y.R.A. callable by name.

    :param name: Name of the callable to remove.
    :type name: str
    :return: None
    """
    if callable is None and name == "":
        raise ValueError("Either name or callable must be " 
                         f"provided for the callable: {name}.")

    if callable is None:
        callable = DataSpace.get_callable(name)
    
    DataSpace.kill(callable)
    Logger.info(f"VyraCallable removed: {name}")

@ErrorTraceback.w_check_error_exist
def remove_vyra_job(name: str, job: VyraJob = None) -> None:
    """
    Remove a V.Y.R.A. job by name.

    :param name: Name of the job to remove.
    :type name: str
    :return: None
    """
    if job is None and name == "":
        raise ValueError("Either name or job must be provided "
                         f"for the job: {name}.")
    
    if job is None:
        job = DataSpace.get_job(name)

    DataSpace.kill(job)
    Logger.info(f"VyraJob removed: {name}")

@ErrorTraceback.w_check_error_exist
def listen_vyra_speaker(
    type: Any, 
    node: VyraNode,
    description: str,
    callback: Callable[[Any], None],
    ident_name: str = "global_speaker_listener",
    qos_profile: Union[int, QoSProfile] = 10
    ) -> VyraSpeakerListener:
    """
    Create a listener for a V.Y.R.A. speaker.
    A listener is a subscriber that listens to messages from a topic and is used to receive simple 
    data types from other V.Y.R.A. OS modules.
    :param type: The ROS2 message datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param callback: Callback function to be called when a message is received.
    :type callback: Callable[[Any], None]
    :param ident_name: Identifier name for the speaker.
    :type ident_name: str
    :raises ValueError: If no callback function is provided.
    :return: The created VyraSpeaker object.
    :rtype: VyraSpeaker
    """
    domain_name = "speaker"
    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a speaker."
        )

    if (not callable(callback) or callback == None):
        raise ValueError("Invalid callback. A callable callback must be provided.")


    name: str = _name_parser(base_name, domain_name, ident_name)
    subscriber = SubscriptionInfo(
        name=name,
        type=type,
        callback=callback
    )
    
    if qos_profile is not None:
        subscriber.qos_profile = qos_profile

    subscriber_server = VyraSubscriber(
        subscriptionInfo=subscriber,
        node=node
    )

    subscriber_server.create_subscription()

    listener: VyraSpeakerListener = VyraSpeakerListener(
        name=ident_name,
        type=type,
        description=description,
        subscriber_server=subscriber_server,
    )

    listener: VyraSpeakerListener = DataSpace.add_speaker_listener(
        listener)

    Logger.log(f'VyraSpeaker listener created: {name}')
    return listener

def remove_vyra_speaker_listener(
    name: str= "", 
    listener: VyraSpeakerListener= None
    ) -> None:
    """ Remove a V.Y.R.A. speaker listener by name.
    :param name: Name of the speaker listener to remove.
    :type name: str
    :return: None
    """
    if listener is None and name == "":
        Logger.error(
            "Either name or listener must be provided."
        )
        raise ValueError("Either name or listener must be provided.")
    
    if listener is None:
        listener = DataSpace.get_speaker_listener(name)
    
    DataSpace.kill(listener)
    Logger.info(f"VyraSpeakerListener removed: {name}")

@ErrorTraceback.w_check_error_exist
async def execute_vyra_callable(
    type: Any, 
    node: VyraNode,
    send_data: dict,
    ident_name: str = "global_callable_executor",
    timeout: float = 30.0,
    async_loop = None
    ) -> Any:
    """
    Create a callable executor for an external V.Y.R.A. (V.Y.R.A. Operating System) 
    service.

    A callable is a function that provides a quick response to the request and 
    does not block the caller. The callable must return a value within a given 
    time limit, otherwise it is considered a failure. This method creates a client
    that can call an external callable service.

    :param type: The ROS2 service datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param send_data: Data to be sent to the callable service.
    :type send_data: dict
    :param ident_name: Identifier name for the callable.
    :type ident_name: str
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :raises ValueError: If no callback function is provided.
    :return: The created VyraCallable object.
    :rtype: VyraCallable
    """
    domain_name = "callable"
    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a callable."
        )

    name: str = _name_parser(base_name, domain_name, ident_name)

    service = ServiceClientInfo(
        name=name,
        type=type,
        timeout=timeout
    )

    client = VyraServiceClient(
        serviceInfo=service,
        node=node
    )

    executor: VyraCallableExecutor = VyraCallableExecutor(
        name=ident_name,
        type=type,
        description="A callable executor for the V.Y.R.A. Operating System.",
        service_client=client
    )

    executor: VyraCallableExecutor = DataSpace.add_callable_executor(executor)

    try:
        if executor.service_client is None:
            Logger.error(
                f"Callable executor <{name}> has no service client. "
            )
            raise ValueError(
                "A service client must be provided before "
                f"creation of the callable executor: {name}."
            )
    
        await executor.service_client.create_service_caller()

        Logger.log(f'VyraCallable executor created: {name}')

        return (await executor.service_client.send(**send_data))
    finally:
        DataSpace.kill(executor)

def run_vyra_job(
    type: Any,
    node: VyraNode,
    send_data: dict,
    ident_name: str = "global_job_runner",
    async_loop = None) -> VyraJobRunner:
    """
    Create a job runner for an external V.Y.R.A. (V.Y.R.A. Operating System) 
    service.

    A job is a function that will be executed in the background and will not block the caller.
    A job can take a long time to complete and will return a result when it is done.
    Additionally, a job can provide feedback information during the process.
    This method creates a client that can call an external job service.

    :param type: The ROS2 service datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param send_data: Data to be sent to the job service.
    :type send_data: dict
    :param ident_name: Identifier name for the job.
    :type ident_name: str
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :return: The created VyraJobRunner object.
    :rtype: VyraJobRunner
    """
    domain_name = "job"
    base_name: str = node.node_settings.name

    if base_name == NodeSettings.name:
        Logger.warn(
            "Node name has not been set. This could cause issues within the " \
            "V.Y.R.A. system. Please set the node name in the NodeSettings before " \
            "creating a callable."
        )

    name: str = _name_parser(base_name, domain_name, ident_name)
    # TBD: Implement job runner creation and execution logic
    raise NotImplementedError("Job runner creation not implemented yet.")

@ErrorTraceback.w_check_error_exist
def remote_callable(func):
    """
    Decorator to register a function or method as a V.Y.R.A. callable.

    :param func: The function to be registered as a remote callable.
    :type func: Callable
    :return: The wrapped function with remote callable marker.
    :rtype: Callable
    """
    @wraps(func)
    async def async_wrapper(*args, **kwargs):
        """
        Async wrapper for remote callable functions with debug logging.
        
        :param args: Positional arguments for wrapped function.
        :param kwargs: Keyword arguments for wrapped function.
        :return: Result from async function call.
        """
        Logger.debug(f"Calling function async: {func.__name__} "
                     f"with args: {args} and kwargs: {kwargs}")
        result = await func(*args, **kwargs)
        return result

    @wraps(func)
    def sync_wrapper(*args, **kwargs):
        """
        Sync wrapper for remote callable functions with debug logging.
        
        :param args: Positional arguments for wrapped function.
        :param kwargs: Keyword arguments for wrapped function.
        :return: Result from function call.
        """
        Logger.debug(f"Calling function: {func.__name__} "
                     f"with args: {args} and kwargs: {kwargs}")
        result = func(*args, **kwargs)
        return result

    if iscoroutinefunction(func):
        Logger.info(f"(API) Registering {func.__name__} as coroutine.")
        wrapper = async_wrapper
    else:
        Logger.warn(f"(API) Registering {func.__name__} as regular function.")
        wrapper = sync_wrapper

    # Registration is performed later in the instance
    setattr(wrapper, "_remote_callable", True)  # Mark that this method should be registered

    return wrapper

def _name_parser(*names) -> str:
    if not CheckerNode.check_node_name('/'.join(names)):
        Logger.error(
            f"Invalid callable prefix name: {'/'.join(names)}. Could not be created. "
            "Names must start with a letter and can contain letters, "
            "numbers, underscores, and hyphens[a-zA-Z0-9_-]."
        )
        raise NameError(
            f"Invalid callable prefix name: {'/'.join(names)}. "
            "Names must start with a letter and can contain letters, "
            "numbers, underscores, and hyphens[a-zA-Z0-9_-]."
        )
    return '/'.join(names)
