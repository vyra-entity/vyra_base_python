from __future__ import annotations

import stat
from ast import Call
from functools import wraps
from inspect import iscoroutinefunction
from typing import Any, Callable, Union

from rclpy.qos import QoSProfile

from vyra_base.com.datalayer.action_client import VyraActionClient
from vyra_base.com.datalayer.action_server import VyraActionServer
from vyra_base.com.datalayer.callable import VyraCallable
from vyra_base.com.datalayer.job import VyraJob
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.publisher import PeriodicCaller, PublisherInfo, VyraPublisher
from vyra_base.com.datalayer.service_client import VyraServiceClient
from vyra_base.com.datalayer.service_server import ServiceInfo, VyraServiceServer
from vyra_base.com.datalayer.speaker import VyraSpeaker
from vyra_base.com.datalayer.subscriber import VyraSubscription
from vyra_base.helper.logger import Logger


class DataSpace:
    """
    Represents a data space in the V.Y.R.A. system.

    This class is used to create and manage various types of data layers such as callables, jobs, observables, and speakers.

    :cvar callables: List of registered V.Y.R.A. callables.
    :cvar jobs: List of registered V.Y.R.A. jobs.
    :cvar speakers: List of registered V.Y.R.A. speakers.
    """

    callables: list[VyraCallable] = []
    jobs: list[VyraJob] = []
    speakers: list[VyraSpeaker] = []

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

        If a speaker with the same publisher name already exists, merges the new object with the existing one.

        :param obj: The V.Y.R.A. speaker to add or merge.
        :type obj: VyraSpeaker
        :return: The added or merged VyraSpeaker object.
        :rtype: VyraSpeaker
        """
        try:
            if obj.publisher_server != None:
                obj_name: str = obj.name

            index = next(
                (i for i, ele in enumerate(cls.speakers) if 
                 ele.publisher_server != None and 
                 obj.publisher_server != None and
                 ele.publisher_server.publisher_info.name == obj_name),
                -1
            )

            return cls.speakers[index].merge(obj)
        except IndexError:
            index = -1
            cls.speakers.append(obj)
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
        try:
            index = next(
                (i for i, ele in enumerate(cls.jobs) if ele.name == obj.name),
                -1
            )
            return cls.jobs[index].merge(obj)
        except ValueError:
            index = -1
            cls.jobs.append(obj)
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
        index = next(
            (i for i, ele in enumerate(cls.callables) if ele.name == obj.name),
            -1
        )
        
        if index == -1:
            cls.callables.append(obj)
            return obj
        return cls.callables[index].merge(obj)
    
def create_vyra_callable(
        name: str, 
        type: Any, 
        node: VyraNode,
        callback: Union[Callable, None] = None,
        async_loop = None
    ) -> VyraCallable:
    """
    Create a callable for a V.Y.R.A. (V.Y.R.A. Operating System) service.

    A callable is a function that provides a quick response to the request and does not block the caller.
    The callable must return a value within a given time limit, otherwise it is considered a failure.

    :param name: Name of the callable.
    :type name: str
    :param type: The ROS2 service datatype definition.
    :type type: Any
    :param node: The ROS2 node definition.
    :type node: VyraNode
    :param callback: Callback function to be called when the service is invoked.
    :type callback: Callable or None
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :raises ValueError: If no callback function is provided.
    :return: The created VyraCallable object.
    :rtype: VyraCallable
    """
    service = ServiceInfo(
        name=name,
        type=type
    )

    server = VyraServiceServer(
        serviceInfo=service,
        node=node,
        async_loop=async_loop
    )

    vyra_callable: VyraCallable = VyraCallable(
        name=name,
        type=type,
        description="A callable for the V.Y.R.A. Operating System.",
        service_server=server
    )
    vyra_callable: VyraCallable = DataSpace.add_callable(vyra_callable)

    if callback is not None:
        Logger.info((f"Overwrite callback function {callback.__name__}"
                     f" to callable {vyra_callable.name}.")
        )
        vyra_callable.connected_callback = callback

    if not vyra_callable.connected_callback:
        raise ValueError(
            "A callback function must be provided for the callable."
        )

    server.create_service(vyra_callable.connected_callback)

    return vyra_callable

def create_vyra_job(
        name: str, 
        type: Any,
        async_loop = None) -> None:
    """
    Create a job for a V.Y.R.A. (V.Y.R.A. Operating System) service.

    A job is a function that will be executed in the background and will not block the caller.
    A job can take a long time to complete and will return a result when it is done.
    Additionally, a job can provide feedback information during the process.

    :param name: Name of the job.
    :type name: str
    :param type: The ROS2 service datatype definition.
    :type type: Any
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :return: None
    """
    pass

def create_vyra_speaker(
        name: str, 
        type: Any, 
        node: VyraNode,
        description: str,
        periodic: bool = False,
        interval_time: Union[float, None] = None,
        periodic_caller: Union[Callable, None] = None,
        qos_profile: Union[int, QoSProfile] = 10,
        async_loop = None
        ) -> VyraSpeaker:
    """
    Create a speaker for a V.Y.R.A. service.

    A speaker is a publisher that sends messages to a topic and is used to publish simple data types to other V.Y.R.A. OS modules.

    :param name: Name of the speaker.
    :type name: str
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
    :param async_loop: Optional event loop for asynchronous execution.
    :type async_loop: Any
    :return: The created VyraSpeaker object.
    :rtype: VyraSpeaker
    """
    
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
        name=name,
        type=type,
        description=description,
        publisher_server=publisher_server,
    )

    vyra_speaker: VyraSpeaker = DataSpace.add_speaker(vyra_speaker)
    
    return vyra_speaker

def remove_vyra_speaker(name: str= "", speaker: VyraSpeaker= None) -> None:
    """
    Remove a V.Y.R.A. speaker by name.

    :param name: Name of the speaker to remove.
    :type name: str
    :return: None
    """
    if speaker is None and name == "":
        raise ValueError("Either name or speaker must be provided.")
    
    if speaker is None:
        speaker = DataSpace.get_speaker(name)

    DataSpace.kill(speaker)
    Logger.info(f"Speaker '{name}' removed from DataSpace.")

def remove_vyra_callable(name: str, callable: VyraCallable = None) -> None:
    """
    Remove a V.Y.R.A. callable by name.

    :param name: Name of the callable to remove.
    :type name: str
    :return: None
    """
    if callable is None and name == "":
        raise ValueError("Either name or callable must be provided.")
    
    if callable is None:
        callable = DataSpace.get_callable(name)
    
    DataSpace.kill(callable)
    Logger.info(f"Callable '{name}' removed from DataSpace.")

def remove_vyra_job(name: str, job: VyraJob = None) -> None:
    """
    Remove a V.Y.R.A. job by name.

    :param name: Name of the job to remove.
    :type name: str
    :return: None
    """
    if job is None and name == "":
        raise ValueError("Either name or job must be provided.")
    
    if job is None:
        job = DataSpace.get_job(name)

    DataSpace.kill(job)
    Logger.info(f"Job '{name}' removed from DataSpace.")

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
        print(f"Calling function async: {func.__name__} with args: {args} and kwargs: {kwargs}")
        result = await func(*args, **kwargs)
        return result

    @wraps(func)
    def sync_wrapper(*args, **kwargs):
        print(f"Calling function: {func.__name__} with args: {args} and kwargs: {kwargs}")
        result = func(*args, **kwargs)
        return result

    wrapper = async_wrapper if iscoroutinefunction(func) else sync_wrapper

    # Registration is performed later in the instance
    setattr(wrapper, "_remote_callable", True)  # Mark that this method should be registered

    return wrapper