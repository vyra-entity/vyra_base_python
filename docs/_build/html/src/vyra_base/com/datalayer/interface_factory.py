from __future__ import annotations

from ast import Call
import stat
from typing import Callable
from typing import Union
from inspect import iscoroutinefunction

from rclpy.qos import QoSProfile

from typing import Any

from functools import wraps

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.helper.logger import Logger

from vyra_base.com.datalayer.action_client import VyraActionClient
from vyra_base.com.datalayer.action_server import VyraActionServer

from vyra_base.com.datalayer.subscriber import VyraSubscription

from vyra_base.com.datalayer.publisher import VyraPublisher
from vyra_base.com.datalayer.publisher import PublisherInfo
from vyra_base.com.datalayer.publisher import PeriodicCaller

from vyra_base.com.datalayer.callable import VyraCallable
from vyra_base.com.datalayer.job import VyraJob
from vyra_base.com.datalayer.speaker import VyraSpeaker

from vyra_base.com.datalayer.service_client import VyraServiceClient
from vyra_base.com.datalayer.service_server import VyraServiceServer
from vyra_base.com.datalayer.service_server import ServiceInfo


class DataSpace:
    """
    DataSpace is a class that represents a data space in the V.Y.R.A..
    It is used to create and manage various types of data layers such as callables, jobs, observables, and speakers.
    """

    callables: list[VyraCallable] = []
    jobs: list[VyraJob] = []
    speakers: list[VyraSpeaker] = []

    @classmethod
    def kill(cls, obj: Any) -> None:
        """
        Kill a specific object in the DataSpace.
        This method will remove the object from the DataSpace and clean up any resources associated with it.
        
        :param obj: Remove a V.Y.R.A. callable, job, or speaker from the DataSpace.
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
    def add_speaker(cls, obj: VyraSpeaker) -> VyraSpeaker:
        """
        Add an object to the DataSpace.
        This method will add the object to the appropriate list in the DataSpace.
        
        :param obj: Add a V.Y.R.A. callable, job, or speaker to the DataSpace. 
                    If the object already exists, it will merge the new object with 
                    the existing one.

        :return: None
        """

        try:
            if obj.publisher_server != None:
                obj_name: str = obj.publisher_server.publisher_info.name

            index = next(
                (i for i, ele in enumerate(cls.speakers) if 
                 ele.publisher_server != None and 
                 obj.publisher_server != None and
                 ele.publisher_server.publisher_info.name == obj_name),
                -1  # Rückgabewert, falls nicht gefunden
            )

            return cls.speakers[index].merge(obj)
        except IndexError:
            index = -1  # Oder eine andere Fehlerbehandlung
            cls.speakers.append(obj)
            return obj
    
    @classmethod
    def add_job(cls, obj: VyraJob) -> VyraJob:
        """
        Add an object to the DataSpace.
        This method will add the object to the appropriate list in the DataSpace.
        
        :param obj: Add a V.Y.R.A. callable, job, or speaker to the DataSpace. 
                    If the object already exists, it will merge the new object with 
                    the existing one.

        :return: None
        """

        try:
            index = next(
                (i for i, ele in enumerate(cls.jobs) if ele.name == obj.name),
                -1  # Rückgabewert, falls nicht gefunden
            )
            return cls.jobs[index].merge(obj)
        except ValueError:
            index = -1  # Oder eine andere Fehlerbehandlung
            cls.jobs.append(obj)
            return obj
    
    @classmethod
    def add_callable(cls, obj: VyraCallable) -> VyraCallable:
        """
        Add an object to the DataSpace.
        This method will add the object to the appropriate list in the DataSpace.
        
        :param obj: Add a V.Y.R.A. callable, job, or speaker to the DataSpace. 
                    If the object already exists, it will merge the new object with 
                    the existing one.

        :return: None
        """
        index = next(
            (i for i, ele in enumerate(cls.callables) if ele.name == obj.name),
            -1  # Rückgabewert, falls nicht gefunden
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
    A callable is a function that will give a very quick response to
    the request, and will not block the caller. A callable must return a
    value within a given time limit, otherwise it will be considered a 
    failure.
    
    :param name: Name of the callable.
    :param type: Containing the ros2 service datatype definition.
    :param node: Definition of the ros2 node.
    :param callback: Callback function that will be called when the 
                     service is invoked.
    :param parameters: Optional parameters of the callable that are needed.
    :param callback: Optional callback function that will be called when the service is invoked.
                     Better use the decorator `@remote_callable` to add a callable connected function.
    
    :return: Callable that can be used to call the service.
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
    Create a job for a V.Y.R.A. (V.Y.R.A. Operating System) service. A job
    is a function that will be executed in the background, and will not
    block the caller. A job can take a long time to complete, and will
    return a result when it is done. Beside the result, a job can also
    provide feedback information during the process, which can be used 
    until the job is completed.

    has completed, or has failed.
    :param name: Name of the service.,
    An observer is a subscription to a topic that will receive updates of
    simple data types. It is been used to watch variables of other variobotic
    os modules. 

    For example this could be used to watch the state of a robot, live coordinates
    of a camera, or the status of a sensor.

    :param name: Name of the service.
    :return: Callable that can be used to watch the service.
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
    A speaker is a publisher that will send messages to a topic. It is 
    used to publish simple data types to other variobotic os modules.

    :param name: Name of the service.
    :return: Callable that can be used to shout to the service.
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
        description=description,
        publisher_server=publisher_server,
    )

    vyra_speaker: VyraSpeaker = DataSpace.add_speaker(vyra_speaker)
    
    return vyra_speaker

def remote_callable(func):
    """
    Decorator to register a function or method as a V.Y.R.A. callable.
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

    # Registrierung erfolgt erst später in der Instanz
    wrapper._remote_callable = True  # Markiere, dass diese Methode registriert werden soll

    return wrapper