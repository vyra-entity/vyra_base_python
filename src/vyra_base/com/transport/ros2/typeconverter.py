"""
ROS2 Type Converter

Utilities for converting Python types to ROS2 types and vice versa.
"""
import datetime
from typing import Union
from uuid import UUID

try:
    import rclpy
    from builtin_interfaces.msg import Time as BuiltinTime
    from unique_identifier_msgs.msg import UUID as Ros2UUID
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    BuiltinTime = None
    Ros2UUID = None


class Ros2TypeConverter:
    """
    A class to convert various Python data types to ROS 2 compatible types.
    """
    
    @staticmethod
    def time_to_ros2buildintime(dt: datetime.datetime) -> BuiltinTime:
        """
        Convert a :class:`datetime.datetime` object to a ROS 2 :class:`builtin_interfaces.msg.Time`.

        :param dt: The datetime object to convert.
        :type dt: datetime.datetime
        :return: The corresponding ROS 2 BuiltinTime message.
        :rtype: builtin_interfaces.msg.Time
        :raises ImportError: If ROS2 is not available
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available. Install rclpy to use this function.")
        
        ts = dt.timestamp()  # float-Sekunden seit Epoch
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        return BuiltinTime(sec=sec, nanosec=nsec)
    
    @staticmethod
    def ros2buildintime_to_datetime(builtin_time: BuiltinTime) -> datetime.datetime:
        """
        Convert a ROS 2 :class:`builtin_interfaces.msg.Time` to a :class:`datetime.datetime` object.

        :param builtin_time: The ROS 2 BuiltinTime message to convert.
        :type builtin_time: builtin_interfaces.msg.Time
        :return: The corresponding datetime object (UTC).
        :rtype: datetime.datetime
        :raises ImportError: If ROS2 is not available
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available. Install rclpy to use this function.")
        
        return datetime.datetime.fromtimestamp(
            builtin_time.sec + builtin_time.nanosec * 1e-9,
            tz=datetime.timezone.utc
        )
    
    @staticmethod
    def uuid_to_ros2uuid(uuid: UUID) -> Ros2UUID:
        """
        Convert a :class:`uuid.UUID` to a ROS 2 :class:`unique_identifier_msgs.msg.UUID`.

        :param uuid: The UUID to convert.
        :type uuid: uuid.UUID
        :return: The corresponding ROS 2 UUID message.
        :rtype: unique_identifier_msgs.msg.UUID
        :raises ImportError: If ROS2 is not available
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available. Install rclpy to use this function.")
        
        ros2_uuid = Ros2UUID()
        ros2_uuid.uuid = list(uuid.bytes)
        return ros2_uuid
    
    @staticmethod
    def ros2uuid_to_uuid(ros2_uuid: Ros2UUID) -> UUID:
        """
        Convert a ROS 2 :class:`unique_identifier_msgs.msg.UUID` to a :class:`uuid.UUID`.

        :param ros2_uuid: The ROS 2 UUID message to convert.
        :type ros2_uuid: unique_identifier_msgs.msg.UUID
        :return: The corresponding UUID.
        :rtype: uuid.UUID
        :raises ImportError: If ROS2 is not available
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available. Install rclpy to use this function.")
        
        return UUID(bytes=bytes(ros2_uuid.uuid))
