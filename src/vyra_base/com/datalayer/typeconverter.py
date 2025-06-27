import datetime
from typing import Union
from uuid import UUID

import rclpy
from builtin_interfaces.msg import Time as BuiltinTime
from unique_identifier_msgs.msg import UUID as Ros2UUID


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
        """
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
        """
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
        """
        uuid_msg = Ros2UUID()
        uuid_msg.uuid = list(uuid.bytes)
        return uuid_msg
    
    @staticmethod
    def ros2uuid_to_uuid(ros2_uuid: Ros2UUID) -> UUID:
        """
        Convert a ROS 2 :class:`unique_identifier_msgs.msg.UUID` to a :class:`uuid.UUID`.

        :param ros2_uuid: The ROS 2 UUID message to convert.
        :type ros2_uuid: unique_identifier_msgs.msg.UUID
        :return: The corresponding UUID object.
        :rtype: uuid.UUID
        """
        return UUID(bytes=bytes(ros2_uuid.uuid))