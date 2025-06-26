
# Standard library imports
import datetime
from typing import Union
from uuid import UUID

# Third party imports
import rclpy
from builtin_interfaces.msg import Time as BuiltinTime
from unique_identifier_msgs.msg import UUID as Ros2UUID


class Ros2TypeConverter:
    """
    A class to convert various python data types to ROS 2 compatible types.
    """
    @staticmethod
    def time_to_ros2buildintime(dt: datetime.datetime) -> BuiltinTime:
        ts = dt.timestamp()  # float-Sekunden seit Epoch
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        return BuiltinTime(sec=sec, nanosec=nsec)
    
    @staticmethod
    def ros2buildintime_to_datetime(builtin_time: BuiltinTime) -> datetime.datetime:
        """
        Converts a ROS 2 BuiltinTime to a datetime object.
        """
        return datetime.datetime.fromtimestamp(
            builtin_time.sec + builtin_time.nanosec * 1e-9,
            tz=datetime.timezone.utc
        )
    
    @staticmethod
    def uuid_to_ros2uuid(uuid: UUID) -> Ros2UUID:
        """
        Converts a UUID string to a ROS 2 compatible UUID string.
        """

        uuid_msg = Ros2UUID()
        uuid_msg.uuid = list(uuid.bytes)
        return uuid_msg
    
    @staticmethod
    def ros2uuid_to_uuid(ros2_uuid: Ros2UUID) -> UUID:
        """
        Converts a ROS 2 UUID string to a standard UUID string.
        """
        return UUID(bytes=bytes(ros2_uuid.uuid))
    