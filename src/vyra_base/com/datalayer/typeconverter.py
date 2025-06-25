
import datetime
import rclpy

from builtin_interfaces.msg import Time as BuiltinTime


class Ros2TypeConverter:
    """
    A class to convert various python data types to ROS 2 compatible types.
    """
    @staticmethod
    def time_to_ros2_buildin_time(dt: datetime.datetime) -> BuiltinTime:
        ts = dt.timestamp()  # float-Sekunden seit Epoch
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        return BuiltinTime(sec=sec, nanosec=nsec)
    
    @staticmethod
    def ros2_buildin_time_to_datetime(builtin_time: BuiltinTime) -> datetime.datetime:
        """
        Converts a ROS 2 BuiltinTime to a datetime object.
        """
        return datetime.datetime.fromtimestamp(
            builtin_time.sec + builtin_time.nanosec * 1e-9,
            tz=datetime.timezone.utc
        )