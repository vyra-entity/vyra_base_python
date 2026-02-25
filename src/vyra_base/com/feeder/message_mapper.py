"""
Message Mapper
==============

Converts between VYRA Python entries / wire-ready dicts and
protocol-specific message objects:

* **ROS2** (``*.msg``)  — field-name-based copy + type conversion
  (``datetime`` → ``builtin_interfaces/Time``, etc.)
* **Protobuf** (``*.proto``) — dict → Protobuf message via
  :class:`~vyra_base.com.converter.protobuf_converter.ProtobufConverter`

Subscriber direction (wire → Python dict) is also supported so that
callbacks always receive plain, de-serialized Python dicts regardless of
the underlying transport.

Usage
-----
::

    from vyra_base.com.feeder.message_mapper import MessageMapper

    # Build wire-ready dict from a NewsEntry (done by NewsFeeder)
    wire_dict = {
        "type": "INFO",
        "message": "hello",
        "timestamp": datetime.now(),
        "module_id": "abc",
        "module_name": "v2_my_module",
    }

    # Publisher direction ─ dict → ROS2 msg
    ros2_msg = MessageMapper.dict_to_ros2_msg(wire_dict, VBASENewsFeed)

    # Subscriber direction ─ ROS2 msg → Python dict
    data = MessageMapper.ros2_msg_to_dict(ros2_msg)

    # Publisher direction ─ dict → Protobuf
    proto_msg = MessageMapper.dict_to_proto_msg(wire_dict, VBASENewsFeed_pb2)

    # Subscriber direction ─ Protobuf → Python dict
    data = MessageMapper.proto_msg_to_dict(proto_msg)
"""
from __future__ import annotations

import logging
from datetime import datetime
from typing import Any, Optional

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Runtime-optional dependencies                                                #
# --------------------------------------------------------------------------- #
try:
    import rclpy  # noqa: F401
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

if _ROS2_AVAILABLE:
    try:
        from vyra_base.com.transport.t_ros2.communication.typeconverter import Ros2TypeConverter
    except ImportError:
        Ros2TypeConverter = None  # type: ignore[assignment]
else:
    Ros2TypeConverter = None  # type: ignore[assignment]

try:
    from vyra_base.com.converter.protobuf_converter import ProtobufConverter as _PBC
    _PROTO_CONVERTER: Optional[_PBC] = _PBC()
except Exception:
    _PROTO_CONVERTER = None


# --------------------------------------------------------------------------- #
# MessageMapper                                                                #
# --------------------------------------------------------------------------- #

class MessageMapper:
    """
    Central converter between VYRA Python entries / dicts and
    protocol-specific message objects.

    All methods are **class-methods / static-methods** — no instance required.

    Publisher direction (Python → wire)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    * :meth:`dict_to_ros2_msg`   – flat Python dict → ROS2 message instance
    * :meth:`dict_to_proto_msg`  – flat Python dict → Protobuf message instance

    Subscriber direction (wire → Python)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    * :meth:`ros2_msg_to_dict`   – ROS2 message instance → Python dict
    * :meth:`proto_msg_to_dict`  – Protobuf message instance → Python dict

    Convenience
    ~~~~~~~~~~~
    * :meth:`to_transport_msg`   – route to ros2/proto/passthrough by protocol
    * :meth:`from_transport_msg` – inverse routing
    """

    # ------------------------------------------------------------------ #
    # Publisher direction                                                  #
    # ------------------------------------------------------------------ #

    @classmethod
    def dict_to_ros2_msg(cls, data: dict, ros2_msg_class: type) -> Any:
        """
        Populate a new ROS2 message instance from a plain Python *dict*.

        The dict keys must already match the wire field names of the ROS2
        message (i.e. feeder-specific pre-processing has already been done
        by the concrete feeder's ``_prepare_entry_for_publish()`` hook).

        Special conversions applied automatically:
        * ``datetime`` value in a ``builtin_interfaces/Time`` field
          → :meth:`Ros2TypeConverter.time_to_ros2buildintime`
        * Numeric values in ``string`` fields → ``str(value)``
        * Enum ``.value`` extraction for numeric fields

        Args:
            data:           Wire-ready dict.  Keys = ROS2 field names.
            ros2_msg_class: The ROS2 message *class* (e.g. ``VBASENewsFeed``).

        Returns:
            Populated ROS2 message instance.
        """
        try:
            ros2_msg = ros2_msg_class()
        except Exception as exc:
            logger.error(f"❌ Cannot instantiate ROS2 msg class {ros2_msg_class}: {exc}")
            return data

        field_types: dict[str, str] = getattr(
            ros2_msg_class, "_fields_and_field_types", {}
        )

        for field_name, field_type in field_types.items():
            value = data.get(field_name)
            if value is None:
                continue

            converted = cls._convert_for_ros2_field(value, field_type)
            try:
                setattr(ros2_msg, field_name, converted)
            except Exception as exc:
                logger.debug(
                    f"⚠️ Could not set ROS2 field '{field_name}' "
                    f"(type={field_type}): {exc}"
                )

        return ros2_msg

    @classmethod
    def dict_to_proto_msg(cls, data: dict, proto_module: Any) -> Any:
        """
        Convert a plain Python *dict* to a Protobuf message instance.

        Finds the primary ``Message`` subclass in *proto_module*
        (the first non-private class that inherits from
        ``google.protobuf.message.Message``) and uses
        :class:`~vyra_base.com.converter.protobuf_converter.ProtobufConverter`
        to populate it from the dict via JSON round-trip.

        Args:
            data:         Wire-ready dict.
            proto_module: Loaded ``*_pb2`` module.

        Returns:
            Protobuf message instance, or *data* on failure.
        """
        if _PROTO_CONVERTER is None:
            logger.warning(
                "⚠️ ProtobufConverter not available — passing raw dict"
            )
            return data

        msg_class = cls._find_proto_message_class(proto_module)
        if msg_class is None:
            logger.warning(
                f"⚠️ No Protobuf message class found in module {proto_module}"
            )
            return data

        # Ensure values are JSON-serialisable
        safe_data = cls._sanitise_dict_for_json(data)
        try:
            return _PROTO_CONVERTER.dict_to_proto(safe_data, msg_class)
        except Exception as exc:
            logger.error(f"❌ dict_to_proto_msg failed: {exc}")
            return data

    @classmethod
    def to_transport_msg(
        cls,
        data: dict,
        protocol: str,
        msg_type: Optional[Any],
    ) -> Any:
        """
        Route conversion to the correct backend based on *protocol*.

        * ``"ros2"``                   → :meth:`dict_to_ros2_msg`
        * ``"zenoh"`` / ``"redis"`` / ``"uds"``
          with a proto module (has ``DESCRIPTOR`` attr) → :meth:`dict_to_proto_msg`
        * Everything else              → return *data* unchanged
          (Zenoh/Redis JSON serialisers handle plain dicts natively)

        Args:
            data:      Wire-ready dict.
            protocol:  Resolved protocol string (case-insensitive).
            msg_type:  ROS2 class **or** proto ``*_pb2`` module **or** ``None``.

        Returns:
            Protocol message object, or *data* if no conversion is needed.
        """
        if msg_type is None:
            return data

        # If data is already the right transport type, short-circuit
        try:
            if isinstance(data, msg_type):
                return data
        except TypeError:
            pass  # msg_type may be a module, not a class

        # Only convert dicts via the mappers
        if not isinstance(data, dict):
            return data

        proto_lower = (protocol or "").lower()

        if proto_lower == "ros2":
            return cls.dict_to_ros2_msg(data, msg_type)

        if proto_lower in ("zenoh", "redis", "uds"):
            # Only convert if caller actually provided a proto _pb2 module
            if cls._is_proto_module(msg_type):
                return cls.dict_to_proto_msg(data, msg_type)
            # otherwise plain dict is fine — Zenoh JSON serialiser handles it

        return data

    # ------------------------------------------------------------------ #
    # Subscriber direction                                                 #
    # ------------------------------------------------------------------ #

    @classmethod
    def ros2_msg_to_dict(cls, msg: Any) -> dict:
        """
        Convert a ROS2 message instance to a Python dict.

        * ``builtin_interfaces/Time`` fields are converted back to ``datetime``.
        * All other values are returned as-is.

        Args:
            msg: ROS2 message instance.

        Returns:
            Python dict with field values.
        """
        result: dict = {}
        field_types: dict[str, str] = getattr(type(msg), "_fields_and_field_types", {})
        for field_name, field_type in field_types.items():
            value = getattr(msg, field_name, None)
            result[field_name] = cls._convert_from_ros2_field(value, field_type)
        return result

    @classmethod
    def proto_msg_to_dict(cls, msg: Any) -> dict:
        """
        Convert a Protobuf message instance to a Python dict.

        Args:
            msg: Protobuf message instance.

        Returns:
            Python dict, or empty dict on failure.
        """
        if _PROTO_CONVERTER is None:
            return {}
        try:
            return _PROTO_CONVERTER.proto_to_dict(msg)
        except Exception as exc:
            logger.error(f"❌ proto_msg_to_dict failed: {exc}")
            return {}

    @classmethod
    def from_transport_msg(cls, msg: Any, protocol: str) -> Any:
        """
        Route deserialization from the correct backend based on *protocol*.

        * ``"ros2"`` → :meth:`ros2_msg_to_dict`
        * Otherwise  → return *msg* unchanged
          (Zenoh/Redis already deliver dicts or bytes handled upstream)

        Args:
            msg:      Received message (ROS2 msg, proto msg, dict, or bytes).
            protocol: Resolved protocol string (case-insensitive).

        Returns:
            Python dict or original *msg*.
        """
        proto_lower = (protocol or "").lower()

        if proto_lower == "ros2" and hasattr(type(msg), "_fields_and_field_types"):
            return cls.ros2_msg_to_dict(msg)

        if proto_lower in ("zenoh", "redis", "uds"):
            if cls._is_proto_message(msg):
                return cls.proto_msg_to_dict(msg)

        return msg  # already a dict or handled by transport layer

    # ------------------------------------------------------------------ #
    # Private helpers                                                      #
    # ------------------------------------------------------------------ #

    @classmethod
    def _convert_for_ros2_field(cls, value: Any, field_type: str) -> Any:
        """Apply type coercions needed for a specific ROS2 field type."""
        if value is None:
            return value

        # datetime → builtin_interfaces/Time
        if isinstance(value, datetime) and "Time" in field_type:
            if _ROS2_AVAILABLE and Ros2TypeConverter:
                return Ros2TypeConverter.time_to_ros2buildintime(value)
            return value

        # string field: coerce everything to str
        if field_type == "string":
            if isinstance(value, bytes):
                return value.decode("utf-8", errors="replace")
            # Extract enum .value first
            raw = getattr(value, "value", value)
            return str(raw) if not isinstance(raw, str) else raw

        # uint/int fields: extract .value from enums
        if field_type.startswith(("uint", "int")):
            if hasattr(value, "value"):
                return int(value.value)
            try:
                return int(value)
            except (TypeError, ValueError):
                return value

        return value

    @classmethod
    def _convert_from_ros2_field(cls, value: Any, field_type: str) -> Any:
        """Undo ROS2-specific type coercions when converting back to Python."""
        if value is None:
            return value

        # ROS2 builtin Time → datetime
        if "Time" in field_type and _ROS2_AVAILABLE and Ros2TypeConverter:
            if hasattr(value, "sec") and hasattr(value, "nanosec"):
                try:
                    return Ros2TypeConverter.ros2buildintime_to_datetime(value)
                except Exception:
                    pass

        return value

    @staticmethod
    def _sanitise_dict_for_json(data: dict) -> dict:
        """Return a copy of *data* with values that are JSON-serialisable."""
        import uuid as _uuid

        result: dict = {}
        for k, v in data.items():
            if isinstance(v, datetime):
                result[k] = v.isoformat()
            elif isinstance(v, _uuid.UUID):
                result[k] = str(v)
            elif hasattr(v, "value"):          # Enum
                result[k] = v.value
            elif hasattr(v, "sec"):            # builtin_interfaces/Time
                result[k] = v.sec + v.nanosec * 1e-9
            else:
                result[k] = v
        return result

    @staticmethod
    def _find_proto_message_class(proto_module: Any):  # → Optional[type]
        """Return the first ``google.protobuf.message.Message`` subclass found."""
        try:
            from google.protobuf.message import Message as _ProtoBase
            for attr_name in dir(proto_module):
                if attr_name.startswith("_"):
                    continue
                attr = getattr(proto_module, attr_name, None)
                if isinstance(attr, type) and issubclass(attr, _ProtoBase):
                    return attr
        except ImportError:
            pass
        return None

    @staticmethod
    def _is_proto_module(obj: Any) -> bool:
        """Return True if *obj* looks like a ``*_pb2`` protobuf module."""
        return (
            hasattr(obj, "DESCRIPTOR") is False        # not a message *instance*
            and hasattr(obj, "__file__")               # is a module
            and getattr(obj, "__name__", "").endswith("_pb2")
        )

    @staticmethod
    def _is_proto_message(obj: Any) -> bool:
        """Return True if *obj* is a Protobuf message *instance*."""
        try:
            from google.protobuf.message import Message as _ProtoBase
            return isinstance(obj, _ProtoBase)
        except ImportError:
            return False
