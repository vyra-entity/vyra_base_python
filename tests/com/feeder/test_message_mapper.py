"""
Tests for vyra_base.com.feeder.message_mapper.MessageMapper
"""
import sys
from datetime import datetime
from unittest.mock import MagicMock, patch

import pytest

from vyra_base.com.feeder.message_mapper import MessageMapper


# ── Helper mock classes ───────────────────────────────────────────────────────

class MockRos2Msg:
    """Minimal mock of a ROS2 message class/instance."""
    _fields_and_field_types = {
        "name": "string",
        "count": "uint32",
        "value": "float64",
        "data": "string",
    }

    def __init__(self):
        self.name = ""
        self.count = 0
        self.value = 0.0
        self.data = ""


class MockRos2MsgWithTime:
    """Mock ROS2 message with a Time field."""
    _fields_and_field_types = {
        "timestamp": "builtin_interfaces/Time",
        "name": "string",
    }

    def __init__(self):
        self.timestamp = None
        self.name = ""


# ── dict_to_ros2_msg tests ────────────────────────────────────────────────────

class TestDictToRos2Msg:
    def test_basic_string_field(self):
        data = {"name": "hello", "count": 5}
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.name == "hello"
        assert result.count == 5

    def test_missing_field_skipped(self):
        data = {"name": "test"}  # 'count' missing
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.name == "test"
        assert result.count == 0  # default unchanged

    def test_numeric_to_string_field(self):
        """Numeric value should be str-coerced for 'string' field type."""
        data = {"data": 42}
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.data == "42"

    def test_enum_value_for_string_field(self):
        """Enum with .value should be extracted then str'd for string fields."""
        class FakeEnum:
            value = "auto"

        data = {"data": FakeEnum()}
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.data == "auto"

    def test_bytes_to_string_field(self):
        data = {"data": b"hello"}
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.data == "hello"

    def test_enum_value_for_uint_field(self):
        """Enum .value should be int-coerced for uint fields."""
        class FakeEnum:
            value = 7

        data = {"count": FakeEnum()}
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        assert result.count == 7

    def test_invalid_value_for_uint_field(self):
        """Non-numeric value for uint field should be returned as-is."""
        data = {"count": "not_a_number"}
        # Should not raise, just log
        result = MessageMapper.dict_to_ros2_msg(data, MockRos2Msg)
        # count will be attempted to be int but fail
        # The field set might fail and be logged — result has original data
        assert isinstance(result, MockRos2Msg)

    def test_instantiation_failure_returns_raw_data(self):
        """If msg class can't be instantiated, return raw dict."""
        class BrokenMsg:
            _fields_and_field_types = {}

            def __init__(self):
                raise RuntimeError("Cannot instantiate")

        data = {"name": "test"}
        result = MessageMapper.dict_to_ros2_msg(data, BrokenMsg)
        assert result == data

    def test_datetime_field_without_ros2(self):
        """When ROS2 is unavailable, datetime value passes through."""
        import vyra_base.com.feeder.message_mapper as mod
        original = mod._ROS2_AVAILABLE
        try:
            mod._ROS2_AVAILABLE = False
            data = {"timestamp": datetime(2024, 1, 1)}
            result = MessageMapper.dict_to_ros2_msg(data, MockRos2MsgWithTime)
            # Timestamp should pass through unchanged (no Ros2TypeConverter)
            assert result.timestamp == datetime(2024, 1, 1)
        finally:
            mod._ROS2_AVAILABLE = original


# ── ros2_msg_to_dict tests ────────────────────────────────────────────────────

class TestRos2MsgToDict:
    def test_basic_message(self):
        msg = MockRos2Msg()
        msg.name = "test"
        msg.count = 10

        result = MessageMapper.ros2_msg_to_dict(msg)
        assert result["name"] == "test"
        assert result["count"] == 10

    def test_empty_message(self):
        msg = MockRos2Msg()
        result = MessageMapper.ros2_msg_to_dict(msg)
        assert "name" in result
        assert "count" in result

    def test_message_without_field_types(self):
        """Object without _fields_and_field_types returns empty dict."""
        class NoFieldTypes:
            name = "hello"

        msg = NoFieldTypes()
        result = MessageMapper.ros2_msg_to_dict(msg)
        assert result == {}

    def test_time_field_without_ros2(self):
        """When ROS2 unavailable, Time field is returned as-is."""
        import vyra_base.com.feeder.message_mapper as mod
        original = mod._ROS2_AVAILABLE

        class MockTime:
            sec = 1000
            nanosec = 0

        try:
            mod._ROS2_AVAILABLE = False
            msg = MockRos2MsgWithTime()
            msg.timestamp = MockTime()
            msg.name = "x"
            result = MessageMapper.ros2_msg_to_dict(msg)
            assert isinstance(result["timestamp"], MockTime)
        finally:
            mod._ROS2_AVAILABLE = original


# ── to_transport_msg tests ────────────────────────────────────────────────────

class TestToTransportMsg:
    def test_none_msg_type_returns_data(self):
        data = {"key": "value"}
        result = MessageMapper.to_transport_msg(data, "ros2", None)
        assert result == data

    def test_non_dict_data_returns_unchanged(self):
        """Non-dict input is returned unchanged."""
        result = MessageMapper.to_transport_msg("raw_string", "ros2", MockRos2Msg)
        assert result == "raw_string"

    def test_already_correct_type_returns_same(self):
        """If data is already instance of msg_type, return as-is."""
        msg = MockRos2Msg()
        result = MessageMapper.to_transport_msg(msg, "ros2", MockRos2Msg)
        assert result is msg

    def test_ros2_protocol_calls_dict_to_ros2(self):
        data = {"name": "hello", "count": 3}
        result = MessageMapper.to_transport_msg(data, "ros2", MockRos2Msg)
        assert isinstance(result, MockRos2Msg)
        assert result.name == "hello"

    def test_ros2_protocol_case_insensitive(self):
        data = {"name": "hello"}
        result = MessageMapper.to_transport_msg(data, "ROS2", MockRos2Msg)
        assert isinstance(result, MockRos2Msg)

    def test_redis_protocol_no_proto_returns_data(self):
        """Redis with non-proto msg_type returns data unchanged."""
        data = {"key": "value"}
        result = MessageMapper.to_transport_msg(data, "redis", MockRos2Msg)
        assert result == data

    def test_unknown_protocol_returns_data(self):
        data = {"key": "value"}
        result = MessageMapper.to_transport_msg(data, "unknown_proto", MockRos2Msg)
        assert result == data


# ── from_transport_msg tests ──────────────────────────────────────────────────

class TestFromTransportMsg:
    def test_ros2_message_converts_to_dict(self):
        msg = MockRos2Msg()
        msg.name = "test"
        result = MessageMapper.from_transport_msg(msg, "ros2")
        assert isinstance(result, dict)
        assert result["name"] == "test"

    def test_non_ros2_returns_unchanged(self):
        data = {"key": "value"}
        result = MessageMapper.from_transport_msg(data, "redis")
        assert result is data

    def test_redis_protocol_non_proto_returns_unchanged(self):
        data = {"key": "value"}
        result = MessageMapper.from_transport_msg(data, "redis")
        assert result == data

    def test_empty_protocol_returns_unchanged(self):
        data = {"key": "value"}
        result = MessageMapper.from_transport_msg(data, "")
        assert result == data

    def test_ros2_dict_without_field_types_returns_data(self):
        """A plain dict has no _fields_and_field_types so from_transport_msg returns it unchanged."""
        data = {"name": "hello"}
        result = MessageMapper.from_transport_msg(data, "ros2")
        # Plain dict has no _fields_and_field_types, condition fails → returns msg unchanged
        assert result == data


# ── _sanitise_dict_for_json tests ─────────────────────────────────────────────

class TestSanitiseDictForJson:
    def test_datetime_to_isoformat(self):
        import uuid
        dt = datetime(2024, 1, 15, 10, 30)
        result = MessageMapper._sanitise_dict_for_json({"ts": dt})
        assert result["ts"] == dt.isoformat()

    def test_uuid_to_str(self):
        import uuid
        u = uuid.uuid4()
        result = MessageMapper._sanitise_dict_for_json({"id": u})
        assert result["id"] == str(u)

    def test_enum_value_extracted(self):
        class FakeEnum:
            value = "active"

        result = MessageMapper._sanitise_dict_for_json({"status": FakeEnum()})
        assert result["status"] == "active"

    def test_plain_values_passthrough(self):
        result = MessageMapper._sanitise_dict_for_json({"a": 1, "b": "hello"})
        assert result == {"a": 1, "b": "hello"}

    def test_builtin_time_to_float(self):
        """An object with .sec and .nanosec is converted to float."""
        class FakeTime:
            sec = 1000
            nanosec = 500000000  # 0.5 sec

        result = MessageMapper._sanitise_dict_for_json({"ts": FakeTime()})
        assert abs(result["ts"] - 1000.5) < 0.0001


# ── _is_proto_module tests ────────────────────────────────────────────────────

class TestIsProtoModule:
    def test_non_module_returns_false(self):
        assert not MessageMapper._is_proto_module("string")

    def test_regular_module_not_pb2_returns_false(self):
        import os
        assert not MessageMapper._is_proto_module(os)

    def test_pb2_like_module_returns_true(self):
        """A module with name ending in _pb2 and __file__ is proto."""
        mock_mod = MagicMock()
        mock_mod.__file__ = "/path/to/some_pb2.py"
        mock_mod.__name__ = "some_pb2"
        # Has no DESCRIPTOR attr (not an instance), has __file__
        del mock_mod.DESCRIPTOR
        assert MessageMapper._is_proto_module(mock_mod)


# ── _is_proto_message tests ───────────────────────────────────────────────────

class TestIsProtoMessage:
    def test_non_proto_object_returns_false(self):
        assert not MessageMapper._is_proto_message("hello")
        assert not MessageMapper._is_proto_message(42)
        assert not MessageMapper._is_proto_message({})

    def test_none_returns_false(self):
        assert not MessageMapper._is_proto_message(None)


# ── dict_to_proto_msg without protobuf ───────────────────────────────────────

class TestDictToProtoMsgNoProto:
    def test_no_proto_converter_returns_data(self):
        """When ProtobufConverter is unavailable, raw dict is returned."""
        import vyra_base.com.feeder.message_mapper as mod
        original = mod._PROTO_CONVERTER

        try:
            mod._PROTO_CONVERTER = None
            data = {"key": "value"}
            result = MessageMapper.dict_to_proto_msg(data, MagicMock())
            assert result == data
        finally:
            mod._PROTO_CONVERTER = original

    def test_no_proto_class_found_returns_data(self):
        """When no Protobuf message class found in module, raw dict is returned."""
        import vyra_base.com.feeder.message_mapper as mod
        original = mod._PROTO_CONVERTER

        try:
            # Create a fake converter that's not None
            fake_converter = MagicMock()
            mod._PROTO_CONVERTER = fake_converter

            # Module with no proto message class
            empty_module = MagicMock(spec=[])  # no proto message classes

            data = {"key": "value"}
            result = MessageMapper.dict_to_proto_msg(data, empty_module)
            assert result == data
        finally:
            mod._PROTO_CONVERTER = original


# ── proto_msg_to_dict without protobuf ───────────────────────────────────────

class TestProtoMsgToDictNoProto:
    def test_no_proto_converter_returns_empty(self):
        import vyra_base.com.feeder.message_mapper as mod
        original = mod._PROTO_CONVERTER

        try:
            mod._PROTO_CONVERTER = None
            result = MessageMapper.proto_msg_to_dict(MagicMock())
            assert result == {}
        finally:
            mod._PROTO_CONVERTER = original
