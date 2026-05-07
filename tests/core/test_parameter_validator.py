"""
Tests for vyra_base.core.parameter_validator
"""
import json
import pytest

from vyra_base.core.parameter_validator import (
    ParameterValidator,
    _convert_value,
    _parse_min_max,
    _load_default_rules,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def validator():
    """Create a ParameterValidator with minimal inline rules."""
    return ParameterValidator(rules={
        "create": {
            "name": {
                "required": True,
                "pattern": r"^[a-zA-Z_][a-zA-Z0-9_]*$",
                "max_length": 64,
                "message_invalid_chars": "Invalid parameter name '{name}'.",
                "message_too_long": "Parameter name exceeds {max_length} characters.",
            },
            "description": {
                "required": True,
                "message_required": "description is required.",
            },
            "default_value": {
                "required": True,
                "message_required": "default_value is required.",
            },
            "types": {
                "allowed": ["integer", "float", "string", "boolean", "list", "dict"],
                "message_invalid": "Invalid type '{type}'. Must be one of: {allowed_csv}.",
            },
            "displayname": {
                "max_length": 255,
                "message_too_long": "displayname exceeds {max_length} characters.",
            },
            "numeric": {
                "message_below_min": "default_value {value} is below min_value {min}.",
                "message_above_max": "default_value {value} exceeds max_value {max}.",
            },
            "list_length": {
                "message_below_min": "List length {length} is below min_value {min}.",
                "message_above_max": "List length {length} exceeds max_value {max}.",
            },
            "range_value": {
                "allowed_keys": ["allowed_values", "allowed_types"],
                "message_unknown_keys": "range_value contains unknown keys: {unknown_keys}.",
            },
        },
        "set": {
            "type_check": {
                "enabled": True,
                "message_wrong_type": "Value '{value}' cannot be converted to type '{expected_type}': {error}",
            },
            "min_max": {
                "enabled": True,
                "message_below_min": "Value {value} is below min_value {min}.",
                "message_above_max": "Value {value} exceeds max_value {max}.",
                "message_non_numeric": "min_value/max_value must be numeric: {error}",
                "message_list_below_min": "List length {length} is below min_value {min}.",
                "message_list_above_max": "List length {length} exceeds max_value {max}.",
            },
            "range_value": {
                "enabled": True,
                "message_not_in_allowed": "Value '{value}' is not in allowed_values {allowed}.",
            },
        },
    })


# ── _convert_value tests ──────────────────────────────────────────────────────

class TestConvertValue:
    def test_convert_integer(self):
        val, err = _convert_value("42", "integer")
        assert val == 42
        assert err is None

    def test_convert_int(self):
        val, err = _convert_value(10, "int")
        assert val == 10
        assert err is None

    def test_convert_float(self):
        val, err = _convert_value("3.14", "float")
        assert abs(val - 3.14) < 0.001
        assert err is None

    def test_convert_boolean_true_string(self):
        for s in ("true", "True", "TRUE", "1", "yes"):
            val, err = _convert_value(s, "boolean")
            assert val is True
            assert err is None

    def test_convert_boolean_false_string(self):
        val, err = _convert_value("false", "boolean")
        assert val is False
        assert err is None

    def test_convert_boolean_from_bool(self):
        val, err = _convert_value(True, "bool")
        assert val is True
        assert err is None

    def test_convert_boolean_from_int(self):
        val, err = _convert_value(0, "boolean")
        assert val is False
        assert err is None

    def test_convert_list_from_list(self):
        val, err = _convert_value([1, 2, 3], "list")
        assert val == [1, 2, 3]
        assert err is None

    def test_convert_list_from_json_string(self):
        val, err = _convert_value('[1, 2, 3]', "list")
        assert val == [1, 2, 3]
        assert err is None

    def test_convert_list_invalid_json(self):
        val, err = _convert_value("not a list", "list")
        assert val is None
        assert err is not None

    def test_convert_list_not_list_type(self):
        val, err = _convert_value('{"a":1}', "list")
        assert val is None
        assert err is not None

    def test_convert_dict_from_dict(self):
        val, err = _convert_value({"a": 1}, "dict")
        assert val == {"a": 1}
        assert err is None

    def test_convert_dict_from_json_string(self):
        val, err = _convert_value('{"a": 1}', "dict")
        assert val == {"a": 1}
        assert err is None

    def test_convert_dict_not_dict_type(self):
        val, err = _convert_value('[1, 2]', "dict")
        assert val is None
        assert err is not None

    def test_convert_string_default(self):
        val, err = _convert_value(42, "string")
        assert val == "42"
        assert err is None

    def test_convert_unknown_type_returns_string(self):
        val, err = _convert_value(99, "unknown_type")
        assert val == "99"
        assert err is None

    def test_convert_integer_invalid(self):
        val, err = _convert_value("not_int", "integer")
        assert val is None
        assert err is not None


# ── _parse_min_max tests ──────────────────────────────────────────────────────

class TestParseMinMax:
    def test_both_numeric(self):
        mn, mx, err = _parse_min_max(0, 100)
        assert mn == 0.0
        assert mx == 100.0
        assert err is None

    def test_none_values(self):
        mn, mx, err = _parse_min_max(None, None)
        assert mn is None
        assert mx is None
        assert err is None

    def test_empty_string_values(self):
        mn, mx, err = _parse_min_max("", "")
        assert mn is None
        assert mx is None
        assert err is None

    def test_invalid_min(self):
        mn, mx, err = _parse_min_max("bad", 10)
        assert err is not None

    def test_string_numbers(self):
        mn, mx, err = _parse_min_max("5", "20")
        assert mn == 5.0
        assert mx == 20.0
        assert err is None


# ── ParameterValidator.validate_create tests ─────────────────────────────────

class TestValidateCreate:
    def test_valid_integer_param(self, validator):
        errors = validator.validate_create(
            name="speed",
            default_value=10,
            type_str="integer",
            description="Speed in m/s",
        )
        assert errors == []

    def test_missing_name_returns_error(self, validator):
        errors = validator.validate_create(
            name="",
            default_value=10,
            type_str="integer",
            description="desc",
        )
        assert any("name" in e.lower() or "required" in e.lower() for e in errors)

    def test_invalid_name_chars(self, validator):
        errors = validator.validate_create(
            name="1invalid",
            default_value=10,
            type_str="integer",
            description="desc",
        )
        assert len(errors) > 0
        assert "1invalid" in errors[0]

    def test_name_too_long(self, validator):
        long_name = "a" * 65
        errors = validator.validate_create(
            name=long_name,
            default_value=10,
            type_str="integer",
            description="desc",
        )
        assert any("64" in e for e in errors)

    def test_missing_description(self, validator):
        errors = validator.validate_create(
            name="my_param",
            default_value=10,
            type_str="integer",
            description="",
        )
        assert any("description" in e.lower() for e in errors)

    def test_missing_default_value(self, validator):
        errors = validator.validate_create(
            name="my_param",
            default_value=None,
            type_str="integer",
            description="desc",
        )
        assert len(errors) > 0

    def test_invalid_type(self, validator):
        errors = validator.validate_create(
            name="my_param",
            default_value=10,
            type_str="nonexistent_type",
            description="desc",
        )
        assert any("nonexistent_type" in e for e in errors)

    def test_value_below_min(self, validator):
        errors = validator.validate_create(
            name="speed",
            default_value=-5,
            type_str="integer",
            description="desc",
            min_value=0,
        )
        assert any("below" in e.lower() for e in errors)

    def test_value_above_max(self, validator):
        errors = validator.validate_create(
            name="speed",
            default_value=200,
            type_str="integer",
            description="desc",
            max_value=100,
        )
        assert any("exceed" in e.lower() for e in errors)

    def test_value_within_range(self, validator):
        errors = validator.validate_create(
            name="speed",
            default_value=50,
            type_str="integer",
            description="desc",
            min_value=0,
            max_value=100,
        )
        assert errors == []

    def test_float_param_valid(self, validator):
        errors = validator.validate_create(
            name="temperature",
            default_value=22.5,
            type_str="float",
            description="Temperature in Celsius",
        )
        assert errors == []

    def test_string_param_valid(self, validator):
        errors = validator.validate_create(
            name="mode",
            default_value="auto",
            type_str="string",
            description="Operation mode",
        )
        assert errors == []

    def test_boolean_param_valid(self, validator):
        errors = validator.validate_create(
            name="enabled",
            default_value=True,
            type_str="boolean",
            description="Enabled flag",
        )
        assert errors == []

    def test_list_param_valid(self, validator):
        errors = validator.validate_create(
            name="items",
            default_value=[1, 2, 3],
            type_str="list",
            description="Item list",
        )
        assert errors == []

    def test_list_length_below_min(self, validator):
        errors = validator.validate_create(
            name="items",
            default_value=[1],
            type_str="list",
            description="desc",
            min_value=3,
        )
        assert any("below" in e.lower() or "length" in e.lower() for e in errors)

    def test_list_length_above_max(self, validator):
        errors = validator.validate_create(
            name="items",
            default_value=[1, 2, 3, 4, 5],
            type_str="list",
            description="desc",
            max_value=3,
        )
        assert any("exceed" in e.lower() or "length" in e.lower() for e in errors)

    def test_dict_param_valid(self, validator):
        errors = validator.validate_create(
            name="config",
            default_value={"key": "value"},
            type_str="dict",
            description="Configuration",
        )
        assert errors == []

    def test_displayname_too_long(self, validator):
        errors = validator.validate_create(
            name="my_param",
            default_value=10,
            type_str="integer",
            description="desc",
            displayname="d" * 256,
        )
        assert any("displayname" in e.lower() or "255" in e for e in errors)

    def test_range_value_dict_valid(self, validator):
        errors = validator.validate_create(
            name="mode",
            default_value="auto",
            type_str="string",
            description="desc",
            range_value={"allowed_values": ["auto", "manual"]},
        )
        assert errors == []

    def test_range_value_unknown_keys(self, validator):
        errors = validator.validate_create(
            name="mode",
            default_value="auto",
            type_str="string",
            description="desc",
            range_value={"unknown_key": ["auto"]},
        )
        assert any("unknown" in e.lower() for e in errors)

    def test_range_value_invalid_json_string(self, validator):
        errors = validator.validate_create(
            name="mode",
            default_value="auto",
            type_str="string",
            description="desc",
            range_value="not_valid_json",
        )
        assert any("json" in e.lower() or "valid" in e.lower() for e in errors)

    def test_range_value_valid_json_string(self, validator):
        errors = validator.validate_create(
            name="mode",
            default_value="auto",
            type_str="string",
            description="desc",
            range_value=json.dumps({"allowed_values": ["auto", "manual"]}),
        )
        assert errors == []


# ── ParameterValidator.validate_set tests ────────────────────────────────────

class TestValidateSet:
    def _make_record(self, type_str, min_value=None, max_value=None, range_value=None):
        """Create a dict-style parameter record."""
        return {
            "type": type_str,
            "min_value": min_value,
            "max_value": max_value,
            "range_value": range_value,
        }

    def test_valid_integer_set(self, validator):
        record = self._make_record("integer", min_value=0, max_value=100)
        errors = validator.validate_set(param_record=record, new_value="50")
        assert errors == []

    def test_integer_below_min(self, validator):
        record = self._make_record("integer", min_value=0)
        errors = validator.validate_set(param_record=record, new_value="-10")
        assert any("below" in e.lower() for e in errors)

    def test_integer_above_max(self, validator):
        record = self._make_record("integer", max_value=100)
        errors = validator.validate_set(param_record=record, new_value="200")
        assert any("exceed" in e.lower() for e in errors)

    def test_wrong_type_rejected(self, validator):
        record = self._make_record("integer")
        errors = validator.validate_set(param_record=record, new_value="not_a_number")
        assert len(errors) > 0

    def test_string_set_valid(self, validator):
        record = self._make_record("string")
        errors = validator.validate_set(param_record=record, new_value="hello")
        assert errors == []

    def test_boolean_set_valid(self, validator):
        record = self._make_record("boolean")
        errors = validator.validate_set(param_record=record, new_value="true")
        assert errors == []

    def test_allowed_values_valid(self, validator):
        record = self._make_record(
            "string",
            range_value={"allowed_values": ["auto", "manual"]}
        )
        errors = validator.validate_set(param_record=record, new_value="auto")
        assert errors == []

    def test_allowed_values_invalid(self, validator):
        record = self._make_record(
            "string",
            range_value={"allowed_values": ["auto", "manual"]}
        )
        errors = validator.validate_set(param_record=record, new_value="unknown_mode")
        assert any("allowed" in e.lower() for e in errors)

    def test_range_value_as_json_string(self, validator):
        record = self._make_record(
            "string",
            range_value=json.dumps({"allowed_values": ["a", "b"]})
        )
        errors = validator.validate_set(param_record=record, new_value="c")
        assert any("allowed" in e.lower() for e in errors)

    def test_list_set_below_min_length(self, validator):
        record = self._make_record("list", min_value=3)
        errors = validator.validate_set(param_record=record, new_value=[1])
        assert any("below" in e.lower() or "length" in e.lower() for e in errors)

    def test_list_set_above_max_length(self, validator):
        record = self._make_record("list", max_value=2)
        errors = validator.validate_set(param_record=record, new_value=[1, 2, 3, 4])
        assert any("exceed" in e.lower() or "length" in e.lower() for e in errors)

    def test_set_with_orm_like_object(self, validator):
        """Test with object-style param record (not dict)."""
        class MockRecord:
            type = "integer"
            min_value = 0
            max_value = 100
            range_value = None

        errors = validator.validate_set(param_record=MockRecord(), new_value="50")
        assert errors == []

    def test_set_enum_type_attribute(self, validator):
        """Test with an enum-like type attribute having .value."""
        class TypeEnum:
            value = "integer"

        class MockRecord:
            type = TypeEnum()
            min_value = None
            max_value = None
            range_value = None

        errors = validator.validate_set(param_record=MockRecord(), new_value="42")
        assert errors == []

    def test_float_set_valid(self, validator):
        record = self._make_record("float", min_value=0.0, max_value=10.0)
        errors = validator.validate_set(param_record=record, new_value="5.5")
        assert errors == []

    def test_set_non_numeric_min_max(self, validator):
        """Cover the num_error branch in validate_set min_max."""
        record = self._make_record("integer", min_value="bad_min", max_value=None)
        errors = validator.validate_set(param_record=record, new_value="42")
        assert any("numeric" in e.lower() or "min" in e.lower() for e in errors)

    def test_type_check_disabled_still_validates(self):
        """Test validate_set with type_check disabled."""
        v = ParameterValidator(rules={
            "set": {
                "type_check": {"enabled": False},
                "min_max": {"enabled": False},
                "range_value": {"enabled": False},
            }
        })
        errors = v.validate_set(
            param_record={"type": "integer", "min_value": None, "max_value": None, "range_value": None},
            new_value="not_a_number"
        )
        assert errors == []

    def test_range_value_invalid_json_string_skips(self, validator):
        """Test that invalid JSON in range_value is silently skipped."""
        record = self._make_record("string", range_value="not_json_at_all")
        errors = validator.validate_set(param_record=record, new_value="anything")
        assert errors == []

    def test_list_set_valid_within_range(self, validator):
        record = self._make_record("list", min_value=1, max_value=5)
        errors = validator.validate_set(param_record=record, new_value=[1, 2])
        assert errors == []


# ── ParameterValidator replace_rules / load_rules tests ──────────────────────

class TestReplaceRules:
    def test_replace_rules_valid(self):
        v = ParameterValidator(rules={})
        v.replace_rules({"create": {}})
        assert v._rules == {"create": {}}

    def test_replace_rules_non_dict_raises(self):
        v = ParameterValidator(rules={})
        with pytest.raises(TypeError):
            v.replace_rules("not a dict")

    def test_load_rules_from_file(self, tmp_path):
        try:
            import yaml
        except ImportError:
            pytest.skip("PyYAML not available")
        rules = {"create": {"name": {"required": True}}}
        f = tmp_path / "rules.yaml"
        f.write_text(yaml.dump(rules))
        v = ParameterValidator(rules={})
        v.load_rules_from_file(str(f))
        assert "create" in v._rules

    def test_load_rules_from_file_without_yaml(self, monkeypatch):
        import vyra_base.core.parameter_validator as mod
        monkeypatch.setattr(mod, "yaml", None)
        v = ParameterValidator(rules={})
        with pytest.raises(ValueError, match="PyYAML"):
            v.load_rules_from_file("/tmp/whatever.yaml")

    def test_load_default_rules_returns_dict(self):
        rules = _load_default_rules()
        assert isinstance(rules, dict)


# ── ParameterValidator with default rules from yaml ──────────────────────────

class TestDefaultRules:
    def test_default_validator_creates(self):
        v = ParameterValidator()
        assert isinstance(v._rules, dict)

    def test_valid_param_with_default_rules(self):
        v = ParameterValidator()
        errors = v.validate_create(
            name="my_param",
            default_value=42,
            type_str="integer",
            description="A test parameter",
        )
        # Should have 0 errors for a valid param
        assert errors == []

    def test_invalid_name_with_default_rules(self):
        v = ParameterValidator()
        errors = v.validate_create(
            name="",
            default_value=42,
            type_str="integer",
            description="desc",
        )
        assert len(errors) > 0
