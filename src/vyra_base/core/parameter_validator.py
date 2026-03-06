"""
VYRA Parameter Validator
========================
Encapsulates all validation logic for parameter creation and value setting.

Rules are loaded from ``parameter_rules.yaml`` (located in this package) via
``importlib.resources``.  The active ruleset can be replaced at runtime by
calling :meth:`ParameterValidator.replace_rules`.

Usage::

    validator = ParameterValidator()

    # Validate creation
    errors = validator.validate_create(
        name="my_param",
        default_value=42,
        type_str="integer",
        description="Speed in m/s",
        displayname="Speed",
        min_value=0,
        max_value=100,
        range_value=None,
    )
    if errors:
        print(errors[0])

    # Validate a set-value call
    errors = validator.validate_set(param_record, new_value="50")
    if errors:
        print(errors[0])

    # Replace ruleset at runtime
    import yaml
    with open("/path/to/custom_rules.yaml") as f:
        new_rules = yaml.safe_load(f)
    validator.replace_rules(new_rules)
"""

from __future__ import annotations

import importlib.resources
import json
import logging
import re
from typing import Any, Optional

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

# ── Internal helpers ──────────────────────────────────────────────────────────

def _load_default_rules() -> dict:
    """Load the bundled ``parameter_rules.yaml`` via importlib.resources with file fallback."""
    if yaml is None:
        logger.warning("PyYAML not available – parameter validation rules not loaded.")
        return {}
    # Primary: importlib.resources (works for installed wheels)
    try:
        pkg = importlib.resources.files("vyra_base.core")
        resource = pkg.joinpath("parameter_rules.yaml")
        with importlib.resources.as_file(resource) as path:
            with open(path, encoding="utf-8") as fh:
                rules = yaml.safe_load(fh) or {}
                logger.debug("Loaded parameter_rules.yaml via importlib.resources")
                return rules
    except Exception as exc:
        logger.debug("importlib.resources fallthrough: %s", exc)

    # Fallback: resolve relative to this file (works for editable installs / dev mounts)
    import pathlib
    fallback = pathlib.Path(__file__).with_name("parameter_rules.yaml")
    if fallback.exists():
        try:
            with open(fallback, encoding="utf-8") as fh:
                rules = yaml.safe_load(fh) or {}
                logger.debug("Loaded parameter_rules.yaml from filesystem fallback: %s", fallback)
                return rules
        except Exception as exc2:
            logger.warning("Could not load parameter_rules.yaml from fallback: %s", exc2)

    logger.warning("parameter_rules.yaml not found – using empty ruleset.")
    return {}


# ─────────────────────────────────────────────────────────────────────────────

class ParameterValidator:
    """
    Validates parameter creation and value-set operations.

    The validation ruleset is loaded from ``parameter_rules.yaml`` on first
    instantiation.  Call :meth:`replace_rules` to swap it out at runtime
    (e.g. to load a project-specific override).
    """

    def __init__(self, rules: Optional[dict] = None) -> None:
        """
        :param rules: Optional in-memory ruleset dict.  If *None* the bundled
            ``parameter_rules.yaml`` is loaded automatically.
        """
        self._rules: dict = rules if rules is not None else _load_default_rules()

    # ── Public API ────────────────────────────────────────────────────────────

    def replace_rules(self, rules: dict) -> None:
        """
        Replace the active validation ruleset at runtime.

        :param rules: A fully-formed rules dict (same structure as
            ``parameter_rules.yaml``).
        """
        if not isinstance(rules, dict):
            raise TypeError("rules must be a dict.")
        self._rules = rules
        logger.info("ParameterValidator: ruleset replaced.")

    def load_rules_from_file(self, path: str) -> None:
        """
        Load a YAML ruleset from an arbitrary file path and make it active.

        :param path: Absolute or relative path to a YAML file.
        :raises FileNotFoundError: If the file does not exist.
        :raises ValueError: If PyYAML is not installed.
        """
        if yaml is None:
            raise ValueError("PyYAML is required to load rules from a file.")
        with open(path, encoding="utf-8") as fh:
            new_rules = yaml.safe_load(fh) or {}
        self.replace_rules(new_rules)

    def validate_create(
        self,
        *,
        name: str,
        default_value: Any,
        type_str: str,
        description: str,
        displayname: Optional[str] = None,
        min_value: Any = None,
        max_value: Any = None,
        range_value: Optional[dict] = None,
    ) -> list[str]:
        """
        Validate all fields for a *new* parameter.

        :returns: List of error messages.  Empty list means valid.
        """
        errors: list[str] = []
        rules = self._rules.get("create", {})

        # ── name ─────────────────────────────────────────────────────────────
        name_rules = rules.get("name", {})
        if name_rules.get("required", True) and not name:
            errors.append("Parameter name is required.")
            return errors  # can't continue without a name

        pattern = name_rules.get("pattern", r"^[a-zA-Z_][a-zA-Z0-9_]*$")
        if not re.match(pattern, name):
            msg = name_rules.get(
                "message_invalid_chars",
                "Invalid parameter name '{name}'."
            ).strip()
            errors.append(msg.replace("{name}", name))

        max_len = int(name_rules.get("max_length", 64))
        if len(name) > max_len:
            msg = name_rules.get(
                "message_too_long",
                "Parameter name exceeds {max_length} characters."
            ).strip()
            errors.append(msg.replace("{max_length}", str(max_len)))

        # ── description ───────────────────────────────────────────────────────
        desc_rules = rules.get("description", {})
        if desc_rules.get("required", True) and not description:
            errors.append(desc_rules.get("message_required", "description is required.").strip())

        # ── default_value ─────────────────────────────────────────────────────
        dv_rules = rules.get("default_value", {})
        if dv_rules.get("required", True) and default_value is None:
            errors.append(dv_rules.get("message_required", "default_value is required.").strip())
            return errors  # type-dependent checks can't proceed

        # ── type ──────────────────────────────────────────────────────────────
        type_rules = rules.get("types", {})
        allowed_types: list[str] = type_rules.get(
            "allowed", ["integer", "float", "string", "boolean", "list", "dict"]
        )
        if type_str not in allowed_types:
            msg = type_rules.get(
                "message_invalid",
                "Invalid type '{type}'. Must be one of: {allowed_csv}."
            ).strip()
            errors.append(
                msg.replace("{type}", type_str)
                   .replace("{allowed_csv}", ", ".join(sorted(allowed_types)))
            )
            return errors  # further checks require a valid type

        # ── displayname length ────────────────────────────────────────────────
        dn_rules = rules.get("displayname", {})
        dn = displayname if displayname else name
        dn_max = int(dn_rules.get("max_length", 255))
        if len(dn) > dn_max:
            msg = dn_rules.get("message_too_long", "displayname exceeds {max_length} characters.").strip()
            errors.append(msg.replace("{max_length}", str(dn_max)))

        # ── type conversion of default_value ──────────────────────────────────
        converted_default, type_error = _convert_value(default_value, type_str)
        if type_error:
            errors.append(f"Cannot convert default_value to type '{type_str}': {type_error}")
            return errors  # range checks require a converted value

        # ── min / max ─────────────────────────────────────────────────────────
        if min_value is not None or max_value is not None:
            mn, mx, num_error = _parse_min_max(min_value, max_value)
            if num_error:
                errors.append(f"min_value/max_value must be numeric: {num_error}")
            else:
                if type_str in ("integer", "float"):
                    num_rules = rules.get("numeric", {})
                    if mn is not None and converted_default < mn:
                        msg = num_rules.get("message_below_min", "default_value {value} is below min_value {min}.").strip()
                        errors.append(msg.replace("{value}", str(converted_default)).replace("{min}", str(mn)))
                    if mx is not None and converted_default > mx:
                        msg = num_rules.get("message_above_max", "default_value {value} exceeds max_value {max}.").strip()
                        errors.append(msg.replace("{value}", str(converted_default)).replace("{max}", str(mx)))
                elif type_str == "list":
                    list_rules = rules.get("list_length", {})
                    list_len = len(converted_default)
                    if mn is not None and list_len < int(mn):
                        msg = list_rules.get("message_below_min", "List length {length} is below min_value {min}.").strip()
                        errors.append(msg.replace("{length}", str(list_len)).replace("{min}", str(int(mn))))
                    if mx is not None and list_len > int(mx):
                        msg = list_rules.get("message_above_max", "List length {length} exceeds max_value {max}.").strip()
                        errors.append(msg.replace("{length}", str(list_len)).replace("{max}", str(int(mx))))

        # ── range_value structure ─────────────────────────────────────────────
        if range_value is not None:
            if isinstance(range_value, str):
                try:
                    range_value = json.loads(range_value)
                except json.JSONDecodeError as exc:
                    errors.append(f"range_value is not valid JSON: {exc}")
                    return errors
            rv_rules = rules.get("range_value", {})
            allowed_keys: set = set(rv_rules.get("allowed_keys", ["allowed_values", "allowed_types"]))
            unknown = set(range_value.keys()) - allowed_keys
            if unknown:
                msg = rv_rules.get(
                    "message_unknown_keys",
                    "range_value contains unknown keys: {unknown_keys}."
                ).strip()
                errors.append(msg.replace("{unknown_keys}", str(unknown)))

        return errors

    def validate_set(
        self,
        *,
        param_record: Any,
        new_value: Any,
    ) -> list[str]:
        """
        Validate a value being written to an *existing* parameter.

        :param param_record: The ORM/dict record with fields
            ``type``, ``min_value``, ``max_value``, ``range_value``.
        :param new_value: The value to validate (may be a string from the wire).
        :returns: List of error messages.  Empty list means valid.
        """
        errors: list[str] = []
        rules = self._rules.get("set", {})

        # Normalise record to dict-like access
        def _get(field: str, default: Any = None) -> Any:
            if isinstance(param_record, dict):
                return param_record.get(field, default)
            type_val = getattr(param_record, field, default)
            # SQLAlchemy enums return enum objects
            if hasattr(type_val, "value"):
                return type_val.value
            return type_val

        type_str: str = str(_get("type", "string") or "string")
        min_raw = _get("min_value")
        max_raw = _get("max_value")
        range_raw = _get("range_value")

        # ── Type compatibility ────────────────────────────────────────────────
        type_check = rules.get("type_check", {})
        if type_check.get("enabled", True):
            converted, type_error = _convert_value(new_value, type_str)
            if type_error:
                msg = type_check.get(
                    "message_wrong_type",
                    "Value '{value}' cannot be converted to type '{expected_type}': {error}"
                ).strip()
                errors.append(
                    msg.replace("{value}", str(new_value))
                       .replace("{expected_type}", type_str)
                       .replace("{error}", str(type_error))
                )
                return errors  # can't do range checks without a converted value
        else:
            converted, _ = _convert_value(new_value, type_str)

        # ── min / max ─────────────────────────────────────────────────────────
        mm_rules = rules.get("min_max", {})
        if mm_rules.get("enabled", True) and (min_raw is not None or max_raw is not None):
            mn, mx, num_error = _parse_min_max(min_raw, max_raw)
            if num_error:
                msg = mm_rules.get("message_non_numeric", "min_value/max_value must be numeric: {error}").strip()
                errors.append(msg.replace("{error}", str(num_error)))
            else:
                if type_str in ("integer", "float"):
                    try:
                        cv = float(converted)  # type: ignore[arg-type]
                        if mn is not None and cv < mn:
                            msg = mm_rules.get("message_below_min", "Value {value} is below min_value {min}.").strip()
                            errors.append(msg.replace("{value}", str(cv)).replace("{min}", str(mn)))
                        if mx is not None and cv > mx:
                            msg = mm_rules.get("message_above_max", "Value {value} exceeds max_value {max}.").strip()
                            errors.append(msg.replace("{value}", str(cv)).replace("{max}", str(mx)))
                    except (TypeError, ValueError):
                        pass
                elif type_str == "list" and isinstance(converted, list):
                    list_len = len(converted)
                    if mn is not None and list_len < int(mn):
                        msg = mm_rules.get("message_list_below_min", "List length {length} is below min_value {min}.").strip()
                        errors.append(msg.replace("{length}", str(list_len)).replace("{min}", str(int(mn))))
                    if mx is not None and list_len > int(mx):
                        msg = mm_rules.get("message_list_above_max", "List length {length} exceeds max_value {max}.").strip()
                        errors.append(msg.replace("{length}", str(list_len)).replace("{max}", str(int(mx))))

        # ── range_value: allowed_values check ────────────────────────────────
        rv_rules = rules.get("range_value", {})
        if rv_rules.get("enabled", True) and range_raw:
            range_dict: dict = {}
            if isinstance(range_raw, str):
                try:
                    range_dict = json.loads(range_raw)
                except (json.JSONDecodeError, ValueError):
                    range_dict = {}
            elif isinstance(range_raw, dict):
                range_dict = range_raw

            allowed_values = range_dict.get("allowed_values")
            if allowed_values and isinstance(allowed_values, list):
                # Compare converted value (or stringify it for list/dict types)
                val_repr = converted if type_str in ("string",) else converted
                if val_repr not in allowed_values:
                    msg = rv_rules.get(
                        "message_not_in_allowed",
                        "Value '{value}' is not in allowed_values {allowed}."
                    ).strip()
                    errors.append(
                        msg.replace("{value}", str(val_repr))
                           .replace("{allowed}", str(allowed_values))
                    )

        return errors


# ── Private helpers ───────────────────────────────────────────────────────────

def _convert_value(value: Any, type_str: str) -> tuple[Any, Optional[str]]:
    """
    Attempt to convert *value* to the Python type dictated by *type_str*.

    :returns: ``(converted, error_message_or_None)``
    """
    try:
        if type_str in ("integer", "int"):
            return int(value), None
        if type_str in ("float",):
            return float(value), None
        if type_str in ("boolean", "bool"):
            if isinstance(value, bool):
                return value, None
            if isinstance(value, str):
                return value.lower() in ("true", "1", "yes"), None
            return bool(value), None
        if type_str in ("list",):
            if isinstance(value, list):
                return value, None
            parsed = json.loads(value) if isinstance(value, str) else value
            if not isinstance(parsed, list):
                raise ValueError("Expected a list")
            return parsed, None
        if type_str in ("dict",):
            if isinstance(value, dict):
                return value, None
            parsed = json.loads(value) if isinstance(value, str) else value
            if not isinstance(parsed, dict):
                raise ValueError("Expected a dict")
            return parsed, None
        # default: string
        return str(value), None
    except (ValueError, TypeError, json.JSONDecodeError) as exc:
        return None, str(exc)


def _parse_min_max(
    min_raw: Any, max_raw: Any
) -> tuple[Optional[float], Optional[float], Optional[str]]:
    """Parse min/max to floats.  Returns (mn, mx, error_str_or_None)."""
    try:
        mn = float(min_raw) if min_raw not in (None, "") else None
        mx = float(max_raw) if max_raw not in (None, "") else None
        return mn, mx, None
    except (TypeError, ValueError) as exc:
        return None, None, str(exc)
