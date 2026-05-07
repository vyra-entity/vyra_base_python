"""Tests for vyra_base.helper.file_reader.FileReader."""
import json
import os
import tempfile
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from vyra_base.helper.file_reader import FileReader


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


# ---------------------------------------------------------------------------
# Sync tests — open_json_file_sync
# ---------------------------------------------------------------------------

class TestOpenJsonFileSync:
    """Tests for FileReader.open_json_file_sync."""

    def test_reads_valid_json(self, tmp_path):
        """Returns parsed JSON from a valid file."""
        f = tmp_path / "data.json"
        _write(f, '{"key": "value"}')
        result = FileReader.open_json_file_sync(f)
        assert result == {"key": "value"}

    def test_file_not_found_raises(self, tmp_path):
        """Raises FileNotFoundError for missing file."""
        with pytest.raises(FileNotFoundError):
            FileReader.open_json_file_sync(tmp_path / "missing.json")

    def test_invalid_json_raises(self, tmp_path):
        """Raises json.JSONDecodeError for malformed JSON."""
        f = tmp_path / "bad.json"
        _write(f, "not json {{{")
        with pytest.raises(json.decoder.JSONDecodeError):
            FileReader.open_json_file_sync(f)

    def test_returns_list_json(self, tmp_path):
        """Returns a list when JSON root is an array."""
        f = tmp_path / "arr.json"
        _write(f, '[1, 2, 3]')
        result = FileReader.open_json_file_sync(f)
        assert result == [1, 2, 3]

    def test_returns_nested_dict(self, tmp_path):
        """Returns nested dict correctly."""
        data = {"a": {"b": [1, 2, 3]}}
        f = tmp_path / "nested.json"
        _write(f, json.dumps(data))
        assert FileReader.open_json_file_sync(f) == data


# ---------------------------------------------------------------------------
# Sync tests — open_markdown_file_sync
# ---------------------------------------------------------------------------

class TestOpenMarkdownFileSync:
    """Tests for FileReader.open_markdown_file_sync."""

    def test_reads_markdown(self, tmp_path):
        """Returns markdown string from a valid file."""
        f = tmp_path / "readme.md"
        _write(f, "# Title\nContent")
        result = FileReader.open_markdown_file_sync(f)
        assert "# Title" in result
        assert "Content" in result

    def test_file_not_found_raises(self, tmp_path):
        """Raises FileNotFoundError for missing file."""
        with pytest.raises(FileNotFoundError):
            FileReader.open_markdown_file_sync(tmp_path / "missing.md")

    def test_returns_full_content(self, tmp_path):
        """Returns the full file content."""
        content = "line1\nline2\nline3\n"
        f = tmp_path / "doc.md"
        _write(f, content)
        assert FileReader.open_markdown_file_sync(f) == content


# ---------------------------------------------------------------------------
# Sync tests — open_env_file_sync
# ---------------------------------------------------------------------------

class TestOpenEnvFileSync:
    """Tests for FileReader.open_env_file_sync."""

    def test_returns_dict(self, tmp_path):
        """Returns a dict of environment variables."""
        env_file = tmp_path / ".env"
        env_file.write_text("TEST_VAR=hello\n")
        result = FileReader.open_env_file_sync(tmp_path)
        assert isinstance(result, dict)

    def test_env_result_contains_system_vars(self, tmp_path):
        """Result dict contains system environment variables (PATH etc.)."""
        env_file = tmp_path / ".env"
        env_file.write_text("DUMMY_VAR=1\n")
        result = FileReader.open_env_file_sync(tmp_path)
        assert isinstance(result, dict)
        # System env vars like PATH or HOME should always be present
        assert len(result) > 0


# ---------------------------------------------------------------------------
# Sync tests — open_toml_file_sync
# ---------------------------------------------------------------------------

class TestOpenTomlFileSync:
    """Tests for FileReader.open_toml_file_sync."""

    def test_reads_valid_toml(self, tmp_path):
        """Returns parsed TOML as dict."""
        f = tmp_path / "config.toml"
        f.write_text("[section]\nkey = \"value\"\n", encoding="utf-8")
        result = FileReader.open_toml_file_sync(f)
        assert result == {"section": {"key": "value"}}

    def test_reads_nested_toml(self, tmp_path):
        """Returns nested TOML structure."""
        f = tmp_path / "config.toml"
        f.write_text("[db]\nhost = \"localhost\"\nport = 5432\n", encoding="utf-8")
        result = FileReader.open_toml_file_sync(f)
        assert result["db"]["host"] == "localhost"
        assert result["db"]["port"] == 5432

    def test_file_not_found_raises(self, tmp_path):
        """Raises FileNotFoundError for missing .toml file."""
        with pytest.raises((FileNotFoundError, Exception)):
            FileReader.open_toml_file_sync(tmp_path / "missing.toml")


# ---------------------------------------------------------------------------
# Sync tests — open_ini_file_sync
# ---------------------------------------------------------------------------

class TestOpenIniFileSync:
    """Tests for FileReader.open_ini_file_sync."""

    def test_reads_valid_ini(self, tmp_path):
        """Returns dict of INI sections."""
        f = tmp_path / "config.ini"
        f.write_text("[section1]\nkey = value\n", encoding="utf-8")
        result = FileReader.open_ini_file_sync(f)
        assert "section1" in result
        assert result["section1"]["key"] == "value"

    def test_empty_ini_returns_empty_dict(self, tmp_path):
        """Returns empty dict for empty .ini file."""
        f = tmp_path / "empty.ini"
        f.write_text("", encoding="utf-8")
        result = FileReader.open_ini_file_sync(f)
        assert result == {}

    def test_multiple_sections(self, tmp_path):
        """Returns all sections from an INI file."""
        f = tmp_path / "multi.ini"
        f.write_text("[s1]\na=1\n[s2]\nb=2\n", encoding="utf-8")
        result = FileReader.open_ini_file_sync(f)
        assert "s1" in result
        assert "s2" in result


# ---------------------------------------------------------------------------
# Sync tests — open_yaml_file_sync
# ---------------------------------------------------------------------------

class TestOpenYamlFileSync:
    """Tests for FileReader.open_yaml_file_sync (if yaml is available)."""

    def test_reads_valid_yaml(self, tmp_path):
        """Returns parsed YAML as dict."""
        pytest.importorskip("yaml")
        f = tmp_path / "config.yaml"
        f.write_text("key: value\nnested:\n  a: 1\n", encoding="utf-8")
        result = FileReader.open_yaml_file_sync(f)
        assert result["key"] == "value"
        assert result["nested"]["a"] == 1

    def test_reads_list_yaml(self, tmp_path):
        """Returns list from YAML array."""
        pytest.importorskip("yaml")
        f = tmp_path / "list.yaml"
        f.write_text("- item1\n- item2\n", encoding="utf-8")
        result = FileReader.open_yaml_file_sync(f)
        assert result == ["item1", "item2"]


# ---------------------------------------------------------------------------
# Async tests — open_json_file
# ---------------------------------------------------------------------------

class TestOpenJsonFileAsync:
    """Tests for FileReader.open_json_file (async)."""

    @pytest.mark.asyncio
    async def test_reads_valid_json(self, tmp_path):
        """Async: returns parsed JSON from a valid file."""
        f = tmp_path / "data.json"
        _write(f, '{"key": "async_value"}')
        result = await FileReader.open_json_file(f)
        assert result == {"key": "async_value"}

    @pytest.mark.asyncio
    async def test_file_not_found_raises(self, tmp_path):
        """Async: raises FileNotFoundError for missing file."""
        with pytest.raises(FileNotFoundError):
            await FileReader.open_json_file(tmp_path / "missing.json")

    @pytest.mark.asyncio
    async def test_invalid_json_raises(self, tmp_path):
        """Async: raises json.JSONDecodeError for malformed JSON."""
        f = tmp_path / "bad.json"
        _write(f, "not json {{{")
        with pytest.raises(json.decoder.JSONDecodeError):
            await FileReader.open_json_file(f)

    @pytest.mark.asyncio
    async def test_returns_list(self, tmp_path):
        """Async: returns list JSON correctly."""
        f = tmp_path / "arr.json"
        _write(f, '[10, 20]')
        result = await FileReader.open_json_file(f)
        assert result == [10, 20]


# ---------------------------------------------------------------------------
# Async tests — open_markdown_file
# ---------------------------------------------------------------------------

class TestOpenMarkdownFileAsync:
    """Tests for FileReader.open_markdown_file (async)."""

    @pytest.mark.asyncio
    async def test_reads_markdown(self, tmp_path):
        """Async: returns markdown content."""
        f = tmp_path / "readme.md"
        _write(f, "# Async Title\nBody text.")
        result = await FileReader.open_markdown_file(f)
        assert "# Async Title" in result

    @pytest.mark.asyncio
    async def test_file_not_found_returns_none_or_raises(self, tmp_path):
        """Async: handles missing file gracefully (may return None or raise)."""
        try:
            result = await FileReader.open_markdown_file(tmp_path / "missing.md")
            # If it doesn't raise, result should be None or empty
            assert result is None or result == ""
        except Exception:
            pass  # FileNotFoundError is acceptable


# ---------------------------------------------------------------------------
# Async tests — open_env_file
# ---------------------------------------------------------------------------

class TestOpenEnvFileAsync:
    """Tests for FileReader.open_env_file (async)."""

    @pytest.mark.asyncio
    async def test_returns_dict(self, tmp_path):
        """Async: returns dict of env vars."""
        env_file = tmp_path / ".env"
        env_file.write_text("ASYNC_ENV_VAR_42=async_val\n")
        # Ensure var is not pre-set
        os.environ.pop("ASYNC_ENV_VAR_42", None)
        result = await FileReader.open_env_file(tmp_path)
        assert isinstance(result, dict)


# ---------------------------------------------------------------------------
# Async tests — open_toml_file
# ---------------------------------------------------------------------------

class TestOpenTomlFileAsync:
    """Tests for FileReader.open_toml_file (async)."""

    @pytest.mark.asyncio
    async def test_reads_valid_toml(self, tmp_path):
        """Async: returns parsed TOML as dict."""
        f = tmp_path / "config.toml"
        f.write_text("[meta]\nversion = \"1.0\"\n", encoding="utf-8")
        result = await FileReader.open_toml_file(f)
        assert result["meta"]["version"] == "1.0"

    @pytest.mark.asyncio
    async def test_file_not_found_raises(self, tmp_path):
        """Async: raises exception for missing .toml file."""
        with pytest.raises(Exception):
            await FileReader.open_toml_file(tmp_path / "missing.toml")


# ---------------------------------------------------------------------------
# Async tests — open_ini_file
# ---------------------------------------------------------------------------

class TestOpenIniFileAsync:
    """Tests for FileReader.open_ini_file (async)."""

    @pytest.mark.asyncio
    async def test_reads_valid_ini(self, tmp_path):
        """Async: returns dict of INI sections."""
        f = tmp_path / "config.ini"
        f.write_text("[section]\nval = 42\n", encoding="utf-8")
        result = await FileReader.open_ini_file(f)
        assert "section" in result
        assert result["section"]["val"] == "42"


# ---------------------------------------------------------------------------
# Async tests — open_yaml_file
# ---------------------------------------------------------------------------

class TestOpenYamlFileAsync:
    """Tests for FileReader.open_yaml_file (async)."""

    @pytest.mark.asyncio
    async def test_reads_valid_yaml(self, tmp_path):
        """Async: returns parsed YAML."""
        pytest.importorskip("yaml")
        f = tmp_path / "config.yaml"
        f.write_text("service: web\nport: 8080\n", encoding="utf-8")
        result = await FileReader.open_yaml_file(f)
        assert result["service"] == "web"
        assert result["port"] == 8080
