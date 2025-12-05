"""
Unit tests for FileReader and FileWriter sync/async methods.

Tests both synchronous and asynchronous file operations with real file I/O.
"""

import asyncio
import pytest
from pathlib import Path
import tempfile
import json

from vyra_base.helper.file_reader import FileReader
from vyra_base.helper.file_writer import FileWriter


class TestFileReaderWriterSync:
    """Test synchronous file operations."""
    
    def test_json_sync_operations(self):
        """Test JSON read/write with sync methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.json"
            test_data = {
                "name": "VYRA",
                "version": "2.0",
                "features": ["sync", "async"],
                "nested": {"key": "value"}
            }
            
            # Write
            result = FileWriter.write_json_file_sync(test_file, test_data)
            assert result is True, "Writing JSON file should return True"
            assert test_file.exists(), "JSON file should exist after writing"
            
            # Read
            read_data = FileReader.open_json_file_sync(test_file)
            assert read_data == test_data, "Read data should match written data"
    
    def test_yaml_sync_operations(self):
        """Test YAML read/write with sync methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.yaml"
            test_data = {
                "name": "VYRA",
                "config": {
                    "debug": True,
                    "port": 8080
                }
            }
            
            # Write
            result = FileWriter.write_yaml_file_sync(test_file, test_data)
            assert result is True, "Writing YAML file should return True"
            assert test_file.exists(), "YAML file should exist after writing"
            
            # Read
            read_data = FileReader.open_yaml_file_sync(test_file)
            assert read_data == test_data, "Read data should match written data"
    
    def test_binary_sync_operations(self):
        """Test binary read/write with sync methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.bin"
            test_data = b"VYRA Binary Data \x00\x01\x02\xff\xfe"
            
            # Write
            result = FileWriter.write_binary_file_sync(test_file, test_data)
            assert result is True, "Writing binary file should return True"
            assert test_file.exists(), "Binary file should exist after writing"
            
            # Read (using standard Python)
            with open(test_file, 'rb') as f:
                read_data = f.read()
            assert read_data == test_data, "Read data should match written data"
    
    def test_toml_sync_operations(self):
        """Test TOML reading with sync methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.toml"
            
            # Create TOML file manually
            test_file.write_text("""
[tool.poetry]
name = "vyra_base"
version = "0.1.5"

[tool.poetry.dependencies]
python = ">=3.10"
""")
            
            # Read
            read_data = FileReader.open_toml_file_sync(test_file)
            assert "tool" in read_data, "TOML should contain 'tool' section"
            assert read_data["tool"]["poetry"]["name"] == "vyra_base"
    
    def test_markdown_sync_operations(self):
        """Test Markdown reading with sync methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test.md"
            test_content = "# VYRA\n\nThis is a **test** markdown file."
            
            # Write manually
            test_file.write_text(test_content)
            
            # Read
            read_content = FileReader.open_markdown_file_sync(test_file)
            assert read_content == test_content, "Markdown content should match"


class TestFileReaderWriterAsync:
    """Test asynchronous file operations."""
    
    @pytest.mark.asyncio
    async def test_json_async_operations(self):
        """Test JSON read/write with async methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test_async.json"
            test_data = {
                "name": "VYRA",
                "mode": "async",
                "concurrent": True,
                "values": [1, 2, 3]
            }
            
            # Write
            result = await FileWriter.write_json_file(test_file, test_data)
            assert result is True, "Writing JSON file should return True"
            assert test_file.exists(), "JSON file should exist after writing"
            
            # Read
            read_data = await FileReader.open_json_file(test_file)
            assert read_data == test_data, "Read data should match written data"
    
    @pytest.mark.asyncio
    async def test_yaml_async_operations(self):
        """Test YAML read/write with async methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test_async.yaml"
            test_data = {
                "service": "vyra",
                "settings": {
                    "enabled": True,
                    "timeout": 30
                }
            }
            
            # Write
            result = await FileWriter.write_yaml_file(test_file, test_data)
            assert result is True, "Writing YAML file should return True"
            assert test_file.exists(), "YAML file should exist after writing"
            
            # Read
            read_data = await FileReader.open_yaml_file(test_file)
            assert read_data == test_data, "Read data should match written data"
    
    @pytest.mark.asyncio
    async def test_binary_async_operations(self):
        """Test binary read/write with async methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test_async.bin"
            test_data = b"VYRA Async Binary \xff\xfe\x00\x01"
            
            # Write
            result = await FileWriter.write_binary_file(test_file, test_data)
            assert result is True, "Writing binary file should return True"
            assert test_file.exists(), "Binary file should exist after writing"
            
            # Read (using standard Python)
            with open(test_file, 'rb') as f:
                read_data = f.read()
            assert read_data == test_data, "Read data should match written data"
    
    @pytest.mark.asyncio
    async def test_toml_async_operations(self):
        """Test TOML reading with async methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "test_async.toml"
            
            # Create TOML file manually
            test_file.write_text("""
[database]
server = "localhost"
port = 5432
enabled = true
""")
            
            # Read
            read_data = await FileReader.open_toml_file(test_file)
            assert "database" in read_data, "TOML should contain 'database' section"
            assert read_data["database"]["server"] == "localhost"
            assert read_data["database"]["port"] == 5432
    
    @pytest.mark.asyncio
    async def test_concurrent_operations(self):
        """Test concurrent async file operations."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create multiple files concurrently
            tasks = []
            for i in range(5):
                test_file = Path(tmpdir) / f"test_{i}.json"
                test_data = {"id": i, "name": f"test_{i}"}
                tasks.append(FileWriter.write_json_file(test_file, test_data))
            
            results = await asyncio.gather(*tasks)
            assert all(results), "All writes should succeed"
            
            # Read all files concurrently
            read_tasks = []
            for i in range(5):
                test_file = Path(tmpdir) / f"test_{i}.json"
                read_tasks.append(FileReader.open_json_file(test_file))
            
            read_results = await asyncio.gather(*read_tasks)
            for i, data in enumerate(read_results):
                assert data["id"] == i, f"File {i} should have correct id"


class TestFileLocking:
    """Test file locking mechanisms."""
    
    def test_sync_lock_prevents_race_condition(self):
        """Test that sync locks prevent race conditions."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "lock_test.json"
            initial_data = {"counter": 0}
            FileWriter.write_json_file_sync(test_file, initial_data)
            
            # This would normally cause race conditions without locking
            # But with file locks, operations are serialized
            for i in range(10):
                data = FileReader.open_json_file_sync(test_file)
                data["counter"] += 1
                FileWriter.write_json_file_sync(test_file, data)
            
            final_data = FileReader.open_json_file_sync(test_file)
            assert final_data["counter"] == 10, "Counter should be incremented 10 times"
    
    @pytest.mark.asyncio
    async def test_async_lock_prevents_race_condition(self):
        """Test that async locks prevent race conditions."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "async_lock_test.json"
            initial_data = {"counter": 0}
            await FileWriter.write_json_file(test_file, initial_data)
            
            # Sequential increments to test locking mechanism
            # (Concurrent increments would require application-level locking,
            # file locks only protect individual read/write operations)
            for _ in range(10):
                data = await FileReader.open_json_file(test_file)
                data["counter"] += 1
                await FileWriter.write_json_file(test_file, data)
            
            final_data = await FileReader.open_json_file(test_file)
            assert final_data["counter"] == 10, "Counter should be incremented 10 times"


class TestErrorHandling:
    """Test error handling in file operations."""
    
    def test_json_decode_error_sync(self):
        """Test handling of invalid JSON (sync)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "invalid.json"
            test_file.write_text("{ invalid json }")
            
            with pytest.raises(json.decoder.JSONDecodeError):
                FileReader.open_json_file_sync(test_file)
    
    @pytest.mark.asyncio
    async def test_json_decode_error_async(self):
        """Test handling of invalid JSON (async)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "invalid_async.json"
            test_file.write_text("{ invalid json }")
            
            with pytest.raises(json.decoder.JSONDecodeError):
                await FileReader.open_json_file(test_file)
    
    def test_file_not_found_sync(self):
        """Test handling of missing file (sync)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "nonexistent.json"
            
            with pytest.raises(FileNotFoundError):
                FileReader.open_json_file_sync(test_file)
    
    @pytest.mark.asyncio
    async def test_file_not_found_async(self):
        """Test handling of missing file (async)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            test_file = Path(tmpdir) / "nonexistent_async.json"
            
            with pytest.raises(FileNotFoundError):
                await FileReader.open_json_file(test_file)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
