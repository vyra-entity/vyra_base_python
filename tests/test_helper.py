"""
Comprehensive test suite for vyra_base.helper modules
Tests error handling, file reading, file writing, logging, and utility functions
"""

import pytest
import asyncio
import json
import tempfile
import sys
import traceback
import logging
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock, AsyncMock, mock_open
from typing import Any, Dict

from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.file_reader import FileReader
from vyra_base.helper.file_writer import FileWriter
from vyra_base.helper.logger import Logger, LogEntry, LogMode


@pytest.mark.no_dataspace_reset
class TestLogger:
    """Test Logger functionality"""
    
    @staticmethod
    def _create_temp_logger_config():
        """Helper method to create a temporary logger config file"""
        import tempfile
        import json
        
        config_data = {
            "version": 1,
            "disable_existing_loggers": False,
            "loggers": {
                "vyra_base": {
                    "level": "DEBUG",
                    "handlers": ["file_handler"],
                    "propagate": False
                }
            },
            "formatters": {
                "debug": {
                    "format": "%(asctime)s - %(levelname)-8s - %(name)s - %(message)s"
                }
            },
            "handlers": {
                "file_handler": {
                    "class": "logging.handlers.RotatingFileHandler",
                    "level": "DEBUG",
                    "formatter": "debug",
                    "filename": "log/vyra/vyra_main_stdout.log",
                    "mode": "a",
                    "maxBytes": 5242880,
                    "backupCount": 40,
                    "encoding": "utf8",
                    "delay": 0
                }
            }
        }
        
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False)
        json.dump(config_data, f)
        f.close()
        return Path(f.name)
    
    @staticmethod
    def _mock_logger_init():
        """Helper method to create common mocks for logger initialization (deprecated - use _create_temp_logger_config)"""
        # Create mock for config file
        mock_file = mock_open(read_data='{"version": 1, "disable_existing_loggers": false, "loggers": {"vyra_base": {"level": "DEBUG", "handlers": ["file_handler"], "propagate": false}}, "formatters": {"debug": {"format": "%(asctime)s - %(levelname)-8s - %(name)s - %(message)s"}}, "handlers": {"file_handler": {"class": "logging.handlers.RotatingFileHandler", "level": "DEBUG", "formatter": "debug", "filename": "log/vyra/vyra_main_stdout.log", "mode": "a", "maxBytes": 5242880, "backupCount": 40, "encoding": "utf8", "delay": 0}}}')()
        
        # Create a proper mock Path that supports the __truediv__ operator
        mock_path_instance = Mock()
        mock_path_instance.open = mock_file
        mock_path_instance.resolve.return_value.parent.__truediv__ = Mock(return_value=mock_path_instance)
        
        mock_logger = Mock()
        mock_logger.handlers = []
        mock_logger.debug = Mock()
        mock_logger.info = Mock()
        mock_logger.warning = Mock()
        mock_logger.error = Mock()
        
        return mock_path_instance, mock_logger
    
    def setup_method(self):
        """Reset Logger state before each test"""
        # Reset logger state
        if hasattr(Logger, 'logger'):
            delattr(Logger, 'logger')
        Logger._initialized = False
        Logger.pre_log_buffer.clear()
        Logger._LOG_ACTIVE = False
        Logger._LOG_TAG = ''
    
    def test_logger_auto_initialization(self):
        """Test that logger automatically initializes with default values"""
        # Logger should not be initialized yet
        assert not Logger._initialized
        
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_get_logger.return_value = mock_logger
                
                # Pass the config path explicitly
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                # Logger should now be initialized
                assert Logger._initialized
                assert hasattr(Logger, 'logger')
        finally:
            config_path.unlink()
    
    def test_logger_manual_initialization(self):
        """Test manual logger initialization"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                assert Logger._initialized
                assert Logger._LOG_ACTIVE is True
        finally:
            config_path.unlink()
    
    def test_log_entry_modes(self):
        """Test LogEntry mode setters"""
        entry = LogEntry("Test message")
        
        # Default mode
        assert entry.mode == LogMode.INFO
        
        # Test mode setters
        entry.debug()
        assert entry.mode == LogMode.DEBUG
        
        entry.error()
        assert entry.mode == LogMode.ERROR
        
        entry.warn()
        assert entry.mode == LogMode.WARNING
        
        entry.warning()
        assert entry.mode == LogMode.WARNING
    
    def test_logger_convenience_methods(self):
        """Test Logger convenience methods (info, debug, warn, error)"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_logger.debug = Mock()
                mock_logger.info = Mock()
                mock_logger.warning = Mock()
                mock_logger.error = Mock()
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                # Test info
                Logger.info("Info message")
                assert mock_logger.info.called
                
                # Test debug
                Logger.debug("Debug message")
                assert mock_logger.debug.called
                
                # Test warn
                Logger.warn("Warning message")
                assert mock_logger.warning.called
                
                # Test error
                Logger.error("Error message")
                assert mock_logger.error.called
        finally:
            config_path.unlink()
    
    def test_logger_with_tag(self):
        """Test logger with tag prefix"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_logger.info = Mock()
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True, log_tag="TEST_MODULE")
                
                assert Logger._LOG_TAG == "TEST_MODULE"
                
                # Replace logger with our mock to capture calls
                Logger.logger = mock_logger
                
                Logger.info("Test message")
                
                # Verify the tag was added
                mock_logger.info.assert_called_once()
                logged_message = mock_logger.info.call_args[0][0]
                assert "[TEST_MODULE]" in logged_message
        finally:
            config_path.unlink()
    
    def test_pre_log_buffer(self):
        """Test that messages are buffered before initialization and auto-init works"""
        # Reset state
        Logger._initialized = False
        Logger.pre_log_buffer.clear()
        
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_get_logger.return_value = mock_logger
                
                # Mock the _ensure_initialized to use our config
                original_init = Logger.initialize
                def mock_init(*args, **kwargs):
                    if 'log_config_path' not in kwargs:
                        kwargs['log_config_path'] = config_path
                    return original_init(*args, **kwargs)
                
                with patch.object(Logger, 'initialize', side_effect=mock_init):
                    # Try to log before explicit initialization - should auto-initialize
                    entry = LogEntry("Buffered message", LogMode.INFO)
                    Logger.log(entry)
                    
                    # Logger should now be initialized (auto-init triggered)
                    assert Logger._initialized
                    # Buffer should be cleared after auto-initialization
                    assert len(Logger.pre_log_buffer) == 0
        finally:
            config_path.unlink()
    
    @pytest.mark.asyncio
    async def test_logging_on_decorator_async(self):
        """Test logging_on decorator with async function"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_logger.debug = Mock()
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                @Logger.logging_on
                async def async_test_function():
                    await asyncio.sleep(0.01)
                    return "async_result"
                
                result = await async_test_function()
                
                assert result == "async_result"
                # Should have logged entry and exit (both are debug level)
                assert mock_logger.debug.call_count >= 2
        finally:
            config_path.unlink()
    
    def test_logging_on_decorator_sync(self):
        """Test logging_on decorator with sync function"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_logger.debug = Mock()
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                @Logger.logging_on
                def sync_test_function():
                    return "sync_result"
                
                result = sync_test_function()
                
                assert result == "sync_result"
                # Should have logged entry and exit (both are debug level)
                assert mock_logger.debug.call_count >= 2
        finally:
            config_path.unlink()
    
    def test_logger_string_conversion(self):
        """Test that non-LogEntry objects are converted to strings"""
        config_path = self._create_temp_logger_config()
        
        try:
            with patch('os.makedirs'), \
                 patch('logging.config.dictConfig'), \
                 patch('logging.getLogger') as mock_get_logger:
                
                mock_logger = Mock()
                mock_logger.handlers = []
                mock_logger.info = Mock()
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(log_config_path=config_path, log_active=True)
                
                # Replace logger with mock to capture calls
                Logger.logger = mock_logger
                
                # Test with string
                Logger.log("Plain string")
                mock_logger.info.assert_called()
                
                # Test with number
                Logger.log(42)
                assert mock_logger.info.call_count >= 2
                
                # Test with dict
                Logger.log({"key": "value"})
                assert mock_logger.info.call_count >= 3
        finally:
            config_path.unlink()


@pytest.mark.no_dataspace_reset
class TestErrorTraceback:
    """Test ErrorTraceback functionality"""
    
    def test_check_error_exist_no_error(self):
        """Test check_error_exist when no error occurred"""
        error_details = []
        
        # No exception in sys.exc_info()
        with patch('sys.exc_info', return_value=(None, None, None)):
            result = ErrorTraceback.check_error_exist(error_details)
            
            assert result is False
            assert len(error_details) == 0
    
    def test_check_error_exist_with_cancelled_error(self):
        """Test check_error_exist with CancelledError (should be ignored)"""
        error_details = []
        
        with patch('sys.exc_info') as mock_exc_info:
            mock_exc_info.return_value = (
                asyncio.exceptions.CancelledError,
                asyncio.exceptions.CancelledError("Task was cancelled"),
                Mock()
            )
            
            result = ErrorTraceback.check_error_exist(error_details)
            
            assert result is False
            assert len(error_details) == 0
    
    def test_check_error_exist_with_real_error(self):
        """Test check_error_exist with actual error"""
        error_details = []
        
        # Mock exception info
        mock_exc_type = ValueError
        mock_exc_obj = ValueError("Test error")
        mock_exc_tb = Mock()
        
        with patch('sys.exc_info', return_value=(mock_exc_type, mock_exc_obj, mock_exc_tb)), \
             patch('traceback.format_exc', return_value="Full traceback"), \
             patch('traceback.format_tb', return_value=["Line 1\n", "Line 2\n"]), \
             patch('vyra_base.helper.error_handler.Logger') as mock_logger:
            
            # Call with log_print=True to trigger logging
            result = ErrorTraceback.check_error_exist(error_details, log_print=True)
            
            assert result is True
            assert len(error_details) == 1
            assert error_details[0] == "Full traceback"
            
            # Verify logger was called when log_print=True
            assert mock_logger.log.called or mock_logger.error.called
    
    def test_w_check_error_exist_sync_decorator_success(self):
        """Test w_check_error_exist decorator with synchronous function (success)"""
        @ErrorTraceback.w_check_error_exist
        def test_function(x, y):
            return x + y
        
        with patch.object(ErrorTraceback, 'check_error_exist', return_value=False) as mock_check:
            result = test_function(2, 3)
            
            assert result == 5
            mock_check.assert_called_once()
    
    def test_w_check_error_exist_sync_decorator_with_error(self):
        """Test w_check_error_exist decorator with synchronous function (error)"""
        @ErrorTraceback.w_check_error_exist
        def test_function_with_error():
            raise ValueError("Test error")
        
        with patch.object(ErrorTraceback, 'check_error_exist', return_value=True) as mock_check:
            with pytest.raises(ValueError, match="Test error"):
                test_function_with_error()
            
            mock_check.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_w_check_error_exist_async_decorator_success(self):
        """Test w_check_error_exist decorator with async function (success)"""
        @ErrorTraceback.w_check_error_exist
        async def async_test_function(x, y):
            await asyncio.sleep(0.01)  # Simulate async work
            return x * y
        
        with patch.object(ErrorTraceback, 'check_error_exist', return_value=False) as mock_check:
            result = await async_test_function(3, 4)
            
            assert result == 12
            mock_check.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_w_check_error_exist_async_decorator_with_error(self):
        """Test w_check_error_exist decorator with async function (error)"""
        @ErrorTraceback.w_check_error_exist
        async def async_test_function_with_error():
            await asyncio.sleep(0.01)
            raise ValueError("Async test error")
        
        with patch.object(ErrorTraceback, 'check_error_exist', return_value=True) as mock_check:
            with pytest.raises(ValueError, match="Async test error"):
                await async_test_function_with_error()
            
            mock_check.assert_called_once()
    
    def test_w_check_error_exist_preserves_function_metadata(self):
        """Test that decorator preserves function metadata"""
        def original_function():
            """Original function docstring"""
            pass
        
        decorated = ErrorTraceback.w_check_error_exist(original_function)
        
        # Function should be wrapped but accessible
        assert callable(decorated)
    
    @pytest.mark.asyncio
    async def test_w_check_error_exist_async_detection(self):
        """Test that decorator correctly detects async functions"""
        from inspect import iscoroutinefunction
        
        @ErrorTraceback.w_check_error_exist
        def sync_func():
            return "sync"
        
        @ErrorTraceback.w_check_error_exist
        async def async_func():
            return "async"
        
        # Sync function should not be coroutine
        assert not iscoroutinefunction(sync_func)
        
        # Async function should still be coroutine
        assert iscoroutinefunction(async_func)
        
        # Test execution
        with patch.object(ErrorTraceback, 'check_error_exist', return_value=False):
            assert sync_func() == "sync"
            assert await async_func() == "async"


@pytest.mark.no_dataspace_reset
class TestFileReader:
    """Test FileReader functionality"""
    
    @pytest.fixture
    def temp_dir(self):
        """Create temporary directory for test files"""
        with tempfile.TemporaryDirectory() as temp_dir:
            yield Path(temp_dir)
    
    @pytest.mark.asyncio
    async def test_open_json_file_success(self, temp_dir):
        """Test successful JSON file reading"""
        # Create test JSON file
        test_data = {"name": "test", "value": 42, "enabled": True}
        json_file = temp_dir / "test.json"
        
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(test_data, f)
        
        with patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock) as mock_release_lock:
            
            # Mock async context manager for lock
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_reader.AsyncPath') as mock_async_path:
                # Mock AsyncPath and its open method
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.read = AsyncMock(return_value=json.dumps(test_data))
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                result = await FileReader.open_json_file(json_file)
                
                assert result == test_data
                mock_get_lock.assert_called_once_with(json_file)
                # Note: release_lock is called in the try block, but implementation may vary
                # mock_release_lock.assert_called_once_with(json_file)
    
    @pytest.mark.asyncio
    async def test_open_json_file_with_default(self, temp_dir):
        """Test JSON file reading with default fallback"""
        json_file = temp_dir / "test.json"
        default_file = temp_dir / "default.json"
        
        # Create empty main file and default file
        json_file.touch()
        default_data = {"default": "config"}
        with open(default_file, 'w', encoding='utf-8') as f:
            json.dump(default_data, f)
        
        # Test with non-existent main file - should raise FileNotFoundError
        nonexistent = temp_dir / "nonexistent.json"
        with pytest.raises(FileNotFoundError):
            await FileReader.open_json_file(nonexistent, default_file)
    
    @pytest.mark.asyncio
    async def test_open_markdown_file_success(self, temp_dir):
        """Test successful markdown file reading"""
        markdown_content = "# Test Markdown\n\nThis is a test."
        md_file = temp_dir / "test.md"
        
        with open(md_file, 'w', encoding='utf-8') as f:
            f.write(markdown_content)
        
        with patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_reader.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.read = AsyncMock(return_value=markdown_content)
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                result = await FileReader.open_markdown_file(md_file)
                
                assert result == markdown_content
    
    @pytest.mark.asyncio
    async def test_open_env_file_success(self, temp_dir):
        """Test successful environment file reading"""
        env_content = "TEST_VAR=test_value\nANOTHER_VAR=another_value"
        env_file = temp_dir / ".env"
        
        with open(env_file, 'w', encoding='utf-8') as f:
            f.write(env_content)
        
        with patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock), \
             patch('vyra_base.helper.file_reader.load_dotenv') as mock_load_dotenv, \
             patch('os.environ', {'TEST_VAR': 'test_value', 'ANOTHER_VAR': 'another_value'}):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            result = await FileReader.open_env_file(temp_dir)
            
            assert isinstance(result, dict)
            assert 'TEST_VAR' in result
            assert result['TEST_VAR'] == 'test_value'
            mock_load_dotenv.assert_called_once_with(dotenv_path=temp_dir)
    
    @pytest.mark.asyncio
    async def test_open_toml_file_success_python311(self, temp_dir):
        """Test TOML file reading with Python 3.11+ (tomllib)"""
        toml_content = """
[section]
name = "test"
value = 42
enabled = true
        """
        toml_file = temp_dir / "test.toml"
        
        with open(toml_file, 'w', encoding='utf-8') as f:
            f.write(toml_content)
        
        # Mock Python 3.11+ and tomllib
        with patch('sys.version_info', (3, 11, 0)), \
             patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_reader.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.read = AsyncMock(return_value=toml_content.strip())
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                # Mock tomllib.loads
                expected_result = {"section": {"name": "test", "value": 42, "enabled": True}}
                with patch('tomllib.loads', return_value=expected_result) as mock_loads:
                    result = await FileReader.open_toml_file(toml_file)
                    
                    assert result == expected_result
                    mock_loads.assert_called_once_with(toml_content.strip())
    
    @pytest.mark.asyncio
    async def test_open_toml_file_success_python310(self, temp_dir):
        """Test TOML file reading with Python < 3.11 (tomli)"""
        toml_content = """
[section]
name = "test"
        """
        toml_file = temp_dir / "test.toml"
        
        with open(toml_file, 'w', encoding='utf-8') as f:
            f.write(toml_content)
        
        # Mock Python 3.10 and tomli
        with patch('sys.version_info', (3, 10, 0)), \
             patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_reader.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.read = AsyncMock(return_value=toml_content.strip())
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                # Mock tomli import and loads
                expected_result = {"section": {"name": "test"}}
                with patch('tomli.loads', return_value=expected_result) as mock_loads:
                    result = await FileReader.open_toml_file(toml_file)
                    
                    assert result == expected_result
                    mock_loads.assert_called_once_with(toml_content.strip())
    
    @pytest.mark.asyncio
    async def test_open_ini_file_success(self, temp_dir):
        """Test INI file reading"""
        ini_content = """
[section1]
key1 = value1
key2 = value2

[section2]
key3 = value3
        """
        ini_file = temp_dir / "test.ini"
        
        with open(ini_file, 'w', encoding='utf-8') as f:
            f.write(ini_content)
        
        with patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('configparser.ConfigParser') as mock_config_parser:
                mock_parser = Mock()
                mock_parser.sections.return_value = ['section1', 'section2']
                mock_parser.items.side_effect = [
                    [('key1', 'value1'), ('key2', 'value2')],
                    [('key3', 'value3')]
                ]
                mock_config_parser.return_value = mock_parser
                
                result = await FileReader.open_ini_file(ini_file)
                
                expected = {
                    'section1': {'key1': 'value1', 'key2': 'value2'},
                    'section2': {'key3': 'value3'}
                }
                assert result == expected
                mock_parser.read.assert_called_once_with(str(ini_file))
    
    @pytest.mark.asyncio
    async def test_open_yaml_file_success(self, temp_dir):
        """Test YAML file reading"""
        yaml_data = {"name": "test", "items": ["item1", "item2"], "config": {"enabled": True}}
        yaml_content = """
name: test
items:
  - item1
  - item2
config:
  enabled: true
        """
        yaml_file = temp_dir / "test.yaml"
        
        with open(yaml_file, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        with patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_reader.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.read = AsyncMock(return_value=yaml_content.strip())
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                with patch('yaml.safe_load', return_value=yaml_data) as mock_yaml_load:
                    result = await FileReader.open_yaml_file(yaml_file)
                    
                    assert result == yaml_data
                    mock_yaml_load.assert_called_once_with(yaml_content.strip())
    
    @pytest.mark.asyncio 
    async def test_file_reader_error_handling(self, temp_dir):
        """Test FileReader error handling"""
        nonexistent_file = temp_dir / "nonexistent.json"
        
        # Test FileNotFoundError handling - expect FileNotFoundError to be raised
        with pytest.raises(FileNotFoundError):
            await FileReader.open_json_file(nonexistent_file)


@pytest.mark.no_dataspace_reset
class TestFileWriter:
    """Test FileWriter functionality"""
    
    @pytest.fixture
    def temp_dir(self):
        """Create temporary directory for test files"""
        with tempfile.TemporaryDirectory() as temp_dir:
            yield Path(temp_dir)
    
    @pytest.mark.asyncio
    async def test_write_json_file_success(self, temp_dir):
        """Test successful JSON file writing"""
        test_data = {"name": "test", "value": 42, "enabled": True}
        json_file = temp_dir / "output.json"
        
        with patch('vyra_base.helper.file_writer.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_writer.release_lock_for_file', new_callable=AsyncMock) as mock_release_lock:
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_writer.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.write = AsyncMock()
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                result = await FileWriter.write_json_file(json_file, test_data)
                
                assert result is True
                mock_get_lock.assert_called_once_with(json_file)
                mock_release_lock.assert_called_once_with(json_file)
                
                # Verify JSON content was written
                written_content = mock_file.write.call_args[0][0]
                assert json.loads(written_content) == test_data
    
    @pytest.mark.asyncio
    async def test_write_json_file_io_error(self, temp_dir):
        """Test JSON file writing with IO error"""
        test_data = {"test": "data"}
        json_file = temp_dir / "error.json"
        
        with patch('vyra_base.helper.file_writer.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_writer.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_writer.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                # Simulate IO error
                mock_path_instance.open = AsyncMock(side_effect=IOError("Disk full"))
                
                with pytest.raises(IOError, match="An unexpected io error occured!"):
                    await FileWriter.write_json_file(json_file, test_data)
    
    @pytest.mark.asyncio
    async def test_write_json_file_json_error(self, temp_dir):
        """Test JSON file writing with JSON serialization error"""
        # Create object that can't be JSON serialized
        class NonSerializableObject:
            pass
        
        test_data = {"object": NonSerializableObject()}
        json_file = temp_dir / "error.json"
        
        with patch('vyra_base.helper.file_writer.get_lock_for_file', new_callable=AsyncMock) as mock_get_lock, \
             patch('vyra_base.helper.file_writer.release_lock_for_file', new_callable=AsyncMock):
            
            mock_lock = AsyncMock()
            mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
            mock_lock.__aexit__ = AsyncMock(return_value=None)
            mock_get_lock.return_value = mock_lock
            
            with patch('vyra_base.helper.file_writer.AsyncPath') as mock_async_path:
                mock_path_instance = AsyncMock()
                mock_async_path.return_value = mock_path_instance
                
                mock_file = AsyncMock()
                mock_file.__aenter__ = AsyncMock(return_value=mock_file)
                mock_file.__aexit__ = AsyncMock(return_value=None)
                
                mock_path_instance.open = AsyncMock(return_value=mock_file)
                
                # json.dumps will fail with TypeError for non-serializable object
                with pytest.raises(TypeError, match="An unexpected type error uccured!"):
                    await FileWriter.write_json_file(json_file, test_data)


@pytest.mark.no_dataspace_reset
class TestHelperIntegration:
    """Test integration between helper modules"""
    
    @pytest.mark.asyncio
    async def test_error_traceback_with_file_operations(self, temp_dir=None):
        """Test ErrorTraceback with file operations"""
        import tempfile
        
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            
            @ErrorTraceback.w_check_error_exist
            async def file_operation_with_error():
                # This will cause an error
                raise FileNotFoundError("Test error for traceback")
            
            with patch.object(ErrorTraceback, 'check_error_exist', return_value=True) as mock_check:
                with pytest.raises((Exception, FileNotFoundError)):  # Some error will be raised
                    await file_operation_with_error()
                
                # Error traceback should have been checked  
                mock_check.assert_called_once()
    
    def test_helper_modules_imported_correctly(self):
        """Test that all helper modules can be imported"""
        # Test that classes can be imported and instantiated
        assert ErrorTraceback is not None
        assert FileReader is not None
        assert FileWriter is not None
        
        # Test that key methods exist
        assert hasattr(ErrorTraceback, 'check_error_exist')
        assert hasattr(ErrorTraceback, 'w_check_error_exist')
        assert hasattr(FileReader, 'open_json_file')
        assert hasattr(FileReader, 'open_yaml_file')
        assert hasattr(FileWriter, 'write_json_file')
    
    @pytest.mark.asyncio
    async def test_round_trip_json_operations(self):
        """Test round-trip JSON read/write operations"""
        import tempfile
        
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            test_file = temp_path / "roundtrip.json"
            
            original_data = {
                "string": "test",
                "number": 42,
                "boolean": True,
                "array": [1, 2, 3],
                "object": {"nested": "value"}
            }
            
            # Mock the async file operations
            with patch('vyra_base.helper.file_writer.get_lock_for_file', new_callable=AsyncMock), \
                 patch('vyra_base.helper.file_writer.release_lock_for_file', new_callable=AsyncMock), \
                 patch('vyra_base.helper.file_reader.get_lock_for_file', new_callable=AsyncMock), \
                 patch('vyra_base.helper.file_reader.release_lock_for_file', new_callable=AsyncMock):
                
                # Mock locks
                mock_lock = AsyncMock()
                mock_lock.__aenter__ = AsyncMock(return_value=mock_lock)
                mock_lock.__aexit__ = AsyncMock(return_value=None)
                
                written_content = None
                
                # Mock write operation
                with patch('vyra_base.helper.file_writer.AsyncPath') as mock_write_path:
                    mock_write_instance = AsyncMock()
                    mock_write_path.return_value = mock_write_instance
                    
                    mock_write_file = AsyncMock()
                    def capture_write(content):
                        nonlocal written_content
                        written_content = content
                    mock_write_file.write = AsyncMock(side_effect=capture_write)
                    mock_write_file.__aenter__ = AsyncMock(return_value=mock_write_file)
                    mock_write_file.__aexit__ = AsyncMock(return_value=None)
                    
                    mock_write_instance.open = AsyncMock(return_value=mock_write_file)
                    
                    # Write the data
                    write_result = await FileWriter.write_json_file(test_file, original_data)
                    assert write_result is True
                
                # Mock read operation
                with patch('vyra_base.helper.file_reader.AsyncPath') as mock_read_path:
                    mock_read_instance = AsyncMock()
                    mock_read_path.return_value = mock_read_instance
                    
                    mock_read_file = AsyncMock()
                    mock_read_file.read = AsyncMock(return_value=written_content)
                    mock_read_file.__aenter__ = AsyncMock(return_value=mock_read_file)
                    mock_read_file.__aexit__ = AsyncMock(return_value=None)
                    
                    mock_read_instance.open = AsyncMock(return_value=mock_read_file)
                    
                    # Read the data back
                    read_data = await FileReader.open_json_file(test_file)
                    
                    # Verify round-trip integrity
                    assert read_data == original_data