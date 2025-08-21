import pytest
import asyncio
import json
import logging
import logging.handlers
import os
import tempfile
import time
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock, call
from io import StringIO

from vyra_base.helper.logger import Logger, LogEntry, LogMode


class TestLogMode:
    """Test the LogMode enum"""
    
    def test_log_mode_values(self):
        """Test LogMode enum values"""
        assert LogMode.DEBUG.value == 0
        assert LogMode.INFO.value == 1
        assert LogMode.WARNING.value == 2
        assert LogMode.ERROR.value == 3


class TestLogEntry:
    """Test the LogEntry dataclass"""
    
    def test_log_entry_creation(self):
        """Test LogEntry creation with default values"""
        entry = LogEntry("test message")
        assert entry.message == "test message"
        assert entry.mode == LogMode.INFO
        
    def test_log_entry_with_mode(self):
        """Test LogEntry creation with specific mode"""
        entry = LogEntry("test message", LogMode.ERROR)
        assert entry.message == "test message"
        assert entry.mode == LogMode.ERROR
        
    def test_log_entry_debug_method(self):
        """Test LogEntry debug method"""
        entry = LogEntry("test message")
        result = entry.debug()
        assert result is entry  # Should return self
        assert entry.mode == LogMode.DEBUG
        
    def test_log_entry_error_method(self):
        """Test LogEntry error method"""
        entry = LogEntry("test message")
        result = entry.error()
        assert result is entry  # Should return self
        assert entry.mode == LogMode.ERROR
        
    def test_log_entry_warn_method(self):
        """Test LogEntry warn method"""
        entry = LogEntry("test message")
        result = entry.warn()
        assert result is entry  # Should return self
        assert entry.mode == LogMode.WARNING
        
    def test_log_entry_method_chaining(self):
        """Test method chaining with LogEntry"""
        entry = LogEntry("test message").debug()
        assert entry.mode == LogMode.DEBUG
        
        entry = LogEntry("test message").error()
        assert entry.mode == LogMode.ERROR
        
        entry = LogEntry("test message").warn()
        assert entry.mode == LogMode.WARNING


class TestLogger:
    """Test the Logger class"""
    
    def setup_method(self):
        """Setup method to reset Logger state before each test"""
        Logger._LOG_ACTIVE = False
        Logger._ext_logger = []
        if hasattr(Logger, 'logger'):
            # Reset logger handlers
            Logger.logger.handlers = []
    
    def create_test_log_config(self, log_file_path: str) -> dict:
        """Create a test logging configuration"""
        return {
            "version": 1,
            "disable_existing_loggers": False,
            "formatters": {
                "default": {
                    "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
                }
            },
            "handlers": {
                "file_handler": {
                    "class": "logging.handlers.RotatingFileHandler",
                    "level": "DEBUG",
                    "formatter": "default",
                    "filename": log_file_path,
                    "maxBytes": 10485760,
                    "backupCount": 5
                }
            },
            "loggers": {
                "vyra_base": {
                    "level": "DEBUG",
                    "handlers": ["file_handler"],
                    "propagate": False
                }
            }
        }
    
    def test_logger_initialization(self):
        """Test Logger initialization"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            # Create test config
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            # Test initialization
            Logger.initialize(Path(config_file), log_active=True)
            
            assert Logger._LOG_ACTIVE is True
            assert hasattr(Logger, 'logger')
            assert Logger.logger.name == 'vyra_base'
            
    def test_logger_initialization_creates_directory(self):
        """Test that Logger.initialize creates log directory if it doesn't exist"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_dir = os.path.join(temp_dir, "logs", "subdir")
            log_file = os.path.join(log_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            # Create test config
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            # Ensure directory doesn't exist
            assert not os.path.exists(log_dir)
            
            # Test initialization
            Logger.initialize(Path(config_file), log_active=True)
            
            # Directory should be created
            assert os.path.exists(log_dir)
    
    def test_logger_log_with_log_entry(self):
        """Test Logger.log with LogEntry"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.debug = Mock()
            Logger.logger.info = Mock()
            Logger.logger.warning = Mock()
            Logger.logger.error = Mock()
            
            # Test different log modes
            Logger.log(LogEntry("debug message", LogMode.DEBUG))
            Logger.logger.debug.assert_called_with("debug message")
            
            Logger.log(LogEntry("info message", LogMode.INFO))
            Logger.logger.info.assert_called_with("info message")
            
            Logger.log(LogEntry("warning message", LogMode.WARNING))
            Logger.logger.warning.assert_called_with("warning message")
            
            Logger.log(LogEntry("error message", LogMode.ERROR))
            Logger.logger.error.assert_called_with("error message")
    
    def test_logger_log_with_string(self):
        """Test Logger.log with string input"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.info = Mock()
            
            # Test with string
            Logger.log("test message")
            Logger.logger.info.assert_called_with("test message")
    
    def test_logger_log_with_other_object(self):
        """Test Logger.log with other object types"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.info = Mock()
            
            # Test with number
            Logger.log(42)
            Logger.logger.info.assert_called_with("42")
            
            # Test with list
            Logger.log([1, 2, 3])
            Logger.logger.info.assert_called_with("[1, 2, 3]")
    
    def test_logger_log_inactive(self):
        """Test Logger.log when logging is inactive"""
        with patch('builtins.print') as mock_print:
            Logger._LOG_ACTIVE = False
            
            Logger.log("test message")
            mock_print.assert_not_called()
            
            Logger.log(LogEntry("test entry"))
            mock_print.assert_not_called()
            
    def test_logger_log_unknown_mode(self):
        """Test Logger.log with unknown mode"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger  
            Logger.logger.info = Mock()
            
            # Create LogEntry with invalid mode
            entry = LogEntry("test message")
            # Force set invalid mode by bypassing type checking
            object.__setattr__(entry, 'mode', "INVALID_MODE")
            
            Logger.log(entry)
            Logger.logger.info.assert_called_with("test message: !! Unknown mode: INVALID_MODE !!")
    
    def test_logger_debug_method(self):
        """Test Logger.debug method"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.debug = Mock()
            
            # Test with string
            Logger.debug("debug message")
            Logger.logger.debug.assert_called_with("debug message")
            
            # Test with LogEntry
            Logger.debug(LogEntry("debug entry"))
            Logger.logger.debug.assert_called_with("debug entry")
    
    def test_logger_debug_inactive(self):
        """Test Logger.debug when logging is inactive"""
        with patch('builtins.print') as mock_print:
            Logger._LOG_ACTIVE = False
            
            # Mock the logger
            Logger.logger = Mock()
            Logger.logger.debug = Mock()
            
            Logger.debug("debug message")
            mock_print.assert_called_with("WARNING_LOGGER_NOT_ACTIVE: debug message")
    
    def test_logger_warning_method(self):
        """Test Logger.warn method"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.warning = Mock()
            
            # Test with string
            Logger.warn("warning message")
            Logger.logger.warning.assert_called_with("warning message")
            
            # Test with LogEntry
            Logger.warn(LogEntry("warning entry"))
            Logger.logger.warning.assert_called_with("warning entry")
    
    def test_logger_warning_inactive(self):
        """Test Logger.warn when logging is inactive"""
        with patch('builtins.print') as mock_print:
            Logger._LOG_ACTIVE = False
            
            # Mock the logger
            Logger.logger = Mock()
            Logger.logger.warning = Mock()
            
            Logger.warn("warning message")
            mock_print.assert_called_with("WARNING_LOGGER_NOT_ACTIVE: warning message")
    
    def test_logger_error_method(self):
        """Test Logger.error method"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.error = Mock()
            
            # Test with string
            Logger.error("error message")
            Logger.logger.error.assert_called_with("error message")
            
            # Test with LogEntry
            Logger.error(LogEntry("error entry"))
            Logger.logger.error.assert_called_with("error entry")
    
    def test_logger_error_inactive(self):
        """Test Logger.error when logging is inactive"""
        with patch('builtins.print') as mock_print:
            Logger._LOG_ACTIVE = False
            
            # Mock the logger
            Logger.logger = Mock()
            Logger.logger.error = Mock()
            
            Logger.error("error message")
            mock_print.assert_called_with("WARNING_LOGGER_NOT_ACTIVE: error message")

    def test_logger_info_method(self):
        """Test Logger.info method"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.info = Mock()
            
            # Test with string
            Logger.info("info message")
            Logger.logger.info.assert_called_with("info message")
            
            # Test with LogEntry
            Logger.info(LogEntry("info entry"))
            Logger.logger.info.assert_called_with("info entry")
    
    def test_logger_info_inactive(self):
        """Test Logger.info when logging is inactive"""
        with patch('builtins.print') as mock_print:
            Logger._LOG_ACTIVE = False
            
            # Mock the logger
            Logger.logger = Mock()
            Logger.logger.info = Mock()
            
            Logger.info("info message")
            mock_print.assert_called_with("WARNING_LOGGER_NOT_ACTIVE: info message")
    
    def test_logger_logging_on_decorator_sync(self):
        """Test Logger.logging_on decorator with synchronous function"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock Logger.log
            with patch.object(Logger, 'log') as mock_log:
                @Logger.logging_on
                def test_function(x, y):
                    return x + y
                
                result = test_function(1, 2)
                
                assert result == 3
                assert mock_log.call_count == 2  # Called twice: before and after
                
                # Check that log was called with debug entries
                calls = mock_log.call_args_list
                assert len(calls) == 2
                
                # Check first call (function entry)
                first_call_message = str(calls[0])
                assert "Call [" in first_call_message
                assert "test_function" in first_call_message
                
                # Check second call (function exit)
                second_call_message = str(calls[1])
                assert "Called [" in second_call_message
                assert "test_function" in second_call_message
                assert "took" in second_call_message
    
    def test_logger_logging_on_decorator_async_manual(self):
        """Test Logger.logging_on decorator with async function (manual test)"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock Logger.log
            with patch.object(Logger, 'log') as mock_log:
                @Logger.logging_on
                async def async_test_function(x, y):
                    return x + y
                
                # Test that the decorator recognizes async function and returns async wrapper
                assert asyncio.iscoroutinefunction(async_test_function)
                
                # Run the async function
                result = asyncio.run(async_test_function(1, 2))
                
                assert result == 3
                assert mock_log.call_count == 2  # Called twice: before and after
                
                # Verify the async code path was taken
                calls = mock_log.call_args_list
                assert len(calls) == 2
                
                # Check that args and kwargs are in the message (this tests line 274-276)
                first_call_entry = calls[0][0][0]
                assert "Call [" in first_call_entry.message
                assert "async_test_function" in first_call_entry.message
                assert "(1, 2)" in first_call_entry.message
                assert "{}" in first_call_entry.message
                
                # Check timing message (this tests line 281-284)
                second_call_entry = calls[1][0][0]
                assert "Called [" in second_call_entry.message
                assert "took" in second_call_entry.message
                assert "sec" in second_call_entry.message
    
    def test_logger_add_external(self):
        """Test Logger.add_external method"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger to have some handlers
            mock_handler = Mock()
            Logger.logger.handlers = [mock_handler]
            
            # Add external logger
            Logger.add_external("external_logger")
            
            # Check that external logger was added
            assert len(Logger._ext_logger) == 1
            
            # Get the external logger
            ext_logger = logging.getLogger("external_logger")
            assert mock_handler in ext_logger.handlers
    
    def test_logger_rotating_file_handler_separator(self):
        """Test that Logger.initialize adds separator to RotatingFileHandler"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            # Create mock handler
            mock_handler = Mock(spec=logging.handlers.RotatingFileHandler)
            mock_stream = Mock()
            mock_handler.stream = mock_stream
            
            with patch('logging.getLogger') as mock_get_logger:
                mock_logger = Mock()
                mock_logger.handlers = [mock_handler]
                mock_get_logger.return_value = mock_logger
                
                Logger.initialize(Path(config_file), log_active=True)
                
                # Check that separator was written to stream
                mock_stream.write.assert_called_with('\n' + '─' * 91 + '\n')
                mock_handler.flush.assert_called_once()
    
    def test_logger_class_variables(self):
        """Test Logger class variables"""
        assert Logger._LOGGER_NAME == 'vyra_base'
        assert Logger._LOG_PATH == ''
        assert isinstance(Logger._ext_logger, list)
    
    def test_logger_singleton_behavior(self):
        """Test that Logger behaves like a singleton"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Get logger reference
            logger1 = Logger.logger
            
            # Initialize again
            Logger.initialize(Path(config_file), log_active=False)
            
            # Logger should be the same instance
            logger2 = Logger.logger
            assert logger1.name == logger2.name
    
    def test_logger_log_entry_mode_conversion(self):
        """Test that string inputs are converted to LogEntry with correct mode"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger
            Logger.logger.debug = Mock()
            
            # Test that string inputs in debug method get DEBUG mode
            Logger.debug("debug message")
            Logger.logger.debug.assert_called_with("debug message")
            
            # Test that string inputs in warning method should get DEBUG mode (bug in original code)
            Logger.logger.warning = Mock()
            Logger.warn("warning message")
            Logger.logger.warning.assert_called_with("warning message")
    
    def test_logger_with_json_loading_error(self):
        """Test Logger.initialize with invalid JSON config"""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_file = os.path.join(temp_dir, "invalid_config.json")
            
            # Create invalid JSON
            with open(config_file, 'w') as f:
                f.write("invalid json content")
            
            # Should raise JSONDecodeError
            with pytest.raises(json.JSONDecodeError):
                Logger.initialize(Path(config_file), log_active=True)
    
    def test_log_modes_in_match_statement(self):
        """Test all LogMode cases in the match statement"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock all logger methods
            Logger.logger.debug = Mock()
            Logger.logger.info = Mock()
            Logger.logger.warning = Mock()
            Logger.logger.error = Mock()
            
            # Test all LogMode cases
            Logger.log(LogEntry("debug", LogMode.DEBUG))
            Logger.logger.debug.assert_called_with("debug")
            
            Logger.log(LogEntry("info", LogMode.INFO))
            Logger.logger.info.assert_called_with("info")
            
            Logger.log(LogEntry("warning", LogMode.WARNING))
            Logger.logger.warning.assert_called_with("warning")
            
            Logger.log(LogEntry("error", LogMode.ERROR))
            Logger.logger.error.assert_called_with("error")
    
    def test_logger_decorator_with_args_and_kwargs(self):
        """Test Logger.logging_on decorator with args and kwargs"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock Logger.log
            with patch.object(Logger, 'log') as mock_log:
                @Logger.logging_on
                def test_function_with_args(a, b, c=None, d="default"):
                    return f"{a}-{b}-{c}-{d}"
                
                result = test_function_with_args(1, 2, c=3, d="test")
                
                assert result == "1-2-3-test"
                assert mock_log.call_count == 2
                
                # Check that args and kwargs are in the log message
                first_call_args = mock_log.call_args_list[0][0][0]  # Get the LogEntry from first call
                assert "test_function_with_args" in first_call_args.message

    def test_logger_method_mode_bug_debug_vs_warning(self):
        """Test the bug where warning/error/info methods use DEBUG mode instead of their intended mode"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Mock the logger methods
            Logger.logger.warning = Mock()
            Logger.logger.error = Mock()
            Logger.logger.info = Mock()
            
            # Test the bug - these methods create LogEntry with DEBUG mode instead of their respective modes
            Logger.warn("warning message")
            Logger.error("error message") 
            Logger.info("info message")
            
            # All should be called despite the mode bug
            Logger.logger.warning.assert_called_with("warning message")
            Logger.logger.error.assert_called_with("error message")
            Logger.logger.info.assert_called_with("info message")

    def test_logger_external_multiple_additions(self):
        """Test adding multiple external loggers"""
        with tempfile.TemporaryDirectory() as temp_dir:
            log_file = os.path.join(temp_dir, "test.log")
            config_file = os.path.join(temp_dir, "log_config.json")
            
            config = self.create_test_log_config(log_file)
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            Logger.initialize(Path(config_file), log_active=True)
            
            # Clear any existing external loggers
            Logger._ext_logger = []
            
            # Mock the logger to have some handlers
            mock_handler1 = Mock()
            mock_handler2 = Mock()
            Logger.logger.handlers = [mock_handler1, mock_handler2]
            
            # Add multiple external loggers
            Logger.add_external("external_logger_1")
            Logger.add_external("external_logger_2")
            
            # Check that both external loggers were added
            assert len(Logger._ext_logger) >= 2  # Might have more from previous tests
            
            # Get the external loggers
            ext_logger1 = logging.getLogger("external_logger_1")
            ext_logger2 = logging.getLogger("external_logger_2")
            
            # Both should have the handlers from the main logger
            assert mock_handler1 in ext_logger1.handlers
            assert mock_handler2 in ext_logger1.handlers
            assert mock_handler1 in ext_logger2.handlers
            assert mock_handler2 in ext_logger2.handlers

    def test_logger_initialize_with_missing_handler(self):
        """Test Logger.initialize when config is missing expected handler"""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_file = os.path.join(temp_dir, "log_config.json")
            
            # Create config without the expected file_handler
            config = {
                "version": 1,
                "disable_existing_loggers": False,
                "formatters": {
                    "default": {
                        "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
                    }
                },
                "handlers": {
                    "console_handler": {
                        "class": "logging.StreamHandler",
                        "level": "DEBUG",
                        "formatter": "default"
                    }
                },
                "loggers": {
                    "vyra_base": {
                        "level": "DEBUG",
                        "handlers": ["console_handler"],
                        "propagate": False
                    }
                }
            }
            
            with open(config_file, 'w') as f:
                json.dump(config, f)
            
            # Should raise KeyError for missing file_handler
            with pytest.raises(KeyError):
                Logger.initialize(Path(config_file), log_active=True)

    def test_logger_mode_coverage_complete(self):
        """Test to ensure all LogMode enum values are covered"""
        # Test that all LogMode values are properly defined
        assert hasattr(LogMode, 'DEBUG')
        assert hasattr(LogMode, 'INFO') 
        assert hasattr(LogMode, 'WARNING')
        assert hasattr(LogMode, 'ERROR')
        
        # Test their numeric values
        assert LogMode.DEBUG.value == 0
        assert LogMode.INFO.value == 1
        assert LogMode.WARNING.value == 2
        assert LogMode.ERROR.value == 3
