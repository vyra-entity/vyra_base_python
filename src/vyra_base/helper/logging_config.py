"""
Professional logging configuration for vyra_base library.

This module provides a centralized logging configuration that can be used
across all vyra_base modules. It supports:
- Console and file handlers with rotation
- Hierarchical logger configuration
- Environment-based log level control
- Structured log formatting (standard and JSON)
- Performance logging decorators
- Context managers for temporary log level changes
- Rate limiting for repeated messages
- Colorized console output (optional)

Example:
    >>> from vyra_base.helper.logging_config import get_logger, log_performance
    >>> logger = get_logger(__name__)
    >>> logger.info("Application started", extra={"user_id": 123})
    >>> 
    >>> @log_performance(threshold_ms=100)
    >>> def slow_function():
    >>>     time.sleep(0.2)
"""
import logging
import logging.config
import logging.handlers
import os
import time
import functools
from pathlib import Path
from typing import Optional, Dict, Any, Callable
from contextlib import contextmanager
import json
from datetime import datetime


class JsonFormatter(logging.Formatter):
    """
    JSON formatter for structured logging.
    
    Outputs log records as JSON objects for easy parsing by log aggregators.
    """
    
    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_obj = {
            "timestamp": datetime.utcfromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }
        
        # Add exception info if present
        if record.exc_info:
            log_obj["exception"] = self.formatException(record.exc_info)
        
        # Add extra fields (safely with getattr)
        extra_fields = getattr(record, "extra_fields", None)
        if extra_fields:
            log_obj.update(extra_fields)
        
        return json.dumps(log_obj)


class ColoredFormatter(logging.Formatter):
    """
    Colored console formatter for better readability.
    
    Uses ANSI color codes to highlight different log levels.
    """
    
    COLORS = {
        'DEBUG': '\033[36m',      # Cyan
        'INFO': '\033[32m',       # Green
        'WARNING': '\033[33m',    # Yellow
        'ERROR': '\033[31m',      # Red
        'CRITICAL': '\033[35m',   # Magenta
    }
    RESET = '\033[0m'
    BOLD = '\033[1m'
    
    def format(self, record: logging.LogRecord) -> str:
        """Format log record with colors."""
        # Colorize log level
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = f"{self.COLORS[levelname]}{self.BOLD}{levelname}{self.RESET}"
        
        # Format the message
        formatted = super().format(record)
        
        # Reset levelname to avoid side effects
        record.levelname = levelname
        
        return formatted


class RateLimitFilter(logging.Filter):
    """
    Filter that rate-limits repeated log messages.
    
    Prevents log spam by suppressing duplicate messages within a time window.
    """
    
    def __init__(self, rate: float = 1.0):
        """
        Initialize rate limit filter.
        
        Args:
            rate: Minimum seconds between identical messages.
        """
        super().__init__()
        self.rate = rate
        self.last_log = {}
    
    def filter(self, record: logging.LogRecord) -> bool:
        """Filter log record based on rate limit."""
        key = (record.module, record.levelno, record.msg)
        now = time.time()
        
        if key in self.last_log:
            if now - self.last_log[key] < self.rate:
                return False
        
        self.last_log[key] = now
        return True


class VyraLoggingConfig:
    """
    Centralized logging configuration for vyra_base library.
    
    This class manages the logging configuration for the entire vyra_base library,
    providing consistent logging across all modules and components.
    """
    
    _initialized = False
    _default_level = logging.INFO
    _log_directory = Path("log/vyra")
    
    @classmethod
    def initialize(
        cls,
        log_level: Optional[str] = None,
        log_directory: Optional[Path] = None,
        log_config: Optional[Dict[str, Any]] = None,
        enable_console: bool = True,
        enable_file: bool = True,
        enable_colors: bool = True,
        enable_json: bool = False,
        rate_limit: Optional[float] = None,
    ) -> None:
        """
        Initialize the logging configuration.
        
        Args:
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
                      Can also be set via VYRA_LOG_LEVEL environment variable.
            log_directory: Directory where log files will be stored.
            log_config: Optional custom logging configuration dict.
            enable_console: Whether to enable console logging.
            enable_file: Whether to enable file logging.
            enable_colors: Whether to use colored output (console only).
            enable_json: Whether to use JSON formatting (file only).
            rate_limit: Minimum seconds between duplicate messages (None = disabled).
        """
        if cls._initialized:
            logging.getLogger(__name__).debug("Logging already initialized, skipping")
            return
        
        # Determine log level from parameter or environment
        if log_level is None:
            log_level = os.getenv("VYRA_LOG_LEVEL", "INFO").upper()
        
        # Determine log directory
        if log_directory is not None:
            cls._log_directory = Path(log_directory)
        
        # Create log directory if it doesn't exist
        cls._log_directory.mkdir(parents=True, exist_ok=True)
        
        # Build configuration
        if log_config is None:
            log_config = cls._build_default_config(
                log_level=log_level,
                log_directory=cls._log_directory,
                enable_console=enable_console,
                enable_file=enable_file,
                enable_colors=enable_colors,
                enable_json=enable_json,
                rate_limit=rate_limit,
            )
        
        # Apply configuration
        logging.config.dictConfig(log_config)
        
        cls._initialized = True
        
        logger = logging.getLogger("vyra_base")
        logger.info(
            f"✅ Logging initialized (level={log_level}, dir={cls._log_directory}, "
            f"colors={enable_colors}, json={enable_json})"
        )
    
    @classmethod
    def _build_default_config(
        cls,
        log_level: str,
        log_directory: Path,
        enable_console: bool,
        enable_file: bool,
        enable_colors: bool,
        enable_json: bool,
        rate_limit: Optional[float],
    ) -> Dict[str, Any]:
        """
        Build default logging configuration.
        
        Args:
            log_level: Logging level string.
            log_directory: Directory for log files.
            enable_console: Whether to enable console handler.
            enable_file: Whether to enable file handler.
            enable_colors: Whether to use colored console output.
            enable_json: Whether to use JSON formatting.
            rate_limit: Minimum seconds between duplicate messages.
            
        Returns:
            Logging configuration dictionary.
        """
        # Standard formatters
        formatters = {
            "standard": {
                "format": "%(asctime)s - %(name)s - %(levelname)-8s - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "detailed": {
                "format": "%(asctime)s - %(name)s - %(levelname)-8s - [%(filename)s:%(lineno)d] - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "simple": {
                "format": "%(levelname)-8s - %(name)s - %(message)s"
            },
            "colored": {
                "()": "vyra_base.helper.logging_config.ColoredFormatter",
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "json": {
                "()": "vyra_base.helper.logging_config.JsonFormatter",
            }
        }
        
        # Configure filters
        filters = {}
        if rate_limit is not None:
            filters["rate_limit"] = {
                "()": "vyra_base.helper.logging_config.RateLimitFilter",
                "rate": rate_limit
            }
        
        # Configure handlers
        handlers = {}
        root_handlers = []
        
        if enable_console:
            console_formatter = "colored" if enable_colors else "standard"
            console_handler: Dict[str, Any] = {
                "class": "logging.StreamHandler",
                "level": log_level,
                "formatter": console_formatter,
                "stream": "ext://sys.stdout"
            }
            if rate_limit is not None:
                console_handler["filters"] = ["rate_limit"]  # type: ignore[assignment]
            handlers["console"] = console_handler
            root_handlers.append("console")
        
        if enable_file:
            log_file = log_directory / "vyra_base.log"
            file_formatter = "json" if enable_json else "detailed"
            handlers["file"] = {
                "class": "logging.handlers.RotatingFileHandler",
                "level": log_level,
                "formatter": file_formatter,
                "filename": str(log_file),
                "maxBytes": 10485760,  # 10MB
                "backupCount": 10,
                "encoding": "utf8"
            }
            root_handlers.append("file")
        
        # Loggers configuration
        loggers = {
            "vyra_base": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
            # Sub-loggers for specific modules
            "vyra_base.core": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
            "vyra_base.com": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
            "vyra_base.storage": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
            "vyra_base.state": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
            "vyra_base.security": {
                "level": log_level,
                "handlers": root_handlers,
                "propagate": False
            },
        }
        
        config = {
            "version": 1,
            "disable_existing_loggers": False,
            "formatters": formatters,
            "filters": filters,
            "handlers": handlers,
            "loggers": loggers,
            "root": {
                "level": "WARNING",
                "handlers": root_handlers if root_handlers else []
            }
        }
        
        return config
    
    @classmethod
    def get_logger(cls, name: str) -> logging.Logger:
        """
        Get a logger instance.
        
        Ensures logging is initialized before returning the logger.
        
        Args:
            name: Logger name (typically __name__ of the module).
            
        Returns:
            Logger instance.
        """
        if not cls._initialized:
            cls.initialize()
        
        return logging.getLogger(name)
    
    @classmethod
    def set_level(cls, level: str, logger_name: Optional[str] = None) -> None:
        """
        Dynamically change logging level.
        
        Args:
            level: New logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
            logger_name: Specific logger to update, or None for root vyra_base logger.
        """
        if logger_name is None:
            logger_name = "vyra_base"
        
        logger = logging.getLogger(logger_name)
        numeric_level = getattr(logging, level.upper(), logging.INFO)
        logger.setLevel(numeric_level)
        
        # Also update handlers
        for handler in logger.handlers:
            handler.setLevel(numeric_level)
        
        logger.info(f"Log level changed to {level} for {logger_name}")
    
    @classmethod
    def add_handler_to_logger(cls, logger_name: str, handler: logging.Handler) -> None:
        """
        Add a custom handler to a specific logger.
        
        This is useful for adding additional logging outputs (e.g., to external systems).
        
        Args:
            logger_name: Name of the logger to add handler to.
            handler: Handler instance to add.
        """
        logger = logging.getLogger(logger_name)
        
        # Avoid duplicate handlers
        if handler not in logger.handlers:
            logger.addHandler(handler)
            logger.debug(f"Added custom handler to {logger_name}")
    
    @classmethod
    def reset(cls) -> None:
        """
        Reset the logging configuration.
        
        This is mainly useful for testing.
        """
        cls._initialized = False
        logging.root.handlers.clear()


def get_logger(name: str) -> logging.Logger:
    """
    Convenience function to get a logger.
    
    Args:
        name: Logger name (typically __name__).
        
    Returns:
        Logger instance.
        
    Example:
        >>> logger = get_logger(__name__)
        >>> logger.info("Application started")
    """
    return VyraLoggingConfig.get_logger(name)


@contextmanager
def temporary_log_level(level: str, logger_name: Optional[str] = None):
    """
    Context manager for temporarily changing log level.
    
    Args:
        level: Temporary log level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
        logger_name: Specific logger to update, or None for vyra_base logger.
        
    Example:
        >>> with temporary_log_level("DEBUG"):
        >>>     # Debug logging enabled here
        >>>     logger.debug("Detailed information")
        >>> # Original level restored here
    """
    if logger_name is None:
        logger_name = "vyra_base"
    
    logger = logging.getLogger(logger_name)
    original_level = logger.level
    
    try:
        VyraLoggingConfig.set_level(level, logger_name)
        yield logger
    finally:
        logger.setLevel(original_level)


def log_performance(
    threshold_ms: float = 100.0,
    logger_name: Optional[str] = None,
    level: str = "WARNING"
):
    """
    Decorator for logging function execution time.
    
    Logs a warning if execution time exceeds threshold.
    
    Args:
        threshold_ms: Threshold in milliseconds for logging.
        logger_name: Logger to use (or None for function's module logger).
        level: Log level to use (DEBUG, INFO, WARNING, ERROR).
        
    Example:
        >>> @log_performance(threshold_ms=100)
        >>> def slow_function():
        >>>     time.sleep(0.2)
        >>>     return "done"
        >>> # Logs: "⚠️ slow_function took 200.5ms (threshold: 100.0ms)"
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal logger_name
            if logger_name is None:
                logger_name = func.__module__
            
            logger = logging.getLogger(logger_name)
            
            start_time = time.perf_counter()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                elapsed_ms = (time.perf_counter() - start_time) * 1000
                
                if elapsed_ms > threshold_ms:
                    log_method = getattr(logger, level.lower(), logger.warning)
                    log_method(
                        f"⚠️ {func.__name__} took {elapsed_ms:.1f}ms "
                        f"(threshold: {threshold_ms}ms)",
                        extra={
                            "function": func.__name__,
                            "elapsed_ms": elapsed_ms,
                            "threshold_ms": threshold_ms
                        }
                    )
                else:
                    logger.debug(
                        f"✓ {func.__name__} took {elapsed_ms:.1f}ms",
                        extra={
                            "function": func.__name__,
                            "elapsed_ms": elapsed_ms
                        }
                    )
        
        return wrapper
    return decorator


def log_exceptions(logger_name: Optional[str] = None):
    """
    Decorator for logging exceptions with full traceback.
    
    Args:
        logger_name: Logger to use (or None for function's module logger).
        
    Example:
        >>> @log_exceptions()
        >>> def risky_function():
        >>>     raise ValueError("Something went wrong")
        >>> # Logs exception with full traceback before re-raising
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal logger_name
            if logger_name is None:
                logger_name = func.__module__
            
            logger = logging.getLogger(logger_name)
            
            try:
                return func(*args, **kwargs)
            except Exception as e:
                logger.exception(
                    f"❌ Exception in {func.__name__}: {str(e)}",
                    extra={
                        "function": func.__name__,
                        "exception_type": type(e).__name__,
                        "exception_message": str(e)
                    }
                )
                raise
        
        return wrapper
    return decorator
