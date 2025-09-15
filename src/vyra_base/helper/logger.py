from __future__ import annotations

import asyncio
import json
import logging
import logging.config
import logging.handlers
import os
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Optional, Union

from vyra_base.helper.func import deep_merge

class LogMode(Enum):
    """
    Enum for the different logging modes.

    * 0: debug-msg
    * 1: info-msg
    * 2: warning-msg
    * 3: error-msg
    """
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3


@dataclass(slots=True)
class LogEntry:
    """
    Object for Logger entries.

    :param message: The log message.
    :type message: str
    :param mode: The log mode (default: LogMode.INFO).
    :type mode: LogMode
    """
    message: str
    mode: LogMode = LogMode.INFO

    def debug(self):
        """
        Set the mode to debug.

        :return: Self with mode set to DEBUG.
        :rtype: LogEntry
        """
        self.mode = LogMode.DEBUG
        return self

    def error(self):
        """
        Set the mode to error.

        :return: Self with mode set to ERROR.
        :rtype: LogEntry
        """
        self.mode = LogMode.ERROR
        return self

    def warn(self):
        """
        Set the mode to warning.

        :return: Self with mode set to WARNING.
        :rtype: LogEntry
        """
        self.mode = LogMode.WARNING
        return self
    
    def warning(self):
        return self.warn(self)


class Logger:
    """
    Class for logging backend information.

    The logger object is used globally and is initialized only once,
    following the Singleton design pattern.
    """
    _LOGGER_NAME: str = 'vyra_base'
    _LOG_PATH: str = ''
    _LOG_ACTIVE: bool = False

    _ext_logger: list = []
    logger: logging.Logger

    pre_log_buffer: list[LogEntry] = []  # List to store log entries before logger is initialized

    @classmethod
    def initialize(
        cls, 
        log_config_path: Path, 
        log_active: bool=True,
        log_config: Optional[dict]=None
    ) -> None:
        """
        Initialize the logger with a configuration file.

        :param log_config_path: Path to the logging configuration file.
        :type log_config_path: Path
        :param log_active: Whether logging is active.
        :type log_active: bool
        """
        with Path(log_config_path).open(mode='r+') as content:
            loaded_log_config: dict = json.load(content)

        if log_config:
            loaded_log_config = deep_merge(loaded_log_config, log_config)

        # Pfad zum Logfile extrahieren
        logfile = loaded_log_config["handlers"]["file_handler"]["filename"]
        logdir = os.path.dirname(logfile)

        # Verzeichnis erstellen, falls nötig
        os.makedirs(logdir, exist_ok=True)

        logging.config.dictConfig(loaded_log_config)

        root_logger = logging.getLogger()
        for handler in list(root_logger.handlers):
            if isinstance(handler, logging.StreamHandler):
                root_logger.removeHandler(handler)

        Logger._LOG_ACTIVE = log_active
        Logger.logger = logging.getLogger(Logger._LOGGER_NAME)

        for handler in Logger.logger.handlers:
            
            if isinstance(handler, logging.handlers.RotatingFileHandler):
                # Trenner direkt in den Stream schreiben
                handler.stream.write('\n' + '─' * 91 + '\n')
                handler.flush()
                break

        [cls.log(entry) for entry in cls.pre_log_buffer]

    @classmethod
    def log(cls, entry: Union[LogEntry, str, Any]) -> None:
        """
        Log a message in different modes.

        :param entry: Message and log level to be logged. For more information see LogEntry class.
        :type entry: Union[LogEntry, str, Any]
        :return: None. If logging is not active, the message will be printed to stdout.
                 Otherwise, the message will be logged according to its mode.
        :rtype: None
        """
        if not isinstance(entry, LogEntry):
            entry = LogEntry(
                message=f"{entry}",
                mode=LogMode.INFO
            )

        if not hasattr(Logger, 'logger'):
            Logger.pre_log_buffer.append(entry)
            return None

        if not Logger._LOG_ACTIVE:
            print(f"WARNING_LOGGER_NOT_ACTIVE: {entry.message}")
            return None
        
        match entry.mode:
            case LogMode.DEBUG:
                Logger.logger.debug(entry.message)
            case LogMode.INFO:
                Logger.logger.info(entry.message)
            case LogMode.WARNING:
                Logger.logger.warning(entry.message)
            case LogMode.ERROR:
                Logger.logger.error(entry.message)
            case _:
                Logger.logger.info(f"{entry.message}: !! Unknown mode: {entry.mode} !!")

    @classmethod
    def debug(cls, entry: Union[LogEntry, str]):
        """
        Log a message in debug mode.

        Independent of the logging configuration, this function will always log the message in
        debug mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.

        :param entry: Message and log level to be logged. For more information see LogEntry class.
        :type entry: Union[LogEntry, str]
        :return: None. If logging is not active, the message will be printed to stdout.
                 Otherwise, the message will be logged according to its mode.
        :rtype: None
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.DEBUG)
        else:
            entry.mode = LogMode.DEBUG
        
        Logger.log(entry)

    @classmethod
    def warn(cls, entry: Union[LogEntry, str]):
        """
        Log a message in warning mode.

        Independent of the logging configuration, this function will always log the message in
        warning mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.

        :param entry: Message and log level to be logged. For more information see LogEntry class.
        :type entry: Union[LogEntry, str]
        :return: None. If logging is not active, the message will be printed to stdout.
                 Otherwise, the message will be logged according to its mode.
        :rtype: None
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.WARNING)
        else:
            entry.mode = LogMode.WARNING

        Logger.log(entry)

    @classmethod
    def warning(cls, entry: Union[LogEntry, str]):
        Logger.warn(entry)

    @classmethod
    def error(cls, entry: Union[LogEntry, str]):
        """
        Log a message in error mode.

        Independent of the logging configuration, this function will always log the message in
        error mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.

        :param entry: Message and log level to be logged. For more information see LogEntry class.
        :type entry: Union[LogEntry, str]
        :return: None. If logging is not active, the message will be printed to stdout.
                 Otherwise, the message will be logged according to its mode.
        :rtype: None
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.ERROR)
        else:
            entry.mode = LogMode.ERROR

        Logger.log(entry)

    @classmethod
    def info(cls, entry: Union[LogEntry, str]):
        """
        Log a message in info mode.

        Independent of the logging configuration, this function will always log the message in
        info mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.

        :param entry: Message and log level to be logged. For more information see LogEntry class.
        :type entry: Union[LogEntry, str]
        :return: None. If logging is not active, the message will be printed to stdout.
                 Otherwise, the message will be logged according to its mode.
        :rtype: None
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.INFO)
        else:
            entry.mode = LogMode.INFO

        Logger.log(entry)

    @staticmethod
    def logging_on(func):
        """
        Decorator-function to log the name of the functions that are called.

        The runtime duration of that function will also be logged.

        :param func: Function to be decorated.
        :type func: Callable
        :return: Wrapper function.
        :rtype: Callable
        """
        func_str = f'[ {func.__qualname__} ]'
        track_str = ''
        if asyncio.iscoroutinefunction(func):
            async def async_wrapper(*args, **kwargs):
                Logger.log(
                    LogEntry(
                        message=f'Call {func_str}{track_str}({args} {kwargs})')
                        .debug()
                )

                start_time = time.perf_counter()
                async_value = await func(*args, **kwargs)
                elapsed_time = time.perf_counter() - start_time
                Logger.log(
                    LogEntry(f'Called {func_str}{track_str} took {elapsed_time:.4f} sec')
                    .debug()
                )
                return async_value
            return async_wrapper
        else:
            def wrapper(*args, **kwargs):
                Logger.log(
                    LogEntry(f'Call {func_str}{track_str}')
                    .debug()
                )
                start_time = time.perf_counter()
                func_value = func(*args, **kwargs)
                elapsed_time = time.perf_counter() - start_time
                Logger.log(
                    LogEntry(f'Called {func_str}{track_str} took {elapsed_time:.4f} sec')
                    .debug()
                )
                return func_value
            return wrapper

    @classmethod
    def add_external(cls, descriptor: str) -> None:
        """
        Add an external logger by descriptor.

        :param descriptor: The logger descriptor.
        :type descriptor: str
        """
        ext_logger = logging.getLogger(descriptor)
        ext_logger.handlers = []  # Vorhandene Handler entfernen
        for handler in cls.logger.handlers:
            ext_logger.addHandler(handler)
            cls._ext_logger.append(ext_logger)