from __future__ import annotations

import asyncio
import json
import logging
import logging.config
import time
import os

from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from typing import Union


class LogMode(Enum):
    """Enum for the different logging modes.

    0: debug-msg
    1: info-msg
    2: warning-msg
    3: error-msg
    """
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3


@dataclass(slots=True)
class LogEntry:
    """ Object for Logger entries. """
    message: str
    mode: LogMode = LogMode.INFO

    def debug(self):
        """Set the mode to debug."""
        self.mode = LogMode.DEBUG
        return self

    def error(self):
        """Set the mode to error."""
        self.mode = LogMode.ERROR
        return self

    def warn(self):
        """Set the mode to warning."""
        self.mode = LogMode.WARNING
        return self


class Logger:
    """Class for logging backend information. The logger object is used globally
       therefore it is initialized only once. Like the OOP
       Singleton design pattern.
    """
    _LOGGER_NAME = 'varioboticos_base'
    _LOG_PATH: str = ''
    _LOG_ACTIVE: bool = False

    _ext_logger: list = []
    logger: logging.Logger

    @classmethod
    def initialize(
        cls, 
        log_config_path: Path, 
        log_active: bool=True):

        with Path(log_config_path).open(mode='r+') as content:
            log_config = json.load(content)

            # Pfad zum Logfile extrahieren
            logfile = log_config["handlers"]["debug_file_handler"]["filename"]
            logdir = os.path.dirname(logfile)

            # Verzeichnis erstellen, falls nötig
            os.makedirs(logdir, exist_ok=True)

            logging.config.dictConfig(log_config)

        print(f"Logger initialized with config: {log_config}")

        Logger._LOG_ACTIVE = log_active

        Logger.logger = logging.getLogger(Logger._LOGGER_NAME)

        Logger.debug('-'*50)
        Logger.debug('Started new Logger')

    @classmethod
    def log(cls, entry: LogEntry) -> None:
        """Logging a message in different modis.

        Args:
            LogEntry: Message and log level to be logged.. For more information
            see LogEntry class.

        Returns:
            None: If logging is not active, the message will be printed to stdout.
            Otherwise, the message will be logged according to its mode.
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.INFO)

        if not Logger._LOG_ACTIVE:
            print(entry.message)
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
        """Logging a message in debug mode. Independent of the
        logging configuration, this function will always log the message in
        debug mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.


        Args:
            LogEntry: Message and log level to be logged.. For more information
            see LogEntry class.

        Returns:
            None: If logging is not active, the message will be printed to stdout.
            Otherwise, the message will be logged according to its mode.
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.DEBUG)

        if not Logger._LOG_ACTIVE:
            print(entry.message)

        Logger.logger.debug(entry.message)

    @classmethod
    def warning(cls, entry: Union[LogEntry, str]):
        """Logging a message in warning mode. Independent of the
        logging configuration, this function will always log the message in
        warning mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.


        Args:
            LogEntry: Message and log level to be logged.. For more information
            see LogEntry class.

        Returns:
            None: If logging is not active, the message will be printed to stdout.
            Otherwise, the message will be logged according to its mode.
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.DEBUG)

        if not Logger._LOG_ACTIVE:
            print(entry.message)

        Logger.logger.warning(entry.message)

    @classmethod
    def error(cls, entry: Union[LogEntry, str]):
        """Logging a message in error mode. Independent of the
        logging configuration, this function will always log the message in
        error mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.


        Args:
            LogEntry: Message and log level to be logged.. For more information
            see LogEntry class.

        Returns:
            None: If logging is not active, the message will be printed to stdout.
            Otherwise, the message will be logged according to its mode.
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.DEBUG)

        if not Logger._LOG_ACTIVE:
            print(entry.message)

        Logger.logger.error(entry.message)

    @classmethod
    def info(cls, entry: Union[LogEntry, str]):
        """Logging a message in info mode. Independent of the
        logging configuration, this function will always log the message in
        info mode. This is useful for logging messages that should always be
        logged, regardless of the logging configuration.


        Args:
            LogEntry: Message and log level to be logged.. For more information
            see LogEntry class.

        Returns:
            None: If logging is not active, the message will be printed to stdout.
            Otherwise, the message will be logged according to its mode.
        """
        if isinstance(entry, str):
            entry = LogEntry(message=entry, mode=LogMode.DEBUG)

        if not Logger._LOG_ACTIVE:
            print(entry.message)

        Logger.logger.info(entry.message)

    @staticmethod
    def logging_on(func):
        """Decorator-function to log the name of the functions that are called.

        The runtime duration of that function will also been logged.

        Args:
            func ([type]): [function to be decorated]

        Returns:
            [type]: [wrapper function]
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
        ext_logger = logging.getLogger(descriptor)
        ext_logger.handlers = []  # Vorhandene Handler entfernen
        for handler in cls.logger.handlers:
            ext_logger.addHandler(handler)
            cls._ext_logger.append(ext_logger)