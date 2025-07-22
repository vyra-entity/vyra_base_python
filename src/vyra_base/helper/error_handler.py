import sys
import asyncio
import traceback
from inspect import iscoroutinefunction

from vyra_base.helper.logger import Logger, LogEntry

class ErrorTraceback:
    """
    Class to check traceback errors and feedback them to the logger.
    """

    @staticmethod
    def check_error_exist(error_details: list = []) -> bool:
        """
        Check if an error occurred and log it.

        :param error_details: List to append error details to.
        :type error_details: list
        :return: True if an error exists, otherwise False.
        :rtype: bool
        """
        exc_type, exc_obj, exc_tb = sys.exc_info()
        if any([exc_type, exc_obj, exc_tb]):
            if exc_type == asyncio.exceptions.CancelledError:
                pass
            else:
                error_details.append(traceback.format_exc())

                Logger.log(LogEntry(
                    f'{exc_type}|{exc_obj}|').error())
                tb_lines: list = []

                for line in traceback.format_tb(exc_tb):
                    tb_lines.append(line)

                Logger.log(LogEntry(
                    '\n'.join(tb_lines)).error())
                return True

        return False

    @staticmethod
    def w_check_error_exist(func):
        """
        Decorator to check if an error occurred after function execution.

        :param func: The function to decorate.
        :type func: callable
        :return: The wrapped function.
        :rtype: callable
        """
        if iscoroutinefunction(func):
            async def async_wrapper(*args, **kwargs):
                """
                Wrapper for asynchronous functions.

                :param args: Positional arguments.
                :param kwargs: Keyword arguments.
                :return: Result of the wrapped function.
                """
                try:
                    return await func(*args, **kwargs)
                finally:
                    ErrorTraceback.check_error_exist()

            return async_wrapper
        else:
            def sync_wrapper(*args, **kwargs):
                """
                Wrapper for synchronous functions.

                :param args: Positional arguments.
                :param kwargs: Keyword arguments.
                :return: Result of the wrapped function.
                """
                try:
                    return func(*args, **kwargs)
                finally:
                    ErrorTraceback.check_error_exist()

            return sync_wrapper
