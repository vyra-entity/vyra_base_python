import sys
import asyncio
import logging
import traceback
from inspect import iscoroutinefunction

logger = logging.getLogger(__name__)

class ErrorTraceback:
    """
    Class to check traceback errors and feedback them to the logger.
    """

    @staticmethod
    def check_error_exist(error_details: list = [], log_print: bool=False) -> bool:
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

                if log_print:
                    logger.error(f'{exc_type}|{exc_obj}|')
                    
                tb_lines: list = []

                for line in traceback.format_tb(exc_tb):
                    tb_lines.append(line)

                if log_print:
                    logger.error('\n'.join(tb_lines))
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
