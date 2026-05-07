"""Examples for Vyra logging configuration usage in modules."""

from __future__ import annotations

import logging
import time

from vyra_base.helper.logging_config import (
    VyraLoggingConfig,
    get_logger,
    log_exceptions,
    log_performance,
    temporary_log_level,
)


def setup_logging() -> None:
    """Initialize logging once at module startup."""
    VyraLoggingConfig.initialize(
        log_level="INFO",
        enable_colors=True,
        enable_console=True,
    )


@log_performance(logger_name="example_module", threshold_ms=5.0)
def simulated_workload(delay_s: float) -> str:
    """Simulate work to demonstrate performance logging."""
    time.sleep(delay_s)
    return "ok"


@log_exceptions(logger_name="example_module")
def failing_operation() -> None:
    """Raise an error to demonstrate exception logging."""
    raise RuntimeError("simulated failure for logging example")


def main() -> None:
    """Run all logging usage patterns."""
    setup_logging()

    logger = get_logger("example_module")
    logger.info("Module startup complete")

    with temporary_log_level("DEBUG", logger_name="example_module"):
        logger.debug("Debug details visible inside context manager")

    result = simulated_workload(0.01)
    logger.info("Workload result: %s", result)

    try:
        failing_operation()
    except RuntimeError:
        logger.info("Expected exception has been logged")


if __name__ == "__main__":
    main()
