"""Open-Teleop Logger Package.

A simple, level-based logging system based on Python's standard logging module.
Includes a TRACE level for very detailed logging and level-specific formatting.
"""

from logging import DEBUG, INFO, WARNING, ERROR, CRITICAL
from open_teleop_logger.logger import (
    TRACE,
    get_logger,
    level_from_string,
    UnifiedFormatter
)

__all__ = [
    'TRACE', 'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL',
    'get_logger', 'level_from_string', 'UnifiedFormatter'
] 