#!/usr/bin/env python3
# Copyright (c) 2023 Open-Teleop
# License: MIT

"""
Open-Teleop Logger Module.

A simple, level-based logging system based on Python's standard logging module.
Includes a TRACE level for very detailed logging and level-specific formatting.
"""

import os
import logging
import logging.handlers

# Define TRACE level (5) - more detailed than DEBUG (10)
TRACE = 5
logging.addLevelName(TRACE, "TRACE")

# Add trace method to Logger class
def trace(self, message, *args, **kwargs):
    """Log a message with TRACE level (more detailed than DEBUG)."""
    if self.isEnabledFor(TRACE):
        self._log(TRACE, message, args, **kwargs)

# Add the method to the Logger class
logging.Logger.trace = trace


class UnifiedFormatter(logging.Formatter):
    """
    Formatter that uses a unified format with timestamps, truncated log levels,
    and logger names for both console and file output.
    """
    
    def __init__(self):
        # Format: YYYY-MM-DD HH:MM:SS [LVL] [logger_name] Message
        super().__init__('%(asctime)s [%(levelname)3.3s] [%(name)s] %(message)s', 
                         datefmt='%Y-%m-%d %H:%M:%S')
        
        # Map for converting full level names to truncated versions
        self.level_map = {
            'CRITICAL': 'CRI',
            'ERROR': 'ERR',
            'WARNING': 'WRN',
            'INFO': 'INF',
            'DEBUG': 'DBG',
            'TRACE': 'TRC',
        }
    
    def format(self, record):
        # Save original level name
        original_levelname = record.levelname
        
        # Replace with truncated version
        if record.levelname in self.level_map:
            record.levelname = self.level_map[record.levelname]
        
        # Format the record
        result = super().format(record)
        
        # Restore original level name
        record.levelname = original_levelname
        
        return result


def setup_file_logging(log_dir, log_file_base_name):
    """
    Set up file logging with daily rotation.
    
    Args:
        log_dir: Directory to store log files
        log_file_base_name: Base name for log files
        
    Returns:
        Configured file handler
    """
    os.makedirs(log_dir, exist_ok=True)
    
    # File handler with daily rotation
    file_handler = logging.handlers.TimedRotatingFileHandler(
        os.path.join(log_dir, f"{log_file_base_name}.log"),
        when='midnight',
        backupCount=7  # Keep one week of logs
    )
    
    # Use the unified formatter for file logs
    file_handler.setFormatter(UnifiedFormatter())
    
    return file_handler


def setup_console_logging():
    """
    Set up console logging with the unified formatter.
    
    Returns:
        Configured console handler
    """
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(UnifiedFormatter())
    
    return console_handler


def get_logger(name, log_dir=None, console_level=logging.INFO, file_level=logging.DEBUG):
    """
    Get a configured logger with the specified name.
    
    Args:
        name: Logger name
        log_dir: Directory for log files (if None, file logging is disabled)
        console_level: Logging level for console output
        file_level: Logging level for file output
        
    Returns:
        A configured logger
    """
    logger = logging.getLogger(name)
    
    # Clear any existing handlers
    logger.handlers = []
    
    # Set the logger level to the lowest of console/file to capture all needed messages
    logger.setLevel(min(console_level, file_level if log_dir else console_level))
    
    # Add console handler
    console_handler = setup_console_logging()
    console_handler.setLevel(console_level)
    logger.addHandler(console_handler)
    
    # Add file handler if log_dir is specified
    if log_dir:
        file_handler = setup_file_logging(log_dir, name)
        file_handler.setLevel(file_level)
        logger.addHandler(file_handler)
    
    return logger


# Convenience functions for common level names to values
def level_from_string(level_name):
    """Convert a level name to its numeric value."""
    level_map = {
        'trace': TRACE,
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'warn': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL,
        'fatal': logging.CRITICAL,
    }
    return level_map.get(level_name.lower(), logging.INFO) 