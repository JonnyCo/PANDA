import logging

class LogHandler:
    def __init__(self, class_name, module_name, log_level="info", log_file_level="debug"):
        self.logger = logging.getLogger(class_name)
        self.logger.setLevel(self._get_log_level(log_level))

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(self._get_log_level(log_level))
        console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(console_formatter)

        # File handler
        file_handler = logging.FileHandler(f"{module_name}.log")
        file_handler.setLevel(self._get_log_level(log_file_level))
        file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)

        # Adding handlers to the logger
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)

    def _get_log_level(self, level_str):
        """Convert log level string to logging module level."""
        levels = {
            "debug": logging.DEBUG,
            "info": logging.INFO,
            "warning": logging.WARNING,
            "error": logging.ERROR,
            "critical": logging.CRITICAL
        }
        return levels.get(level_str.lower(), logging.INFO)

    def debug(self, message, func=None):
        self._log(logging.DEBUG, message, func)

    def info(self, message, func=None):
        self._log(logging.INFO, message, func)

    def warning(self, message, func=None):
        self._log(logging.WARNING, message, func)

    def error(self, message, func=None):
        self._log(logging.ERROR, message, func)

    def critical(self, message, func=None):
        self._log(logging.CRITICAL, message, func)

    def exception(self, message, func=None):
        """Log an exception with traceback."""
        self._log(logging.ERROR, message, func, exc_info=True)

    def _log(self, level, message, func, exc_info=False):
        """Helper method to include the function name in log messages."""
        if func:
            message = f"{func.__name__} - {message}"
        self.logger.log(level, message, exc_info=exc_info)

    def setLogLevel(self, log_level, log_file_level):
        """Update the log levels for console and file handlers."""
        for handler in self.logger.handlers:
            if isinstance(handler, logging.StreamHandler):
                handler.setLevel(self._get_log_level(log_level))
            elif isinstance(handler, logging.FileHandler):
                handler.setLevel(self._get_log_level(log_file_level))

    def removeHandler(self):
        """Remove all handlers from the logger."""
        for handler in self.logger.handlers:
            self.logger.removeHandler(handler)
