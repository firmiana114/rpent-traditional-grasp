"""Project-local logging helpers."""

from __future__ import annotations

import logging

_LOGGER_NAME = "rpent_traditional_grasp"
_package_logger = logging.getLogger(_LOGGER_NAME)
_package_logger.setLevel(logging.INFO)
_package_logger.addHandler(logging.NullHandler())


def get_logger(name: str = "") -> logging.Logger:
    """Return a logger under the standalone project namespace."""
    return logging.getLogger(f"{_LOGGER_NAME}.{name}" if name else _LOGGER_NAME)


def configure_logging(level: int = logging.INFO) -> None:
    """Configure a concise console logger for standalone commands."""
    package_logger = logging.getLogger(_LOGGER_NAME)
    package_logger.setLevel(max(level, logging.DEBUG))
    if any(
        not isinstance(handler, logging.NullHandler)
        for handler in package_logger.handlers
    ):
        return
    package_logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)s [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
    )
    package_logger.addHandler(handler)
    package_logger.propagate = False
