import atexit
import os
import sys
from datetime import datetime
from pathlib import Path


class _TeeStream:
    def __init__(self, console_stream, file_stream):
        self._console = console_stream
        self._file = file_stream

    def write(self, data):
        self._console.write(data)
        self._file.write(data)
        return len(data)

    def flush(self):
        self._console.flush()
        self._file.flush()

    def isatty(self):
        return self._console.isatty()


_LOGGER_INITIALIZED = False
_LOG_FILE_PATH = None


def setup_print_logger(log_dir="logs", prefix="run"):
    global _LOGGER_INITIALIZED, _LOG_FILE_PATH

    if _LOGGER_INITIALIZED:
        return _LOG_FILE_PATH

    Path(log_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pid = os.getpid()
    log_path = Path(log_dir) / f"{prefix}_{stamp}_{pid}.log"

    log_file = open(log_path, "a", encoding="utf-8", buffering=1)
    sys.stdout = _TeeStream(sys.__stdout__, log_file)
    sys.stderr = _TeeStream(sys.__stderr__, log_file)

    def _cleanup():
        try:
            sys.stdout.flush()
        except Exception:
            pass
        try:
            sys.stderr.flush()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass

    atexit.register(_cleanup)

    _LOGGER_INITIALIZED = True
    _LOG_FILE_PATH = str(log_path)
    return _LOG_FILE_PATH
