import logging

def set_logfile(filename, name=None, propagate=False):
    if name:
        name = "file."+name
    else:
        name = "file"

    logger = logging.getLogger(name)
    for h in logger.handlers[:]:
        logger.removeHandler(h)
    logger.addHandler(logging.FileHandler(filename, mode='w'))
    if not propagate:
        logger.propagate = 0
    else:
        logger.propagate = 1

logging.basicConfig(level=logging.WARNING)

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

file = logging.getLogger("file")
file.setLevel(level=logging.INFO)
file.addHandler(NullHandler())
file.propagate = 0



class LoggerProxy(object):
    def __init__(self, name=None):
        self.console = logging.getLogger(name)
        if name:
            self.file = logging.getLogger("file."+name)
        else:
            self.file = logging.getLogger("file")

    def debug(self, msg, *args, **kwargs):
        self.console.debug(msg, *args, **kwargs)
        self.file.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.console.info(msg, *args, **kwargs)
        self.file.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.console.warning(msg, *args, **kwargs)
        self.file.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.console.error(msg, *args, **kwargs)
        self.file.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.console.critical(msg, *args, **kwargs)
        self.file.critical(msg, *args, **kwargs)

def logger(name=None):
    return LoggerProxy(name)
