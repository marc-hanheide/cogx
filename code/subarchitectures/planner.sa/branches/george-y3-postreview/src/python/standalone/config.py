import logging

def logging_settings_from_dict(dict):
    for k,v in dict.iteritems():
        if not ":" in k:
            continue

        logger, key = k.split(":")
        if logger == "root":
            logger = None
            
        if key == "level":
            logging.getLogger(logger).setLevel(logging.__dict__[v])
        elif key == "filelevel":
            if logger is None:
                logger = "file"
            else:
                logger = "file."+logger
            logging.getLogger(logger).setLevel(logging.__dict__[v])
        elif key == "filename":
            set_logfile(v, logger)

def set_logfile(filename, name=None, propagate=False):
    if name:
        name = "file."+name
    else:
        name = "file"

    logger = logging.getLogger(name)
    for h in logger.handlers[:]:
        logger.removeHandler(h)

    h=logging.FileHandler(filename, mode='w')
    #h.setFormatter(logging.Formatter("%(name)s: %(message)s"))
    logger.addHandler(h)
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

logfile_scope = None

class LoggerProxy(object):
    def __init__(self, name=None):
        self.console = logging.getLogger(name)
        self.name = name

    def _log(self, level, msg, *args, **kwargs):
        self.console.log(level, msg, *args, **kwargs)

        elems = ["file"]
        if logfile_scope:
            elems.append(logfile_scope)
        if self.name:
            elems.append(self.name)

        logging.getLogger(".".join(elems)).log(level, msg, *args, **kwargs)

    def trace(self, msg, *args, **kwargs):
        self._log(logging.DEBUG, msg, *args, **kwargs)
        
    def debug(self, msg, *args, **kwargs):
        self._log(logging.DEBUG, msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self._log(logging.INFO, msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self._log(logging.WARNING, msg, *args, **kwargs)
        
    def error(self, msg, *args, **kwargs):
        self._log(logging.ERROR, msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self._log(logging.CRITICAL, msg, *args, **kwargs)

logging_factory = LoggerProxy
        
def set_logging_factory(factory):
    global logging_factory
    logging_factory = factory
        
def logger(name=None):
    return logging_factory(name)
