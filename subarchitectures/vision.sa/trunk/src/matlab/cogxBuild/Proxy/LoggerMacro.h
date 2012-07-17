#include "LoggerProxy.h"

#define MSSG(X) [&](){ ostringstream s; s << X; return s.str(); }()
#define TRACE(X) CLoggerProxy::getLogger().trace(MSSG(X))
#define DEBUG(X) CLoggerProxy::getLogger().debug(MSSG(X))
#define INFO(X)  CLoggerProxy::getLogger().info(MSSG(X))
#define WARN(X)  CLoggerProxy::getLogger().warn(MSSG(X))
#define ERROR(X) CLoggerProxy::getLogger().error(MSSG(X))

