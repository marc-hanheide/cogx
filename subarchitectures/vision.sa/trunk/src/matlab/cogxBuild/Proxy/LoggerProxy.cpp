
#include "LoggerProxy.h"

namespace matlab {

static CLoggerProxy* plog = new CLoggerProxy();
void CLoggerProxy::setLogger(CLoggerProxy& logger)
{
   if (plog)
      delete plog;
   plog = &logger;
}

CLoggerProxy& CLoggerProxy::getLogger()
{
   return *plog;
}

};
