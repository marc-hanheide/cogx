/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskBase.h"
#include "SOIFilter.h"
#include <IceUtil/IceUtil.h>

namespace cast {

static long long g_order = 0;
static IceUtil::Monitor<IceUtil::Mutex> g_OrderMonitor;

long long WmEvent::getEventOrder()
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(g_OrderMonitor);
  ++g_order;
  return g_order;
}

WmTaskExecutor::WmTaskExecutor(SOIFilter* soif)
  : castutils::CCastLoggerMixin(soif)
{
  pSoiFilter = soif;
}

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
