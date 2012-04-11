/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   10.04.2012
 **/

#include "LoggerMixin.hpp"

using namespace std;
using namespace cast;


namespace castutils {


void LoggerMixin::warn(const std::string &s)
{
  warn(s.c_str());
}


void LoggerMixin::warn(const char *format, ...)
{
  CASTComponent::ComponentLoggerPtr logger = _component->getLogger();
  if(logger && logger->isWarnEnabled()) {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    CAST_WARN(logger, msg, _component->getLogAdditions());
  }
}


void LoggerMixin::fatal(const std::string &s)
{
  fatal(s.c_str());
}


void LoggerMixin::fatal(const char *format, ...)
{
  CASTComponent::ComponentLoggerPtr logger = _component->getLogger();
  if(logger && logger->isFatalEnabled()) {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    CAST_FATAL(logger, msg, _component->getLogAdditions());
  }
}


void LoggerMixin::cast_assert(
    const bool condition, const std::string &s)
{
  cast_assert(condition, s.c_str());
}


void LoggerMixin::cast_assert(
    const bool condition, const char *format, ...)
{
  CASTComponent::ComponentLoggerPtr logger = _component->getLogger();
  if(logger && logger->isErrorEnabled()) {
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    CAST_ASSERT(logger, condition, msg, _component->getLogAdditions());
  }
}



} // namespace castutils
