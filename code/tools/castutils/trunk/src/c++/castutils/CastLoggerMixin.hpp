/**
 * @author Marko Mahnič
 * @date September 2011
 *
 */
#ifndef _CASTUTILS_CASTLOGGERMIXIN_H_
#define _CASTUTILS_CASTLOGGERMIXIN_H_

#include <cast/core/CASTComponent.hpp>
#include <cstdarg>
#include <string>

namespace castutils
{

/**
 * Enable a derived class to log through a CAST component.
 *
 * Can also be used as a standalone object.
 */
class CCastLoggerMixin
{
  cast::CASTComponent* pLogComponentMixin;
public:
  CCastLoggerMixin(cast::CASTComponent* pCastComponent = NULL) {
    pLogComponentMixin = pCastComponent;
  }
  void setLoggingComponent(cast::CASTComponent* pCastComponent){
    pLogComponentMixin = pCastComponent;
  }

  void println(const char *format, ...) const {
    if (!pLogComponentMixin) return;
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    pLogComponentMixin->println("%s", msg);
  }

  void println(const std::string & _s) const {
    println("%s", _s.c_str());
  }

  void error(const char *format, ...) const {
    if (!pLogComponentMixin) return;
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    pLogComponentMixin->error("%s", msg);
  }

  void error(const std::string & _s) const {
    error("%s", _s.c_str());
  }

  void log(const char *format, ...) const {
    if (!pLogComponentMixin) return;

    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    pLogComponentMixin->log("%s", msg);
  }

  void log(const std::string & _s) const {
    log("%s", _s.c_str());
  }

  void debug(const char * format, ...) const {
    if (!pLogComponentMixin) return;
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(msg, 1024, format, arg_list);
    va_end(arg_list);

    pLogComponentMixin->debug("%s", msg);
  }

  void debug(const std::string & _s) const {
    debug("%s", _s.c_str());
  }
};

} //namespace
#endif // _CASTUTILS_CASTLOGGERMIXIN_H_
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
