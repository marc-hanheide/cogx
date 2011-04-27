/**
 * $Id: Except.cc,v 1.4 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include <stdarg.h>
#include "Except.hh"

namespace Z
{

/**
 * Except constructor.
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
Except::Except(const char *file, const char *function, int line,
               const char *format, ...) throw()
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
  _what = msg;
}

}

