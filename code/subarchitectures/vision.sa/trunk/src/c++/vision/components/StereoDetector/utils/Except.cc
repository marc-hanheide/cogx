/**
 * @file Except.cc
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Exception class
 */

#include <stdarg.h>
#include "Except.hh"

namespace Z
{

/**
 * @brief Except constructor: Create exception message.\n
 * Example:\n
 *   Except(__FILE__, __FUNCTION__, __LINE__, "There were %d %s in the tree.", 42, "elephants");\n
 * output:\n
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."\n
 * Note: You can use the __HERE__ macro to get shorter statements:\n
 *   Except(__HERE__, "There were %d %s in the tree.", 42, "elephants");\n
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
Except::Except(const char *file, const char *function, int line, const char *format, ...) throw()
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

