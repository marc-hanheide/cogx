/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "AlreadyExistsOnWMException.hpp"
#include <cstdarg>

using namespace std;

namespace cast {

  
/**
 * Except constructor.
 * \param file     (in) __FILE__ macro
 * \param function (in) __FUNCTION__ macro
 * \param line     (in) __LINE__ macro
 * \param format   (in) printf-style format string
 */
AlreadyExistsOnWMException::AlreadyExistsOnWMException(const cast::cdl::WorkingMemoryAddress& _wma, 
						       const char *file, 
						       const char *function, 
						       int line,
						       const char *format, ...) throw() 
  : WMException(_wma) 
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "AlreadyExistsOnWMException: %s:%s:%d: %s", file, function, line, what);
  _what = msg;
}

} //namespace cast
