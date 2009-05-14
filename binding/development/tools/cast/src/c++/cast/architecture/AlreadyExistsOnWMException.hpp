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

#ifndef ALREADYEXISTSONWM_EXCEPTION_H_
#define ALREADYEXISTSONWM_EXCEPTION_H_

#include "WMException.hpp"

namespace cast {

/**
 * An informative exception class.
 * Example:
 *   Except(__FILE__, __FUNCTION__, __LINE__, "There were %d %s in the tree.",
 *          42, "elephants");
 * output:
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."
 * Note: You can use the __HERE__ macro to get shorter statements:
 *   Except(__HERE__, "There were %d %s in the tree.", 42, "elephants");
 *
 * Taken from zwork.
 */


class AlreadyExistsOnWMException : public WMException {
  std::string _what;

 public:

  AlreadyExistsOnWMException(const cast::cdl::WorkingMemoryAddress& _wma, 
			     const char *file, const char *function, int line,
			     const char *format, ...) throw() ;
  virtual ~AlreadyExistsOnWMException() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void set(const std::string &s) {_what = s;}

private:
  /// not correct
  AlreadyExistsOnWMException();
};

}

#endif
