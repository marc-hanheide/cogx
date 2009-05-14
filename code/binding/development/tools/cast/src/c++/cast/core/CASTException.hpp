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

#ifndef CAST_EXCEPTION_H_
#define CAST_EXCEPTION_H_

#include <balt/core/BALTException.hpp>

namespace cast {

  class CASTException : public BALTException {
    std::string _what;
    
  public:
    CASTException() throw() {};
    CASTException(const char *file, const char *function, int line,
		  const char *format, ...) throw();
    virtual ~CASTException() throw() {}
    virtual const char* what() const throw() {return _what.c_str();}
    void set(const std::string &s) {_what = s;}
  };
  
} //namespace cast

#endif
