/*
 * Author: Marko Mahnič
 * Created: June 2010
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef EXCEPTION_WQJ9V10Y
#define EXCEPTION_WQJ9V10Y

#include <exception>
#include <string>
#include <sstream>

namespace cogx { namespace vision {

// Examples:
//    throw Exception("An exception with a simple string description");
//    throw Exception(EXCEPTMSG("A more complex -" << "ostringstream" << "- description"));
class Exception: public std::exception
{
   std::string _what;
public:
   Exception(const std::string& what) { _what = what; }
   Exception(std::ostringstream& what) { _what = what.str(); }
   ~Exception() throw() {}
   virtual const char* what() const throw() { return _what.c_str(); }
};
//#define EXCEPTMSG(streamexpr)  ((std::ostringstream&)(std::ostringstream() << streamexpr)).str()
#define EXCEPTMSG(streamexpr)  (std::ostringstream&)(std::ostringstream() << streamexpr)

}} // namespace
#endif /* end of include guard: EXCEPTION_WQJ9V10Y */
// vim:sw=3:ts=8:et
