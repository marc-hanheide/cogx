/*
 * BALT - The Boxes and Lines Toolkit for component communication.
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

#ifndef STRING_MAP_H_
#define STRING_MAP_H_

#include <ext/hash_map>
#include <map>

// from http://forums.devshed.com/c-programming-42/tip-about-stl-hash-map-and-string-55093.html
namespace __gnu_cxx { 
  template<> struct hash<std::string>
  {
    size_t operator()(const std::string& _str) const { 
      return hash<const char*>()(_str.c_str()); 
    } 
  };
} // namespace __gnu_cxx


namespace cast {
  /// the C++ template typedef workaround (cf. http://www.gotw.ca/gotw/079.htm)
  /// the struct is not constructable
  template <typename T>
  struct StringMap {
    //typedef std::map<std::string,T> map;
    typedef __gnu_cxx::hash_map<std::string,T> map;
  private:    
    StringMap();
    StringMap(const StringMap&);
    ~StringMap();
  };  
} // namespace cast

#endif // STRING_MAP_H_

