/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef BALT_TYPE
#define BALT_TYPE

/**
The job of this class is to provide static methods for mapping from a
name of a datatype to particular objects.
*/

//#include <balt/core/StringMap.hpp>
#include <balt/core/TypeMap.hpp>
#include <string>

//typedef cast::StringMap<std::string>::map TypeMap;

class BALTType {

public:
  /**
   * Returns the balt-typename of the datatype. This will be teh same
   * for the same class across languages and platforms.
   */
  template <class T>
  static const std::string& typeName() {
    return demangle(typeid(T));
  }

      
private:
  static const std::string& demangle(const std::type_info&);
  static std::string cfilt(const char* _typename);
  
  static TypeMap<std::string>::map m_typeMap;

};


template <>
const std::string& BALTType::typeName<std::string>();

#endif
