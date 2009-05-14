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

#ifndef CAST_ARCHITECTURE_CONFIGURATION_H_
#define CAST_ARCHITECTURE_CONFIGURATION_H_

#include <cast/cdl/CAST.hh>

#include <string>



namespace cast {

/**
Manages mappings between ontological types and framework datatypes
 */
class ArchitectureConfiguration {
  
public:
  
  ArchitectureConfiguration(){};
  virtual ~ArchitectureConfiguration(){};
  
  static const std::string XARCH_PREFIX;
  static const std::string GLOBAL_PREFIX;

};

} //namespace cast

#endif
