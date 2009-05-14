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

#ifndef CAST_SUBARCHITECTURE_WORKING_MEMORY_PROTOCOL_H_
#define CAST_SUBARCHITECTURE_WORKING_MEMORY_PROTOCOL_H_

#include <cast/core/CASTCore.hpp>
#include "WorkingMemoryPullQuery.hpp"

#include <vector>

namespace cast {

class SubarchitectureWorkingMemoryProtocol {


public:
  static std::string createIDQuery(const std::string &_srcID, const std::string &_srcSA, const std::string &_subarch, const std::string &_id); 
  static std::string createExistsQuery(const std::string &_srcID, const std::string &_srcSA, const std::string &_subarch, const std::string &_id); 
  static std::string createOverwriteCountQuery(const std::string &_srcID, const std::string &_srcSA, const std::string &_subarch, const std::string &_id); 
  static std::string createTypeQuery(const std::string &_srcID, const std::string &_srcSA, const std::string &_subarch, const std::string &_type, const int &_count);
  static std::string createIDArrayQuery(const std::string &_srcID, const std::string &_srcSA, const std::string &_subarch, const std::vector<std::string> &_ids);
  static std::string createQuery(AbstractWorkingMemoryPullQuery * _pQuery);
  
  static AbstractWorkingMemoryPullQuery * parseQuery(const std::string &_query);
  
  static WorkingMemoryPullQuery<std::string> * extractID(const std::string &_query);
  static WorkingMemoryPullQuery<std::string> * extractExists(const std::string &_query);
  static WorkingMemoryPullQuery<std::string> * extractOverwriteCount(const std::string &_query);
  static AbstractWorkingMemoryPullQuery * extractUnknown(const std::string &_query);
  static WorkingMemoryPullQuery< std::vector<std::string> > * extractIDArray(const std::string &_query);
  static WorkingMemoryPullQuery<std::string> * extractType(const std::string &_query);
  
  static void tokenizeString(const std::string & _str,
			     std::vector < std::string > &_tokens,
			     const std::string & _delimiters  = " ");


private: 
  static std::string ID_PREFIX;
  static std::string EXISTS_PREFIX;
  static std::string OVERWRITE_COUNT_PREFIX;
  static std::string ID_ARRAY_PREFIX;
  static std::string TYPE_PREFIX;

  

};

} //namespace cast


#endif
