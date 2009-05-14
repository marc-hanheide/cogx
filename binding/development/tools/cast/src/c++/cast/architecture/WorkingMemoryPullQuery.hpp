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

#ifndef CAST_SUBARCHITECTURE_WORKING_MEMORY_PULL_QUERY_H_
#define CAST_SUBARCHITECTURE_WORKING_MEMORY_PULL_QUERY_H_

#include <cast/core/CASTCore.hpp>

namespace cast {

  class AbstractWorkingMemoryPullQuery {

  public:

    enum WorkingMemoryQueryType {
      TYPE, ID, EXISTS, OVERWRITE_COUNT, ID_ARRAY, OTHER
    };

    AbstractWorkingMemoryPullQuery(const std::string & _srcID,
				   const std::string & _srcSA,
				   const std::string & _subarchitecture,
				   const int &_count, 
				   WorkingMemoryQueryType _type) {
      m_srcID = _srcID;
      m_srcSA = _srcSA;
      m_subarchitecture = _subarchitecture;
      m_count = _count;
      m_type = _type;
    }
  
    virtual ~AbstractWorkingMemoryPullQuery(){};

    const WorkingMemoryQueryType & getType() const {
      return m_type;
    }

    const int & getCount() const {
      return m_count;
    }

    const std::string & getSubarchitectureID() const {
      return m_subarchitecture;
    }

    /**
     * @return the srcID
     */
    const std::string & getSourceID() const {
      return m_srcID;
    }

    /**
     * @return the srcSA
     */
    const std::string & getSourceSA() const {
      return m_srcSA;
    }

  private: 
    std::string m_subarchitecture;
    int m_count;
    WorkingMemoryQueryType m_type;
    std::string m_srcID;
    std::string m_srcSA;

  };

  template <class T>
  class WorkingMemoryPullQuery: public AbstractWorkingMemoryPullQuery {

  public:

  
    WorkingMemoryPullQuery(const std::string & _srcID,
			   const std::string & _srcSA,
			   const std::string & _subarchitecture,
			   const int &_count, 
			   T * _pQueryObject,
			   WorkingMemoryQueryType _type) :
      AbstractWorkingMemoryPullQuery(_srcID,
				     _srcSA,
				     _subarchitecture,
				     _count,
				     _type) {
      m_pQueryObject = _pQueryObject;
    }

    ~WorkingMemoryPullQuery() {
      if(m_pQueryObject) {
	delete m_pQueryObject;
      }
    }

    T * getQueryObject() {
      return m_pQueryObject;
    }

  private:
    T * m_pQueryObject;
  };

} //namespace cast

#endif
