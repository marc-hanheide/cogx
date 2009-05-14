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

#ifndef FRAMEWORK_QUERY_H_
#define FRAMEWORK_QUERY_H_

#include "includes.hpp"


class FrameworkQuery {

public:
  FrameworkQuery(const std::string & _source, const std::string & _query) :
    m_source(_source),
    m_query(_query) {
  }
  
  FrameworkQuery(const FrameworkQuery & _query) :
    m_source(_query.m_source),
    m_query(_query.m_query) {
  }
  
  const std::string & getSource() const {
    return m_source;
  }
  
  const std::string & getQuery() const {
    return m_query;
  }
  
  void setQuery(const std::string & _query) {
    m_query = _query;
  }
  
private:
  std::string m_source;
  std::string m_query;
};

#endif
