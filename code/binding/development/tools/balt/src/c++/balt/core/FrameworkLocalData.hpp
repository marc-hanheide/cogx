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
 
#ifndef FRAMEWORK_LOCAL_DATA_H_
#define FRAMEWORK_LOCAL_DATA_H_


#include <cassert>
#include "includes.hpp"

/**
 * Generic class for wrapping data when transmitting it in the
 * framework.
 */
template <class T>
class FrameworkLocalData {

public:

    FrameworkLocalData() {
      m_pData = NULL;
      m_source = "";
    }

    FrameworkLocalData(const std::string & _source) {
      m_source = std::string(_source);
      m_pData = NULL;
    }


    /**
     * This constructor creates a new copy of _data.
     */
    FrameworkLocalData(const std::string & _source, const T & _data) {
      m_source = std::string(_source);
      m_pData = new T(_data);
    }

    /**
     * This constructor creates a new copy of _data.
     */
    FrameworkLocalData(const std::string & _source, T * _pData) {
      m_source = std::string(_source);
      m_pData = _pData;
    }

    /**
     * Copy constructor.
     */
    FrameworkLocalData(const FrameworkLocalData<T> & _fd) {
        m_source = std::string(_fd.m_source);
        m_pData = new T(*(_fd.m_pData));
    }


    /**
     * Destructor. This deletes the memory used by the data member.
     */
    ~FrameworkLocalData() {
      if(m_pData) {
	delete m_pData;
      }
      m_pData = NULL;
    }
    
    const T & getData() const {
      assert(m_pData != NULL);
      return (*m_pData);
    }

    /**
     * Get the internal data pointer. Can be used to directly
     * manipulate the wrapped data.
     *
     * @return 
     */
    T * & data() {
      assert(m_pData != NULL);
      return m_pData;
    }

    const std::string & getSource() const {
      return m_source;
    }

    /**
     * Copues the input as new stored data. Frees memory of previously
     * stored data if there is ant.
     */
    void setData(const T & _data) {
      if(m_pData) {
	delete m_pData;
      }
      m_pData = new T(_data);
    }

    /**
     * Set the internal data pointer to the input. Frees memory of
     * previously stored data if there is ant.
     */
    void setData(T * _pData) {
      if(m_pData) {
	delete m_pData;
      }
      m_pData = _pData;
    }

    void setSource(const std::string & _source) {
      m_source = std::string(_source);
    }


private:
    std::string m_source;
    T * m_pData;

};


#endif
