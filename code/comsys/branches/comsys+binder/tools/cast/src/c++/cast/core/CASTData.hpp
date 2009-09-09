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

#ifndef CAST_CAST_DATA_HPP_
#define CAST_CAST_DATA_HPP_

#include <cast/core/CASTUtils.hpp>

namespace cast {

  /**
   * Generic class for wrapping data up with its working memory
   * information.
   */
  template <class T>
  class CASTData  {

  public:
    
    /**
     * This constructor uses the existing data pointed to by _data as
     * its own.
     *
     * @param _id The id for the data.
     * @param _data Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     IceInternal::Handle<T> _data) :
      m_id(_id),
      m_type(typeName<T>()),
      m_data(_data),
      m_version(0)
    {}
    
    /**
     * This constructor uses the existing data pointed to by _data as
     * its own.
     *
     * @param _id The id for the data.
     * @param _version The version number of the data.
     * @param _data Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     const int _version,
	     IceInternal::Handle<T> _data) :
      m_id(_id),
      m_type(typeName<T>()),
      m_data(_data),
      m_version(_version)
    {}
    
    /**
     * This constructor cast data from the entry and stores details.
     *
     * @param _entry The working memory entry
     */
    CASTData(const cdl::WorkingMemoryEntryPtr _entry) :
      m_id(_entry->id),
      m_type(typeName<T>()),
      m_data(IceInternal::Handle<T>::dynamicCast(_entry->entry)),
      m_version(_entry->version)
    {}
    

    /**
     * Copy constructor
     *
     * @param _rhs Other object.
     */
    CASTData(const CASTData<T> & _rhs) :
      m_id(_rhs.m_id),
      m_type(typeName<T>()),
      m_data(_rhs.m_data),
      m_version(_rhs.m_version)
    {}
    



  
    /**
     * Get the id of the object.
     * 
     * @return The object's id.
     */  
    /*   const std::string & getID() const { */
    /*     return m_id; */
    /*   } */

    const std::string& getID() const {
      return m_id;
    }


//     /**
//      * Set the id of the object.
//      * 
//      * @param _id
//      *            The object's id.
//      */  
//     void setID(const std::string & _id) {
//       m_id = std::string(_id);
//     }

  
    /**
     * Get the data pointer. Can be used to directly manipulate the
     * wrapped data.
     *
     * @return The object's pointer to the data object.
     */
    IceInternal::Handle<T>  getData() {
      return m_data;
    }

    /**
     * Get the ontological type of the data object.
     * @return The ontological type of the data.
     */
    const std::string & getType() const {
      return m_type;
    }


    int getVersion() const {
      return m_version;
    }


    /**
     * Shallow assignment.
     */
    CASTData<T> & operator=(const CASTData<T> & _rhs) {
      m_data = _rhs.m_data;
      m_id = _rhs.m_id;
      m_version = _rhs.m_version;
      return *this;
    }



  protected:
    ///The id of the data, used to locate it in working memory
    std::string m_id;
    ///the ontological type of the data
    std::string m_type;
    ///a pointer to the actual stored data
    IceInternal::Handle<T> m_data;
    ///the version number of this data
    int m_version;
  };


} //namespace cast


#endif
