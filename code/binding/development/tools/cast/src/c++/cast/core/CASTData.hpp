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

#ifndef CAST_CAST_TYPED_DATA_WITH_ID_H_
#define CAST_CAST_TYPED_DATA_WITH_ID_H_

#include "CASTTypedData.hpp"

namespace cast {

  /**
   * Generic class for wrapping data up with an ontological type
   */
  template <class T>
  class CASTData : public CASTTypedData<T> {

  public:
  

    /**
     * Create an empty pointer with a null data object.
     */
    CASTData() 
      : CASTTypedData<T>() {
      m_id = "";
    }

    /**
     * Copy from non-IDed object. Creates a new cop of the data stored by _fd.
     *
     * @param _id The id for the data.
     * @param _fd The data to copy.
     */
    CASTData(const std::string & _id, const CASTTypedData<T> & _fd) 
      : CASTTypedData<T>(_fd){
      m_id = std::string(_id);
    }

  
 
    /**
     * This constructor uses the existing data pointed to by _pData as
     * its own.
     *
     * @param _id The id for the data.
     * @param _type The type for the data.
     * @param _pData Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     const std::string & _type,  
	     T * _pData)
      : CASTTypedData<T>(_type,_pData) {
      m_id = std::string(_id);
    }

    /**
     * This constructor uses the existing data pointed to by _pData as
     * its own.
     *
     * @param _id The id for the data.
     * @param _type The type for the data.
     * @param _pData Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     const std::string & _type,  
	     const T & _data)
      : CASTTypedData<T>(_type,_data) {
      m_id = std::string(_id);
    }

        /**
     * This constructor uses the existing data pointed to by _pData as
     * its own.
     *
     * @param _id The id for the data.
     * @param _type The type for the data.
     * @param _pData Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     const std::string & _type,
	     int _version,
	     T * _pData)
      : CASTTypedData<T>(_type,_version,_pData) {
      m_id = std::string(_id);
    }

    /**
     * This constructor uses the existing data pointed to by _pData as
     * its own.
     *
     * @param _id The id for the data.
     * @param _type The type for the data.
     * @param _pData Pointer to the data to use.
     */
    CASTData(const std::string & _id, 
	     const std::string & _type, 
	     int _version,
	     const T & _data)
      : CASTTypedData<T>(_type,_version,_data) {
      m_id = std::string(_id);
    }
    
    virtual ~CASTData() {
      //cout<<"~CASTData"<<endl;
      //cout<<m_id<<endl;
    }
  
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


    /**
     * Set the id of the object.
     * 
     * @param _id
     *            The object's id.
     */  
    void setID(const std::string & _id) {
      m_id = std::string(_id);
    }

  
    /**
     * Shallow assignment.
     */
    CASTData<T> & operator=(const CASTData<T> & _ctd) {
      //m_type = std::string(_fd.m_type);
      //m_pData = new T(*(_fd.m_pData));
      setType(_ctd.getType());
      setData(_ctd.m_pData);
      setID(_ctd.m_id);
      return *this;
    }



  protected:
    ///The id of the data, used to locate it in working memory
    std::string m_id;

  };


} //namespace cast


#endif
