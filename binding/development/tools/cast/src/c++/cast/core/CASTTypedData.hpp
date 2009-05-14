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

#ifndef CAST_CAST_TYPED_DATA_H_
#define CAST_CAST_TYPED_DATA_H_

#include <string>
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>


namespace cast {

  /**
   * A simple class for wrapping up an object with an ontological
   * type. The wrapped data can only be accessed by a const pointer to
   * protected it from deletion when multiple components are accessing
   * it. The object can be constructed in two ways: m_hasDeletePriviledge =
   * true (default) means that the wrapped data is deleted on object
   * deletion; m_hasDeletePriviledge = false means that the wrapped data is
   * not deleted on object deletion. The latter case is used when data
   * is shared locally amongst many components. The former case is used
   * when data is copied across machine or language barriers.
   * 
   * @author nah
   */
  template <class T>
  class CASTTypedData {

  public:

  
    /**
     * Construct a new empty data object.
     */ 
    CASTTypedData() : m_pData(){    
      m_type = "";
      m_version = 0;
    }

    /**
     * Construct a new object with an ontological type and no data.
     *   
     * @param _type The ontological type of the data.
     * @param _data The data itself.
     */
    CASTTypedData(const std::string & _type) :     m_pData() {
      m_type = std::string(_type);
      m_version = 0; 
   }

    /**
     * Construct a new object with an ontological type and a data
     * object. This constructor creates a new copy of _data.
     * 
     * @param _type The ontological type of the data.
     * @param _data The data itself.
     */
    CASTTypedData(const std::string & _type, 
		  const T & _data):
      m_pData(new T(_data)){
      m_type = std::string(_type);
      m_version = 0;
    }
  
    /**
     * Construct a new object with an ontological type and a data
     * object.  This constructor uses existing _data.
     * 
     * @param _type The ontological type of the data.
     * @param _pData A pointer to the data itself.
     */
    CASTTypedData(const std::string & _type, 
		  T * _pData):
      m_pData(_pData)
    {
      m_type = std::string(_type);
      m_version = 0;
    }


        /**
     * Construct a new object with an ontological type and a data
     * object. This constructor creates a new copy of _data.
     * 
     * @param _type The ontological type of the data.
     * @param _data The data itself.
     */
    CASTTypedData(const std::string & _type, 
		  const int & _version,
		  const T & _data):
      m_pData(new T(_data)){
      m_type = std::string(_type);
      m_version = _version;
    }
  
    /**
     * Construct a new object with an ontological type and a data
     * object.  This constructor uses existing _data.
     * 
     * @param _type The ontological type of the data.
     * @param _pData A pointer to the data itself.
     */
    CASTTypedData(const std::string & _type, 
		  const int & _version,
		  T * _pData):
      m_pData(_pData)
    {
      m_type = std::string(_type);
      m_version = _version;
    }

    /**
     * Create a new data object by deep copying the input.
     */
    CASTTypedData(const CASTTypedData<T> & _fd) : m_pData(new T(*(_fd.m_pData))) {
      m_type = std::string(_fd.m_type);
      m_version = _fd.m_version;
    }

    /**
     * Destructor. If the object was constructed with delete
     * priviledges, this deletes the memory used by the data object. To
     * prevent the stored data from being deleted when the wrapping
     * object is, set the data pointer to
     * null. i.e. pMyCASTTypedData->setData(NULL).
     */
    virtual ~CASTTypedData() {
    
    }
  

    //nah: removed, no longer safe with smart pointers... probably was never safe actully!
    //  /**
    // * Get the data object.
    // * @return The data object.
    // */
    //   const T & getData() const {
    //     if(!m_pData) {
    //       ostringstream outStream;
    //       outStream<<"ERROR: data pointer is null for typed data: "<<m_type<<endl;
    //       throw BALTException(__HERE__,
    // 			  outStream.str().c_str());

    //     }
    //     return (*m_pData);
    //   }


    /**
     * Get the internal data pointer. Can be used to directly manipulate
     * the wrapped data.
     *
     * @return The object's pointer to the data object.
     */
    const boost::shared_ptr<const T>  getData() const {
      //cout<<"in data(): "<<m_pData.use_count()<<endl;
      return m_pData;
    }
  


    /**
     * Get the ontological type of the data object.
     * @return The ontological type of the data.
     */
    const std::string & getType() const {
      return m_type;
    }
  
    /**
     * Set the data object. Copies the input as new stored data. Frees
     * memory of previously stored data if there is any.
     *
     * @param _data The new data object.
     */
    void setData(const T & _data) {
      m_pData = new T(_data);
    }

    /**
     * Set the internal data pointer to the input. Frees memory of
     * previously stored data if there is any.
     *
     * @param _pData Pointer to new data.
     */
    void setData(T * _pData) {
      m_pData = _pData;
    }

    void setData(boost::shared_ptr<T> _pData) {
      m_pData = _pData;
    }
  
    /**
     * Set the ontological type of the data.
     * @param _type The new ontological type.
     */
    void setType(const std::string & _type) {
      m_type = std::string(_type);
    }

    void protectData() {
      m_hasDeletePriviledge = false;
    }


    int getVersion() const {
      return m_version;
    }

    void setVersion(const int & _versionNumber) {
      m_version = _versionNumber;
    }
    /**
     * Shallow assignment.
     */
    CASTTypedData<T> & operator=(const CASTTypedData<T> & _ctd) {
      //m_type = std::string(_fd.m_type);
      //m_pData = new T(*(_fd.m_pData));
      setType(_ctd.m_type);
      setData(_ctd.m_pData);
      setVersion(_ctd.m_version);

      return *this;
    }
  
  private:
    ///the ontological type of the data
    std::string m_type;
    ///a pointer to the actual stored data
    boost::shared_ptr<T> m_pData;
    ///whether this object is allow to delete the stored data or not
    bool m_hasDeletePriviledge;
    ///the version number of this data
    int m_version;

  };

} //namespace cast

#endif
