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
 
 
#ifndef CAST_CAST_WORKING_MEMORY_ITEM_H_
#define CAST_CAST_WORKING_MEMORY_ITEM_H_

#include "CASTData.hpp"
#include "CASTException.hpp"

#include <omniORB4/CORBA.h>
#include <cast/cdl/CAST.hh>
#include <boost/shared_ptr.hpp>
#include <balt/interface/RemoteDataTranslator.hpp>
#include <balt/interface/RemoteTranslatorManager.hpp>


/**
 * Class used for working memory storage, This is a fairly empty
 * super-class used to combine data from local (same language and
 * machine) and remote (all other) sources.
 */

namespace cast {

class CASTWorkingMemoryItem {

public:
  CASTWorkingMemoryItem();
  virtual ~CASTWorkingMemoryItem();
  virtual const std::string & getType() const = 0;
  virtual bool isLocal() const = 0;
  virtual void toAny(CORBA::Any &_any) const  = 0;
  virtual CASTWorkingMemoryItem * clone() = 0;

  virtual int getVersion();
  virtual void setVersion(const int & _version);

};

template <class T>
class WorkingMemoryItem: 
  public CASTWorkingMemoryItem,
  public CASTData<T>
{

public:

   /**
   * Construct a new empty data object.
   */ 
  WorkingMemoryItem() : CASTWorkingMemoryItem(), CASTData<T>() {
  }

/*   /\** */
/*    * Construct a new object with an ontological type and no data. */
/*    *    */
/*    * @param _type The ontological type of the data. */
/*    * @param _data The data itself. */
/*    *\/ */
/*   WorkingMemoryItem(const std::string & _type) : CASTData<T>(_type) { */

/*   } */

/*   /\** */
/*    * Construct a new object with an ontological type and a data */
/*    * object. This constructor creates a new copy of _data. */
/*    *  */
/*    * @param _type The ontological type of the data. */
/*    * @param _data The data itself. */
/*    *\/ */
/*   WorkingMemoryItem(const std::string & _type, const T & _data) :  */
/*     CASTData<T>(_type,_data) { */
/*   } */
  
  /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id,
		    const std::string & _type, 
		    T * _pData) :
    CASTWorkingMemoryItem(), 
    CASTData<T>(_id,_type,0,_pData) {
  }
  

    /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id,
		    const std::string & _type, 
		    int _version,
		    T * _pData) :
    CASTWorkingMemoryItem(), 
    CASTData<T>(_id,_type,_version,_pData) {
  }

/**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id,
		    const std::string & _type, 
		    const T & _data) :
    CASTWorkingMemoryItem(), 
    CASTData<T>(_id,_type,0,_data) {
  }

  /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id,
		    const std::string & _type, 
		    int _version,
		    const T & _data) :
    CASTWorkingMemoryItem(), 
    CASTData<T>(_id,_type,_version,_data) {
  }
  
  /**
   * Create a new data object by deep copying the input.
   */
  WorkingMemoryItem(const WorkingMemoryItem<T> & _fd) : 
    CASTWorkingMemoryItem(), 
    CASTData<T>(_fd){
  }
  
  /**
   * Destructor. This deletes the memory used by the data object. To
   * prevent the stored data from being deleted when the wrapping
   * object is, set the data pointer to
   * null. i.e. pMyWorkingMemoryItem->data() == NULL.
   */
  virtual ~WorkingMemoryItem() {
  }

  virtual int getVersion() {
    return CASTData<T>::getVersion();
  }
  virtual void setVersion(const int & _version) {
    CASTData<T>::setVersion(_version);
  }


  const std::string & getType() const {
    return CASTData<T>::getType();
  }

  virtual bool isLocal() const {
    return true;
  }

  virtual void toAny(CORBA::Any &_any) const {
    
    if(!this->getData()) { 
      throw CASTException(__HERE__,  
 			  "data pointer is null for translation to any"); 
      
    } 
    
    std::string datatype = getType();
    const RemoteDataTranslator<T> * 
      pTranslator = 
      RemoteTranslatorManager::translator<T>();
    
    if(pTranslator == NULL) { 
      throw CASTException(__HERE__,  
 			  "Translator pointer is null for translation to any. Data type = %s", 
			  getType().c_str());       
    } 

    //We need to copy here as the translation process is consuming
    //make the copy

    //cout<<"got things"<<endl;
    T* pData = new T(*(this->getData()));
    //cout<<"created new"<<endl;
    pTranslator->translate(pData,_any);
    //cout<<"translated"<<endl;  
}

  virtual CASTWorkingMemoryItem * clone() {
    return new WorkingMemoryItem<T>(*this);
  }

};


template <>
class WorkingMemoryItem<CORBA::Any>: 
  public CASTWorkingMemoryItem,
  public CASTData<CORBA::Any>
{
public:
   /**
   * Construct a new empty data object.
   */ 
  WorkingMemoryItem();

/*   /\** */
/*    * Construct a new object with an ontological type and no data. */
/*    *    */
/*    * @param _type The ontological type of the data. */
/*    * @param _data The data itself. */
/*    *\/ */
/*   WorkingMemoryItem(const std::string & _type) : CASTData<CORBA::Any>(_type) { */

/*   } */

/*   /\** */
/*    * Construct a new object with an ontological type and a data */
/*    * object. This constructor creates a new copy of _data. */
/*    *  */
/*    * @param _type The ontological type of the data. */
/*    * @param _data The data itself. */
/*    *\/ */
/*   WorkingMemoryItem(const std::string & _type, const CORBA::Any & _data) : CASTData<CORBA::Any>(_type,_data) { */
/*   } */
  
  /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    CORBA::Any * _pData);
    /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    int _version,
		    CORBA::Any * _pData);
  /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    int _version,
		    const CORBA::Any & _data);
    /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    const CORBA::Any & _data);
  /**
   * Create a new data object by deep copying the input.
   */
  WorkingMemoryItem(const WorkingMemoryItem<CORBA::Any> & _fd);
  /**
   * Destructor. This deletes the memory used by the data object. To
   * prevent the stored data from being deleted when the wrapping
   * object is, set the data pointer to
   * null. i.e. pMyWorkingMemoryItem->data() == NULL.
   */
  virtual ~WorkingMemoryItem();
  const std::string & getType() const;
  virtual int getVersion();
  virtual void setVersion(const int & _version);
  virtual bool isLocal() const;
  virtual void toAny(CORBA::Any &_any) const;
  virtual CASTWorkingMemoryItem * clone();
};


/**
 * The class used to pass around CASTWorkingMemoryItems for adding to
 * wms. These are the objects that actually own the memory of the
 * CASTWorkingMemoryItems. This is done is done via a boost shared_ptr
 * so that even if the original working memory entry is overwritten,
 * the data is not lost to objects that have CASTWorkingMemoryEntry
 * objects or CASTData objects.
 */

class CASTWorkingMemoryEntry {

public:


  /**
   * @param _pItem The working memory item for this entry. This class
   * assumes ownership.
   */
  CASTWorkingMemoryEntry(const std::string & _src,
			 const cdl::WorkingMemoryOperation & _operation,
			 const cdl::WorkingMemoryAddress & _address,
			 CASTWorkingMemoryItem * _pItem) : 
    m_operation(_operation),
    m_address(_address),
    m_pItem(_pItem), //shared ptr owns the input memory
    m_src(_src)
  {}

  /**
   * @param _pItem The working memory item for this entry. Shares
   * ownership with original pointer.
   */
  CASTWorkingMemoryEntry(const std::string & _src,
			 const cdl::WorkingMemoryOperation & _operation,
			 const cdl::WorkingMemoryAddress & _address,
			 boost::shared_ptr<CASTWorkingMemoryItem> _pItem) : 
    m_operation(_operation),
    m_address(_address),
    m_pItem(_pItem), //shared ptr shares the input memory
    m_src(_src)
  {}

  CASTWorkingMemoryEntry(const CASTWorkingMemoryEntry &_wme) : 
    m_operation(_wme.m_operation),
    m_address(_wme.m_address),
    m_pItem(_wme.m_pItem->clone()), //shared ptr owns the cloned mem
    m_src(_wme.m_src) {
    //cout<<"clone copy!"<<endl;
  }  

    /**
   * Shallow assignment.
   */
  CASTWorkingMemoryEntry & operator=(const CASTWorkingMemoryEntry & _wme) {
    m_src = _wme.m_src;
    m_operation = _wme.m_operation;
    m_address = _wme.m_address;
    //creates and extra reference on the shared ptr
    m_pItem = _wme.m_pItem;
    return *this;
  }


  virtual ~CASTWorkingMemoryEntry() {
  }  

  virtual const cdl::WorkingMemoryOperation & getOperation() const {
    return m_operation;
  }

  virtual const cdl::WorkingMemoryAddress & getAddress() const {
    return m_address;
  }

  virtual const std::string & getSource() const {
    return m_src;
  }

  /**
   * Get the item object.
   * @return The item object.
   */
  const CASTWorkingMemoryItem & getItem() const {
    return (*m_pItem);
  }
  
  /**
   * Get the internal item pointer. Can be used to directly manipulate
   * the wrapped item.
   *
   * @return The object's pointer to the item object.
   */
  boost::shared_ptr<CASTWorkingMemoryItem> & item() {
    return m_pItem;
  }


protected:

  cdl::WorkingMemoryOperation m_operation;
  cdl::WorkingMemoryAddress m_address;
  boost::shared_ptr<CASTWorkingMemoryItem> m_pItem;
  std::string m_src;
};

}//noamespace cast


#endif
