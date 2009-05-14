#include "CASTWorkingMemoryItem.hpp"

namespace cast {

  CASTWorkingMemoryItem::CASTWorkingMemoryItem() {}
  CASTWorkingMemoryItem::~CASTWorkingMemoryItem(){}
  int CASTWorkingMemoryItem::getVersion() {throw(CASTException(__HERE__, "should be overidden in subclass"));}
  void CASTWorkingMemoryItem::setVersion(const int & _overwriteCount) {throw(CASTException(__HERE__, "should be overidden in subclass"));}


  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem() : CASTWorkingMemoryItem(), CASTData<CORBA::Any>() {
  }

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
  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem(const std::string & _id, 
							       const std::string & _type, 
							       CORBA::Any * _pData) : 
  CASTWorkingMemoryItem(), 
    CASTData<CORBA::Any>(_id,_type,0,_pData) {
  }
    /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    int _version,
		    CORBA::Any * _pData) : 
    CASTWorkingMemoryItem(), 
    CASTData<CORBA::Any>(_id,_type,_version,_pData) {
  }

  /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    int _version,
		    const CORBA::Any & _data) : 
    CASTWorkingMemoryItem(), 
    CASTData<CORBA::Any>(_id,_type,_version, _data) {
  }

    /**
   * Construct a new object with an ontological type and a data
   * object.  This constructor uses existing _data.
   *
   * @param _type The ontological type of the data.
   * @param _pData A pointer to the data itself.
   */
  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem(const std::string & _id, 
		    const std::string & _type, 
		    const CORBA::Any & _data) : 
    CASTWorkingMemoryItem(), 
    CASTData<CORBA::Any>(_id,_type,0, _data) {
  }

  /**
   * Create a new data object by deep copying the input.
   */
  WorkingMemoryItem<CORBA::Any>::WorkingMemoryItem(const WorkingMemoryItem<CORBA::Any> & _fd) : 
    CASTWorkingMemoryItem(), 
    CASTData<CORBA::Any>(_fd){
  }

  /**
   * Destructor. This deletes the memory used by the data object. To
   * prevent the stored data from being deleted when the wrapping
   * object is, set the data pointer to
   * null. i.e. pMyWorkingMemoryItem->data() == NULL.
   */
  WorkingMemoryItem<CORBA::Any>::~WorkingMemoryItem() {
  }

  const std::string & WorkingMemoryItem<CORBA::Any>::getType() const {
    return CASTData<CORBA::Any>::getType();
  }

  int WorkingMemoryItem<CORBA::Any>::getVersion() {
    return CASTData<CORBA::Any>::getVersion();
  }

  void WorkingMemoryItem<CORBA::Any>::setVersion(const int & _overwriteCount) {
    CASTData<CORBA::Any>::setVersion(_overwriteCount);
  }


  bool WorkingMemoryItem<CORBA::Any>::isLocal() const {
    return false;
  }

  
  void WorkingMemoryItem<CORBA::Any>::toAny(CORBA::Any &_any) const {
    //deep copy
    _any = *(getData());
  }

  CASTWorkingMemoryItem * WorkingMemoryItem<CORBA::Any>::clone() {
    return new WorkingMemoryItem<CORBA::Any>(*this);
  }


}
