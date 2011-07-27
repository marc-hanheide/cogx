#ifndef WM_VIEW_CHANGE_HANDLER_HPP
#define WM_VIEW_CHANGE_HANDLER_HPP

#include <cast/architecture.hpp>


namespace castutils {

  //fwd reference
  template <class StoredType>
  class WMView;

  
  /**
   * Change handler for use by WMView when events are received.
   */
  template <class StoredType>
  class ChangeHandler : public IceUtil::Shared {
    
    
  public:
    typedef IceInternal::Handle<StoredType> StoredTypePtr;
    
    ChangeHandler() {};
    virtual ~ChangeHandler() {};
    virtual void entryChanged(const WMView<StoredType> & _map,
                              const cast::cdl::WorkingMemoryChange & _wmc,  
                              StoredTypePtr _newEntry, StoredTypePtr _oldEntry) 
    throw (cast::CASTException) = 0;

  };
  

};

#endif