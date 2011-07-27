#ifndef WM_VIEW_HPP
#define WM_VIEW_HPP

#include "ChangeHandler.hpp"

#include <cast/architecture.hpp>

#include <map>

namespace castutils {
  
  
  typedef IceInternal::Handle<cast::WorkingMemoryReaderComponent> WorkingMemoryReaderComponentPtr;
  
  template <class StoredType>
  class WMView : public IceUtil::Shared {
    
  public:
    
    
    
    typedef IceInternal::Handle<StoredType> StoredTypePtr;  
    typedef std::pair<cast::cdl::WorkingMemoryAddress, StoredTypePtr > WMViewPair;
    typedef std::map< cast::cdl::WorkingMemoryAddress, StoredTypePtr > WMViewMap;
    
    typedef IceUtil::Handle< ChangeHandler<StoredType> > ChangeHandlerPtr;
    typedef IceUtil::Handle< WMView<StoredType> > WMViewPtr;  
    
    typedef typename WMViewMap::iterator iterator;
    typedef typename WMViewMap::const_iterator const_iterator;
    
    WMView(WorkingMemoryReaderComponentPtr _component):
    m_component(_component) {}
    
    virtual ~WMView(){}
    
    ///Must be called to activate view
    void start() {
      
      m_component->addChangeFilter(cast::createGlobalTypeFilter<StoredType>(cast::cdl::ADD), 
                                   new cast::MemberFunctionChangeReceiver<WMView>(this,
                                                                                  &WMView::entryAdded));
      m_component->addChangeFilter(cast::createGlobalTypeFilter<StoredType>(cast::cdl::OVERWRITE), 
                                   new cast::MemberFunctionChangeReceiver<WMView>(this,
                                                                                  &WMView::entryOverwritten));
      m_component->addChangeFilter(cast::createGlobalTypeFilter<StoredType>(cast::cdl::DELETE), 
                                   new cast::MemberFunctionChangeReceiver<WMView>(this,
                                                                                  &WMView::entryDeleted));
    }
    
    /**
     * Handle a new entry of the given type.
     */
    virtual void entryAdded(const cast::cdl::WorkingMemoryChange & _wmc) {
      assert(m_map.end() == m_map.find(_wmc.address));
      try {
        StoredTypePtr entry = m_component->getMemoryEntry<StoredType>(_wmc.address);
        m_map.insert(WMViewPair(_wmc.address, entry));
        dispatchChangeEvent(_wmc, entry, StoredTypePtr());
      }
      catch (const cast::DoesNotExistOnWMException & e) {
        //if it was deleted before it could be added then that's no problem to us
      }
    }
    
    /**
     * Handle an overwrite of the given type.
     */
    virtual void entryOverwritten(const cast::cdl::WorkingMemoryChange & _wmc) {
      typename WMViewMap::iterator i = m_map.find(_wmc.address);
      assert(m_map.end() != i);          
      try {
        StoredTypePtr newEntry = m_component->getMemoryEntry<StoredType>(_wmc.address);
        StoredTypePtr oldEntry = i->second;
        i->second = newEntry;     
        dispatchChangeEvent(_wmc, newEntry, oldEntry);
      }
      catch (const cast::DoesNotExistOnWMException & e) {
        //if it was deleted before it could be read, then remove from map
        m_map.erase(i);
      }
    }
    
    virtual void entryDeleted(const cast::cdl::WorkingMemoryChange & _wmc) {
      
      typename WMViewMap::iterator i = m_map.find(_wmc.address);
      
      //it could already be gone if we got an exception earlier
      if(m_map.end() != i) {        
        StoredTypePtr oldEntry = i->second;
        m_map.erase(i);
        dispatchChangeEvent(_wmc, StoredTypePtr(), oldEntry);
      }
      else {
        dispatchChangeEvent(_wmc, StoredTypePtr(), StoredTypePtr());
        
      }
      
    }
    
    typename WMViewMap::const_iterator find(const cast::cdl::WorkingMemoryAddress _wma) const {
      return m_map.find(_wma);
    }
    
    typename WMViewMap::const_iterator begin() const {
      return m_map.begin();
    }
    
    typename WMViewMap::const_iterator end() const {
      return m_map.end();
    }
    
    
    void registerChangeHandler(ChangeHandlerPtr _handler) {
      m_changeHandlers.push_back(_handler);
    }
    
  private:
    
    void dispatchChangeEvent(const cast::cdl::WorkingMemoryChange & _wmc,  
                             StoredTypePtr _newEntry, StoredTypePtr _oldEntry) {
      for (typename std::list< ChangeHandlerPtr >::iterator i = m_changeHandlers.begin();
           i != m_changeHandlers.end(); ++i) {
        (*i)->entryChanged(*this, _wmc, _newEntry, _oldEntry);
      } 
    }
    
    WMViewMap m_map;
    WorkingMemoryReaderComponentPtr m_component;
    std::list< ChangeHandlerPtr  > m_changeHandlers;
  };
  
  
};

#endif
