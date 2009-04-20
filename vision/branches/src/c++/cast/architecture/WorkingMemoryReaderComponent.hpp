
/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes, Henrik Jacobsson
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

#ifndef CAST_WORKING_MEMORY_READER_COMPONENT_H_
#define CAST_WORKING_MEMORY_READER_COMPONENT_H_

#include <WorkingMemoryWriterComponent.hpp>
#include <WorkingMemoryChangeReceiver.hpp>
#include <WorkingMemoryChangeFilterMap.hpp>
#include <CASTData.hpp>


#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <IceUtil/Thread.h> 



namespace cast {

  
  //fwd declarations
  class WorkingMemoryReaderComponent;
  
  /**
   * Class used to propagate change information to a
   * derived class in a separate thread.
   * 
   * @author nah
   */
  class WorkingMemoryChangeThread : 
    public IceUtil::Thread {
    
  public:
    
    /**
     * Contruct a new change thread, using _pWMRP as the component to write
     * change events to.
     *
     * @param _pWMRP The WorkingMemoryReaderComponent object to write
     * change events to.
     */
    WorkingMemoryChangeThread(WorkingMemoryReaderComponent * _pWMRP);
    
    virtual 
    void 
    run();

    
    
    /**
     * Stop the thread running. After this is called events can no
     * longer be queued.
     */
    void stop();

    /**
     * Add a list of working memory change structs to the queue qaiting
     * to be passed into the WorkingMemoryReaderComponent. The change
     * structs are passed into the * component in the order they appear in
     * the input list.
     *
     * @param _pChangeList A pointer to a sequence of
     * WorkingMemoryChange structs.
     */
    void queueChange(const cdl::WorkingMemoryChange & _change);


    /**
     * Makes the choice to use event objects or method calls.
     */
    bool forwardToSubclass(std::list<cdl::WorkingMemoryChange> & _wmcl);

  public:

    /**
     * Destructor. Hidden to prevent the thread being destroyed.
     */
    virtual ~WorkingMemoryChangeThread();


    /**
     * Wait for the WorkingMemoryReaderComponent's semaphore to be free,
     * then call the workingMemoryChanged method from that object. New
     * change events received whilst the component's semaphore is locked
     * are queue. This locks the semaphore whilst writing.
     */
    void runQueue();

    /**
     * Wait for the WorkingMemoryReaderComponent's semaphore to be free,
     * then call the workingMemoryChanged method from that object. New
     * change events received whilst the component's semaphore is locked
     * are discarded.  This locks the semaphore whilst writing.
     */
    void runDiscard();


  private:

    /**
     * Goes through m_receiversToRemove in the component and removes
     * as required.
     */
    inline void removeChangeFilters() const;

    /**
     * The list of change structs ready to be written to the component.
     */
    std::list<cdl::WorkingMemoryChange>  m_changeList;
  
    ///Controls access to m_changeList
    IceUtil::Mutex m_queueAccess;
    ///Whether the thread should do anthing
    bool m_bRun;

    ///The component that this object writes change events into.
    WorkingMemoryReaderComponent * m_pWMRP;


    IceUtil::Monitor<IceUtil::Mutex> m_changeMonitor;

    //temp used for comparisons
    cdl::WorkingMemoryChangeFilter m_tmpFilter;
    
    //temp used to store pointers to receivers
    std::vector<WorkingMemoryChangeReceiver *> m_receivers;
    
  };
  



  /**
   * Defines a class of component that can read from a working memory. This
   * component is based largely on receiving change events that inform the
   * component about what can be read. These change events can be filtered.
   * When no filters are added all events are allowed through. When
   * filters are added, only the events matched by the filters are allowed
   * through.
   * 
   * @author nah
   */
  class WorkingMemoryReaderComponent :
    public WorkingMemoryWriterComponent,
    virtual public interfaces::WorkingMemoryReaderComponent
  {
    
  private:

    //temp vector to store receivers scheduled for removal
    std::vector<const WorkingMemoryChangeReceiver *> m_receiversToRemove;
    
    
    bool m_bReceivingChanges;
    
    /**
     * Flag to determine whether this component should receive change
     * events from other subarchitectures.
     */
    bool m_receiveXarchChangeNotifications;

    /**
     * Does the removing work
     */
    void removeChangeFilterHelper(const WorkingMemoryChangeReceiver * _receiver);

    /**
     * The thread that is used to forward change events to the derived
     * class.
     */
    WorkingMemoryChangeThread * m_pWMChangeThread;    
    IceUtil::ThreadControl m_pWMChangeThreadControl;


    /**
     * Used to determine whether change events are queued or discarded.
     * Default is to discard them.
     */
    cdl::WorkingMemoryChangeQueueBehaviour m_queueBehaviour;

    /**
     * The set of filters that are applied to determine whether to
     * forward change events to the derived class or not.
     */
    WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver *> * m_pChangeObjects;    
    
    IceUtil::Monitor<IceUtil::Mutex> m_wmcMonitor;

  public:

    /**
     * Constructor.
     *
     * @param _id The unique identifier of this component.
     */
    WorkingMemoryReaderComponent();  

    /**
     * Destructor.
     */
    virtual ~WorkingMemoryReaderComponent();


    /**
     * Start this component running. This overridden method also starts
     * the encapsulated thread that forwards change information.
     * 
     */
    virtual void startInternal();

    virtual void stopInternal();

    ///Friend declaration for change thread.
    friend class WorkingMemoryChangeThread;


  public:

    /**
     * Removes the given change filter from the set of filters
     * receiving change events. To ensure thread safety the removal
     * does not happen immediately. If the receiver object should be
     * deleted after remove (i.e. "delete _receiver") supply
     * DELETE_RECEIVER as the additional argument.
     *
     */
    void removeChangeFilter(const WorkingMemoryChangeReceiver * _receiver, 
			    const cdl::ReceiverDeleteCondition & _condition = cdl::DONOTDELETERECEIVER);
 
    /**
     * Set filters to receive no changes at all.
     */
    virtual 
    void receiveNoChanges();

    /**
     * Set filters to receive changes using filters.
     */
    virtual void receiveChanges();
  

    /**
     * Determine whether this component should receive change events
     * from other subarchitectures.
     */
    bool isReceivingXarchChangeNotifications();

    /**
     * Determine whether this component should receive change events
     * from other subarchitectures.
     * 
     * @param The
     *            new value.
     */
    void setReceiveXarchChangeNotifications(bool _receiveXarchChangeNotifications);
  
    int getFilterCount() const {
      if(m_pChangeObjects) {
	return m_pChangeObjects->size();
      }
      else {
	return 0;
      }
    }


    /**
     * Get the entry from working memory with the given id. Returned
     * in stored format. Use getWorkingMemoryEntry<T> to get it in
     * CASTData in required format.
     * 
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     * 
     */
    virtual
    cdl::WorkingMemoryEntryPtr
    getBaseMemoryEntry(const std::string & _id) 
      throw (DoesNotExistOnWMException) {
      return getBaseMemoryEntry(_id, getSubarchitectureID());
    }

    /**
     * Get the entry from working memory with the given id. Returned
     * in stored format. Use getWorkingMemoryEntry<T> to get it in
     * CASTData in required format.
     * 
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     * 
     */
    virtual
    cdl::WorkingMemoryEntryPtr
    getBaseMemoryEntry(const cdl::WorkingMemoryAddress & _wma) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return getBaseMemoryEntry(_wma.id, _wma.subarchitecture);
    }

    /**
     * Get the entry from working memory with the given id. Returned
     * in stored format. Use getWorkingMemoryEntry<T> to get it in
     * CASTData in required format.
     * 
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     * 
     */
    virtual
    cdl::WorkingMemoryEntryPtr
    getBaseMemoryEntry(const std::string & _id, 
		       const std::string & _subarch) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      assert(!_id.empty());
      assert(m_workingMemory);
      cdl::WorkingMemoryEntryPtr entry(m_workingMemory->getWorkingMemoryEntry(_id, _subarch, getComponentID()));
      //println("got entry");
      updateVersion(entry->id, entry->version);
      //println("updated ids");
      return entry;
    }


    template <class T>
    IceInternal::Handle<T>
    getMemoryEntry(const std::string & _id) 
      throw (DoesNotExistOnWMException) {
      return getMemoryEntry<T>(_id,getSubarchitectureID());
    }


    template <class T>
    IceInternal::Handle<T>
    getMemoryEntry(const cdl::WorkingMemoryAddress & _wma) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return getMemoryEntry<T>(_wma.id, _wma.subarchitecture);
    }


    template <class T>
    IceInternal::Handle<T>
    getMemoryEntry(const std::string & _id, 
		   const std::string & _subarch) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return IceInternal::Handle<T>::dynamicCast(getBaseMemoryEntry(_id,_subarch)->entry);
    }


    template <class T>
    CASTData<T>
    getMemoryEntryWithData(const std::string & _id) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return getMemoryEntryWithData<T>(_id,getSubarchitectureID());
    }

    
    template <class T>
    CASTData<T>
    getMemoryEntryWithData(const cdl::WorkingMemoryAddress & _wma) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return getMemoryEntryWithData<T>(_wma.id, _wma.subarchitecture);
    }
    
    template <class T>
    CASTData<T>
    getMemoryEntryWithData(const std::string & _id,
			   const std::string & _subarch) 
      throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return CASTData<T>(getBaseMemoryEntry(_id,_subarch));
    }




    /**
     * Get the entry from working memory with the given
     * id. DEPRECATED: use getMemory* equivalent. Boost dependencies
     * will be phased out.
     * 
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     * 
     */
    template <class T>
    boost::shared_ptr< CASTData<T> > 
    getWorkingMemoryEntry(const std::string & _id)  __attribute__ ((deprecated));

    /**
     * Get the entry from working memory with the given
     * id. DEPRECATED: use getMemory* equivalent. Boost dependencies
     * will be phased out.
     * 
     * @param _wma
     *            The address for the entry in working memory.
     * @return The requested entry.
     * 
     */
    template <class T>
    boost::shared_ptr< CASTData<T> > 
    getWorkingMemoryEntry(const cdl::WorkingMemoryAddress & _wma)  __attribute__ ((deprecated));


    /**
     * Get the entry from working memory in a particular subarchitecture
     * with the given id.
     * 
     * @param _subarch
     *            The subarchitecture in which the id is located.
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     *
     * @throws DoesNotExistOnWMException if the entry does not exist.
     */
    template <class T>
    boost::shared_ptr< CASTData<T> > 
    getWorkingMemoryEntry(const std::string & _id, const std::string & _subarch)  __attribute__ ((deprecated));


    
    template <class T>
    void
    getBaseMemoryEntries(cdl::WorkingMemoryEntrySeq & _entries,
			 const unsigned int _count = 0) {
      getBaseMemoryEntries<T>(_entries,getSubarchitectureID(), _count);
    }

    template <class T>
    void
    getBaseMemoryEntries(cdl::WorkingMemoryEntrySeq & _entries,
			 const std::string & _subarch,
			 const unsigned int _count = 0) 
      throw(UnknownSubarchitectureException) {
      assert(!_subarch.empty());//subarch must not be empty
      const std::string & type(typeName<T>());
      
      m_workingMemory->getWorkingMemoryEntries(type,_subarch,_count,getComponentID(), _entries);

      for(cdl::WorkingMemoryEntrySeq::const_iterator i = _entries.begin();
	  i < _entries.end(); ++i) {
	updateVersion((*i)->id, (*i)->version);
      }      
    }

    template <class T>
    void
    getMemoryEntries(std::vector< IceInternal::Handle<T> > & _entries,
		     const unsigned int _count = 0) {
      getMemoryEntries<T>(_entries,getSubarchitectureID(), _count);
    }

    template <class T>
    void
    getMemoryEntries(std::vector< IceInternal::Handle<T> > & _entries,
		     const std::string & _subarch,
		     const unsigned int _count = 0) 
      throw (UnknownSubarchitectureException) {
      assert(!_subarch.empty());//subarch must not be empty

      //get base entries
      cdl::WorkingMemoryEntrySeq entries;
      getBaseMemoryEntries<T>(entries,_subarch,_count);

      //add cast result to other vector
      for(cdl::WorkingMemoryEntrySeq::const_iterator i = entries.begin();
	  i < entries.end(); ++i) {
	_entries.push_back(IceInternal::Handle<T>::dynamicCast((*i)->entry));
      }      
    }

    template <class T>
    void
    getMemoryEntriesWithData(std::vector< CASTData<T> > & _entries,
			     const unsigned int _count = 0) {
      getMemoryEntriesWithData<T>(_entries,getSubarchitectureID(), _count);
    }

    template <class T>
    void
    getMemoryEntriesWithData(std::vector< CASTData<T> > & _entries,
			     const std::string & _subarch,
			     const unsigned int _count = 0) 
      throw (UnknownSubarchitectureException) {
      assert(!_subarch.empty());//subarch must not be empty

      //get base entries
      cdl::WorkingMemoryEntrySeq entries;
      getBaseMemoryEntries<T>(entries,_subarch,_count);

      //add cast result to other vector
      for(cdl::WorkingMemoryEntrySeq::const_iterator i = entries.begin();
	  i < entries.end(); ++i) {
	_entries.push_back(CASTData<T>(*i));
      }      
    }


    //     virtual 
    //     void
    //     runComponent() {
    //       using namespace cdl;
    //       using namespace std;
    //       using namespace boost;

    //       WorkingMemoryEntryPtr wme(getBaseMemoryEntry("asd"));
    //       TestStructStringPtr tss(getMemoryEntry<TestStructString>("asd"));
    //       CASTData<TestStructString> ctd(getMemoryEntryWithData<TestStructString>("asd"));
    //       boost::shared_ptr<const CASTData<TestStructString> > shrd(getWorkingMemoryEntry<TestStructString>("asd"));

    //       WorkingMemoryEntrySeq entries;
    //       getBaseMemoryEntries<TestStructString>(entries);
    //       vector<TestStructStringPtr> entries2;
    //       getMemoryEntries(entries2);

    //       vector< CASTData<TestStructString> > entries3;
    //       getMemoryEntriesWithData(entries3);

    //       vector< shared_ptr<const CASTData<TestStructString> > > entries4;
    //       getWorkingMemoryEntries(entries4);

    //     }

    

    /**
     * Retrieve the working memory entries matching the given type from
     * the local subarchitecture working memory.
     * 
     * @param _count
     *            The number of entries to return. A value of 0 means
     *            all matching entries. 
     * @param _results A sequence of matching entries starting with the most
     *         recent. Must not be null initially.
     * 
     */
    template <class T>
    void  getWorkingMemoryEntries(std::vector < boost::shared_ptr< CASTData<T> > > & _results)   __attribute__ ((deprecated));

    /**
     * Retrieve the working memory entries matching the given type from
     * the local subarchitecture working memory.
     * 
     * @param _count
     *            The number of entries to return. A value of 0 means
     *            all matching entries. 
     * @param _results A sequence of matching entries starting with the most
     *         recent. Must not be null initially.
     * 
     */
    template <class T>
    void  getWorkingMemoryEntries(const int & _count,
				  std::vector < boost::shared_ptr< CASTData<T> > > & _results)  __attribute__ ((deprecated));

    /**
     * Retrieve the working memory entries matching the given type from
     * the specified subarchitecture working memory.
     * 
     * @param _subarch
     *            The subarchitecture which should be used to locate the
     *            working memory containing the entries.
     * @param _count
     *            The number of entries to return. A value of 0 means
     *            all matching entries.
     * 
     * @param _results A sequence of matching entries starting with the most
     *         recent. Must not be null initially.
     */
    template <class T>
    void  getWorkingMemoryEntries(const std::string & _subarch,				  
				  const int & _count,
				  std::vector < boost::shared_ptr< CASTData<T> > > & _results)  __attribute__ ((deprecated));

    /**
     * Add a new filter object to receive the change events.  See
     * ChangeFilterFactory for functions for creating filters.
     *
     * @param _filter The filter to match for the receiver
     * @param _receiver
     *            The receiver object
     */
    void addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter,  
			 WorkingMemoryChangeReceiver * _pReceiver);


    void receiveChangeEvent(const cdl::WorkingMemoryChange& wmc, 
			    const Ice::Current & _ctx);

    /**
     * This method sleeps until workingMemoryChanged has returned after
     * writing at least one change event. 
     *
     */
    void waitForChanges();
  
  };

} //namespace cast


//the follwing done out here to allow for use of __attribute__ spec above

template <class T>
boost::shared_ptr< cast::CASTData<T> > 
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntry(const std::string & _id)  {  
  return getWorkingMemoryEntry<T>(_id,getSubarchitectureID());
}

template <class T>
boost::shared_ptr< cast::CASTData<T> > 
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntry(const cdl::WorkingMemoryAddress & _wma)  {  
  return getWorkingMemoryEntry<T>(_wma.id,_wma.subarchitecture);
}

template <class T>
boost::shared_ptr< cast::CASTData<T> > 
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntry(const std::string & _id,
							  const std::string & _subarch)  {  
  return boost::shared_ptr<cast::CASTData<T> >(new cast::CASTData<T>(getBaseMemoryEntry(_id, _subarch)));
}


template <class T>
void  
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntries(std::vector < boost::shared_ptr< CASTData<T> > > & _results) {
  return getWorkingMemoryEntries<T>(0,_results);
}

template <class T>
void
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntries(const int & _count,
							    std::vector < boost::shared_ptr< CASTData<T> > > & _results) {
  return getWorkingMemoryEntries<T>(m_subarchitectureID,_count,_results);
}



template <class T>
void  
cast::WorkingMemoryReaderComponent::getWorkingMemoryEntries(const std::string & _subarch,				  
							    const int & _count,
							    std::vector < boost::shared_ptr< CASTData<T> > > & _results) {
  
  assert(!_subarch.empty());//subarch must not be empty

  //get base entries
  cdl::WorkingMemoryEntrySeq entries;
  getBaseMemoryEntries<T>(entries,_subarch,_count);
  
  //add cast result to other vector
  for(cdl::WorkingMemoryEntrySeq::const_iterator i = entries.begin();
      i < entries.end(); ++i) {
    _results.push_back(boost::shared_ptr<CASTData<T> >(new CASTData<T>(*i)));
  }      

  
}




#endif
