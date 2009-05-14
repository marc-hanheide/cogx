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

#ifndef CAST_WORKING_MEMORY_READER_PROCESS_H_
#define CAST_WORKING_MEMORY_READER_PROCESS_H_

#include <cast/core/CASTCore.hpp>
#include <balt/interface/BALTInterface.hpp>

#include "cast/architecture/WorkingMemoryAttachedComponent.hpp"
#include "cast/architecture/SubarchitectureWorkingMemoryProtocol.hpp"
#include "cast/architecture/WorkingMemoryChangeReceiver.hpp"
#include "cast/architecture/WorkingMemoryChangeFilterMap.hpp"
#include "cast/architecture/DoesNotExistOnWMException.hpp"


#include <list>
#include <vector>

namespace cast {



  //fwd declarations
  class WorkingMemoryReaderProcess;

  /**
   * Class used to propagate change information to a
   * derived class in a separate thread.
   * 
   * @author nah
   */
  class WorkingMemoryChangeThread : public omni_thread {
    
  public:
    
    /**
     * Contruct a new change thread, using _pWMRP as the process to write
     * change events to.
     *
     * @param _pWMRP The WorkingMemoryReaderProcess object to write
     * change events to.
     */
    WorkingMemoryChangeThread(WorkingMemoryReaderProcess * _pWMRP);
    
    
    /**
     * Start the thread running. After this is called events can be
     * queued.
     */
    void start();
    
    
    /**
     * Stop the thread running. After this is called events can no
     * longer be queued.
     */
    void stop();

    /**
     * Add a list of working memory change structs to the queue qaiting
     * to be passed into the WorkingMemoryReaderProcess. The change
     * structs are passed into the * process in the order they appear in
     * the input list.
     *
     * @param _pChangeList A pointer to a sequence of
     * WorkingMemoryChange structs.
     */
    void queueChange(const cdl::WorkingMemoryChange & _change);


    /**
     * Makes the choice to use event objects or method calls.
     */
    bool forwardToSubclass(const cdl::WorkingMemoryChangeList & _wmcl);

  public:

    /**
     * Destructor. Hidden to prevent the thread being destroyed.
     */
    virtual ~WorkingMemoryChangeThread();

    /**
     * Run method of thread. This calls one of runQueue or runDiscard,
     * determined by the m_queueBehaviour variable of the
     * WorkingMemoryReaderProcess object.
     */
    virtual void* run_undetached(void *arg);

    /**
     * Wait for the WorkingMemoryReaderProcess's semaphore to be free,
     * then call the workingMemoryChanged method from that object. New
     * change events received whilst the process's semaphore is locked
     * are queue. This locks the semaphore whilst writing.
     */
    void runQueue();

    /**
     * Wait for the WorkingMemoryReaderProcess's semaphore to be free,
     * then call the workingMemoryChanged method from that object. New
     * change events received whilst the process's semaphore is locked
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
    std::list<cdl::WorkingMemoryChange> * m_pChangeList;
  
    ///Controls access to m_pChangeList
    omni_mutex m_queueAccess;
    ///Whether the thread should do anthing
    bool m_bRun;

    ///The process that this object writes change events into.
    WorkingMemoryReaderProcess * m_pWMRP;

    //omni_mutex m_changeConditionMutex;
    //omni_condition * m_pChangeCondition;

    omni_semaphore * m_pChangeSemaphore;

    //temp used for comparisons
    cdl::WorkingMemoryChangeFilter m_tmpFilter;
    
    //temp used to store pointers to receivers
    std::vector<WorkingMemoryChangeReceiver *> m_receivers;
    
  };




  /**
   * Defines a class of process that can read from a working memory. This
   * process is based largely on receiving change events that inform the
   * process about what can be read. These change events can be filtered.
   * When no filters are added all events are allowed through. When
   * filters are added, only the events matched by the filters are allowed
   * through.
   * 
   * @author nah
   */
  class WorkingMemoryReaderProcess :
    virtual public WorkingMemoryAttachedComponent, 
    public PushReceiver<cdl::WorkingMemoryChange>,
    public PushSender<cdl::WorkingMemoryChangeFilter >{
    
  private:

    void logReadEvent(const std::string & _subarch, const std::string & _type, int _count) {
      logEvent(cdl::ui::GET, getProcessIdentifier(), _subarch, _type, "");
    }
  
    void logReadEvent(const std::string & _subarch, const std::string & _id) {
      logEvent(cdl::ui::GET, getProcessIdentifier(), _subarch, "", _id);
    }

    void logChangeEvents(const unsigned int & _filteredCount, 
			 const unsigned int & _totalCount) {
      m_componentStatusMutex.lock();
      m_componentStatus.m_totalChangeEventsFiltered += _filteredCount;
      m_componentStatus.m_totalChangeEventsReceived += _totalCount;
      m_componentStatusMutex.unlock();
    }
  

  bool existsOnWorkingMemoryXarch(const std::string & _id, 
				  const std::string & _subarch);
  
    bool existsOnWorkingMemoryLocal(const std::string & _id);


    //temp vector to store receivers scheduled for removal
    std::vector<const WorkingMemoryChangeReceiver *> m_receiversToRemove;
    
    /**
     * Does the removing work
     */
    void removeChangeFilterHelper(const WorkingMemoryChangeReceiver * _receiver);

  public:

    /**
     * Constructor.
     *
     * @param _id The unique identifier of this process.
     */
    WorkingMemoryReaderProcess(const std::string &_id);  

    /**
     * Destructor.
     */
    virtual ~WorkingMemoryReaderProcess();

    /**
     * Receive raw change information from any attached working
     * memories.
     *
     * @param _pData The sequence of change events.
     */
    virtual void 
    receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData);
  


    /**
     * Set the push connector used by the component to register change
     * filters with working memory
     * 
     * @param _connectionID The id of the connector.  
     * @param _pOut The connector itself.
     * 
     */
    virtual void 
    setPushConnector(const std::string & _connectionID, 
		     PushConnectorOut<cdl::WorkingMemoryChangeFilter> * _pOut);


    /**
     * Friend function that is called to forward a change events to to
     * the subclass. This is currently coded as a separate function to
     * support experimenting with approaches to threading.
     *
     * @param _arg Void pointer to a ChangeSignalClass object. 
     */

    friend void signalChange(void * _arg);


    /**
     * Start this component running. This overridden method also starts
     * the encapsulated thread that forwards change information.
     * 
     */
    virtual void start();

    virtual void stop();

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
			    const cdl::ReceiverDeleteCondition & _condition = cdl::DO_NOT_DELETE_RECEIVER);
 
    /**
     * Set filters to receive no changes at all.
     */
    virtual void receiveNoChanges();

    /**
     * Set filters to receive changes using filters.
     */
    virtual void receiveChanges();
  
    bool m_bReceivingChanges;


//     /**
//      * Remove all filter objects. This allows all change events through to workingMemoryChanged.
//      */
//     virtual void clearFilterObjects();
  
  
    /**
     * Flag to determine whether this component should receive change
     * events from other subarchitectures.
     */
    bool m_receiveXarchChangeNotifications;

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

  


    /**
     * The thread that is used to forward change events to the derived
     * class.
     */
    WorkingMemoryChangeThread * m_pWMChangeThread;


    /**
     * Used to determine whether change events are queued or discarded.
     * Default is to discard them.
     */
    cdl::WorkingMemoryChangeQueueBehaviour m_queueBehaviour;

  protected:
  
    int getFilterCount() const {
      if(m_pChangeObjects) {
	return m_pChangeObjects->size();
      }
      else {
	return 0;
      }
    }

  public:



    /**
     * Get the entry from working memory with the given id.
     * 
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     * 
     */
    template <class T>
    boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const std::string & _id) {
      return getWorkingMemoryEntry<T>(_id,m_subarchitectureID);
    }

    /**
     * Get the entry from working memory in a particular subarchitecture
     * with the given id.
     * 
     * @param _subarch
     *            The subarchitecture in which the id is located.
     * @param _id
     *            The id for the entry in working memory.
     * @return The requested entry.
     */
    template <class T>
    boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const cdl::WorkingMemoryAddress & _wma) {
      return getWorkingMemoryEntry<T>(std::string(_wma.m_id),std::string(_wma.m_subarchitecture));
    }


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
    boost::shared_ptr< const CASTData<T> > getWorkingMemoryEntry(const std::string & _id,
								 const std::string & _subarch) {

      debug("getWorkingMemoryEntry: " + _id + " " + _subarch);
      assert(!_id.empty());//id must not be empty
      if(_subarch.empty()) {
	println("subarch empty");
      }
      assert(!_subarch.empty());//subarch must not be empty

      logReadEvent(_subarch,_id);

      std::string query = SubarchitectureWorkingMemoryProtocol
	::createIDQuery(getProcessIdentifier(),m_subarchitectureID,_subarch,_id);
      //println("subarch: " + _subarch);
      //println("query: " + query);

      std::vector< boost::shared_ptr< const CASTData<T> > > results;
    
      queryWorkingMemory(_subarch,query,results);

      //println("query count: " + pResults->size());


      if (results.size() == 0) {
	throw(DoesNotExistOnWMException(makeAdress(_subarch,_id),
					__HERE__, 
					"Entry does not exist on wm. Was looking in subarch %s for id %s", 
					_subarch.c_str(),_id.c_str()));
      }
      else if (results.size() > 1) {
	//should never happen
	throw(SubarchitectureProcessException(__HERE__, 
					      "requested entry count different from returned count: 1 != %i. Looking in subarch %s for id %s", 
					      results.size(), _subarch.c_str(), _id.c_str()));
      }


      boost::shared_ptr< const CASTData<T> > result = results[0];

      updateVersion(result->getID(), result->getVersion());

      return result;
    
    }







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
    void  getWorkingMemoryEntries(std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {
      return getWorkingMemoryEntries<T>(0,_results);
    }

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
				  std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {
      return getWorkingMemoryEntries<T>(m_subarchitectureID,_count,_results);
    }


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
				  std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {

      assert(!_subarch.empty());//subarch must not be empty

      std::string type = typeName<T>();

      logReadEvent(_subarch,type,_count);

      std::string query = SubarchitectureWorkingMemoryProtocol
	::createTypeQuery(getProcessIdentifier(),m_subarchitectureID,_subarch,type,_count);

      //println("subarch: " + _subarch);
      //println("type query: " + query);

      queryWorkingMemory(_subarch,query,_results);

      //println("query count: " + pResults->size());

      for(unsigned int i = 0; i < _results.size(); i++) {
 	updateVersion(_results[i]->getID(), _results[i]->getVersion());
      }


    }



    /**
     * Retrieve the working memory entries with the given ids from the
     * local subarchitecture working memory.
     * 
     * @param _subarch
     *            The subarchitecture which should be used to locate the
     *            working memory containing the entries.
     * @param _ids
     *            An array of working memory entry ids.
     * 
     * @param _results A sequence of matching entries. Must not be null initially.
     */
    template <class T>
    void  getWorkingMemoryEntries(const std::vector<std::string> & _ids,
				  std::vector < boost::shared_ptr<const CASTData<T> > > & _results) {
      return getWorkingMemoryEntries<T>(m_subarchitectureID,_ids,_results);
    }



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



    /**
     * Connection used to push change filters from working memory.
     */  
    PushConnectorOut<cdl::WorkingMemoryChangeFilter> * m_pFilterOutput;


    /**
     * The set of filters that are applied to determine whether to
     * forward change events to the derived class or not.
     */
    WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver *> * m_pChangeObjects;

    

    /**
     * This method sleeps until workingMemoryChanged has returned after
     * writing at least one change event. 
     *
     * NOT PROPERLY TESTED!!!
     */
    void waitForChanges();

  omni_mutex m_wmcConditionMutex;
  omni_condition * m_pWMCCondition;
  
  
};

} //namespace cast

#endif
