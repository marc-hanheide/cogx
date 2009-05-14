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

#ifndef CAST_SUBARCHITECTURE_WORKING_MEMORY_H_
#define CAST_SUBARCHITECTURE_WORKING_MEMORY_H_

#include <cast/core/CASTCore.hpp>
#include <cast/core/CASTProcessingComponent.hpp>
#include <cast/core/CASTWorkingMemory.hpp>
#include <cast/core/CASTWMPermissionsMap.hpp>
#include "WorkingMemoryPullQuery.hpp"
#include "WorkingMemoryChangeFilterMap.hpp"
#include "SubarchitectureWorkingMemoryProtocol.hpp"
#include "SubarchitectureProcessException.hpp"


#include <vector>
#include <memory>
#include <ext/hash_set>

namespace cast {

  typedef StringMap<PullConnectorOut<cdl::WorkingMemoryEntryList> *>::map WMEntryListPullConnectorMap;
  typedef StringMap<PullConnectorOut<cdl::WorkingMemoryPermissions> *>::map PermissionsPullConnectorMap;
  typedef StringMap<PushConnectorOut<cdl::WorkingMemoryEntry> *>::map WMEntryPushConnectorMap;
  typedef std::vector< PushConnectorOut<cdl::WorkingMemoryChange> * > WMChangePushConnectorVector; 
  typedef StringMap<PushConnectorOut<cdl::WorkingMemoryChange> * >::map WMChangePushConnectorMap; 

  typedef __gnu_cxx::hash_set<std::string> StringSet;


class SubarchitectureWorkingMemory: 
  public CASTProcessingComponent,
  public PushReceiver< cdl::WorkingMemoryEntry >,
  public PushSender< cdl::WorkingMemoryEntry >,
  public PushReceiver< CASTWorkingMemoryEntry >,
  public PushSender< cdl::WorkingMemoryChange >,
  public PushReceiver< cdl::WorkingMemoryChange >,
  public PullSender< cdl::WorkingMemoryEntryList >,
  public PullReceiver< cdl::WorkingMemoryEntryList >,
  public PullReceiver< std::vector<CASTWorkingMemoryEntry *> >,
  public PullReceiver< int >,
  public PullReceiver< bool >,
  public PushReceiver< cdl::WorkingMemoryChangeFilter >,
  public PushSender< cdl::WorkingMemoryChangeFilter >,
  public PullSender< cdl::WorkingMemoryPermissions >,
  public PullReceiver< cdl::WorkingMemoryPermissions > {

public:



  /**
   * Construct new object with a unique id. 
   * @param _id
   */
  SubarchitectureWorkingMemory(const std::string &_id);


  /**
   * Destructor. Cleans up container memory.
   */
  virtual ~SubarchitectureWorkingMemory();


  /**
   * Set the pull connector used to obtain working memory entries from
   * other working memories. Connections are only established if
   * _connectionID is prefixed with
   * ArchitectureConfiguration::XARCH_PREFIX. This method can be called
   * multiple times (i.e. once for each connected working memory).
   * 
   * @param _connectionID
   *            The ID of the connector.
   * @param _senderAdaptor
   *            A connector to use.
   * @see cast.core.interfaces.WMELPullInterface.WMELPullSender#setPullConnector(java.lang.String,
   *      cast.core.interfaces.WMELPullInterface.WMELPullConnectorOut)
   * @see ArchitectureConfiguration#XARCH_PREFIX
   */
  virtual void setPullConnector(const std::string & _connectionID, 
				PullConnectorOut<cdl::WorkingMemoryEntryList> * _pOut);

  virtual void setPullConnector(const std::string & _connectionID, 
				PullConnectorOut<cdl::WorkingMemoryPermissions> * _pOut);



  /**
   * Adds a push connector to use for broadcasting change information.
   * Xarch connectors must have ids prefixed with
   * ArchitectureConfiguration.XARCH_PREFIX. Global connectors must
   * have ids prefixed with ArchitectureConfiguration.GLOBAL_PREFIX.
   * Otherwise the connector is stored in a single variable, not a
   * collection, for local change broadcast.
   * 
   * @param _connectionID
   *            The id of the connection object.
   * @param _out
   *            The connection object.
   * @see ArchitectureConfiguration#XARCH_PREFIX
   * @see ArchitectureConfiguration#GLOBAL_PREFIX
   */
  virtual void setPushConnector(const std::string & _connectionID, 
				PushConnectorOut<cdl::WorkingMemoryChange> * _pOut);
  

  /**
   * Adds a push connector to use for forwarding wm operations on to
   * other wms. Xarch connectors must have ids prefixed with
   * ArchitectureConfiguration.XARCH_PREFIX. Global connectors must
   * have ids prefixed with ArchitectureConfiguration.GLOBAL_PREFIX.
   * Otherwise the connector is stored in a single variable, not a
   * collection, for local change broadcast.
   * 
   * @param _connectionID
   *            The id of the connection object.
   * @param _out
   *            The connection object.
   * @see ArchitectureConfiguration#XARCH_PREFIX
   * @see ArchitectureConfiguration#GLOBAL_PREFIX
   */
  virtual void setPushConnector(const std::string & _connectionID, 
				PushConnectorOut<cdl::WorkingMemoryEntry> * _pOut);

  virtual void setPushConnector(const std::string & _connectionID, 
				PushConnectorOut<cdl::WorkingMemoryChangeFilter> * _pOut);



  /**
   * Receive a pull query for a sequence of working memory entries.
   * The input parameter is parsed, then the resulting query object is
   * acted upon.
   * 
   * @param _query
   *            A framework query object.
   * @return The requested list of working memory entries.
   */
  virtual void receivePullQuery(const FrameworkQuery & _query, 
				FrameworkLocalData<cdl::WorkingMemoryEntryList> *& _pData);


  /**
   * Receive a pull query for a sequence of working memory entries.
   * The input parameter is parsed, then the resulting query object is
   * acted upon.
   * 
   * @param _query
   *            A framework query object.
   * @return The requested list of working memory entries.
   */
  virtual void receivePullQuery(const FrameworkQuery & _query, 
				FrameworkLocalData<std::vector<CASTWorkingMemoryEntry *> > *& _pData);


  /**
   * Receive a pull query for an update count of a wm entry.
   * 
   * @param _query
   *            A framework query object.
   * 
   */
  virtual void receivePullQuery(const FrameworkQuery & _query, 
				FrameworkLocalData< int > *& _pData);

  /**
   * Receive a pull query about the existance of a wm entry
   * 
   * @param _query
   *            A framework query object.
   * 
   */
  virtual void receivePullQuery(const FrameworkQuery & _query, 
				FrameworkLocalData< bool > *& _pData);


    /**
   * Receive a pull query about wm locks and permissions
   * 
   * @param _query
   *            A framework query object.
   * 
   */
  virtual void receivePullQuery(const FrameworkQuery & _query, 
				FrameworkLocalData< cdl::WorkingMemoryPermissions > *& _pData);


  /**
   * Receive data to be added, overwritten or deleted from the working
   * memory.
   * 
   * @param _src
   *            The source component for the data.
   * @param _data
   *            The data to be added, overwritten or deleted.
   * @see #performOperation(String, WorkingMemoryEntry)
   * @see WorkingMemoryEntry#m_operation
   */
  virtual void receivePushData(FrameworkLocalData<cdl::WorkingMemoryEntry> *_pData);

  virtual void receivePushData(FrameworkLocalData<CASTWorkingMemoryEntry> *_pData);

  /**
   * Used to receive change data from other working memories. Based on
   * various configuration options this data may be pushed on to
   * connected components.
   * 
   * @param _src
   *            The source component of the change information.
   * @param _data
   *            The change data.
   * @see #isForwardingXarchChangeNotifications()
   */
  virtual void receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData);



  /**
   * Receive a change filter to be for this working memory.
   * 
   */
  virtual void receivePushData(FrameworkLocalData<cdl::WorkingMemoryChangeFilter> *_pData);

  virtual void configure(std::map<std::string,std::string>& _config);


protected: 
  
  /**
   * Add a subarchitecture which should be ignored for changes
   * 
   * @param _subarch
   */
  void ignoreChangesFromSubarchitecture(const std::string & _subarch);


  /**
   * Determines whether the wm is allowed to forward this change to the
   * attached reader processes.
   * 
   * @param _change
   * @return true if the change is not from an ignored sa and passes all the filters
   */
  bool isAllowedChange(const cdl::WorkingMemoryChange & _change) const;


  /**
   * Determines whether the wm is allowed to forward this change to the
   * wm with the given id
   * 
   * @param _change
   * @return true if the change is not from an ignored sa and passes all the filters
   */

  bool
  isAllowedChange(const std::string & _wmid,
		  const cdl::WorkingMemoryChange & _change) const;


  /**
   * Add some data to working memory. If the given id already exists
   * in the working memory then the data isn't added and false is
   * returned.
   * 
   * @param _id
   *            The id to use for the data.
   * @param _pData
   *            The data itself.
   * @return True if the data is added successfully, else false.
   */
  virtual bool addToWorkingMemory(const std::string & _id, 
				  boost::shared_ptr<CASTWorkingMemoryItem> _pData);


  /**
   * Overwrite some data to working memory. If the given id does not
   * exist in the working memory then the data isn't written and false
   * is returned.
   * 
   * @param _id
   *            The id to use for the data.
   * @param _pData
   *            The data itself.
   * @return True if the data is overwritten successfully, else false.
   */
  virtual bool 
  overwriteWorkingMemory(const std::string & _id, 
			 boost::shared_ptr<CASTWorkingMemoryItem> _pData,
			 const std::string & _component);
  

  /**
   * Delete the specified entry from working memory. If the given id
   * does not exist in the working memory then nothing happens and
   * null is returned.
   * 
   * @param _id
   *            The id of the entry to delete.
   * @return The item that is deleted, or null if nothing is deleted.
   */
  virtual boost::shared_ptr<CASTWorkingMemoryItem> 
  deleteFromWorkingMemory(const std::string & _id,
			  const std::string & _component);


/**
   * Handle a working memory query generated by an attached component.
   * This determines whether it is a request for a local or remote
   * working memory, and calls either handleLocalWorkingMemoryQuery or
   * handleGlobalWorkingMemoryQuery accordingly.
   * 
   * @param _pQuery
   *            An object representing a query.
   * @param _pWMEL Ouput for the requested working memory entries. Must not be null.
   */
  void handleWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery, 
				std::vector<CASTWorkingMemoryEntry*> & _wmel);
  


  

  /**
   * Handle a working memory query intended for another working
   * memory. This method forwards the query on to that working memory
   * if it is connected to it.
   * 
   * @param queryObject
   *            The object representation of the query.
   * @param _pWMEL Ouput for the requested working memory entries. Must not be null.   
   */
  void handleXarchWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery,
				      std::vector<CASTWorkingMemoryEntry*> & _wmel);


  /**
   * Handle a working memory query intended for this working memory.
   * 
   * @param _pQuery
   *            The object representation of the query.
   * @param _pWMEL Ouput for the requested working memory entries. Must not be null.   
   */
  virtual void 
  handleLocalWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery,
				std::vector<CASTWorkingMemoryEntry*> & _wmel);


    

  cdl::WorkingMemoryPermissions
  handleLocalPermissionsQuery(const std::string & _id,
			      const cdl::WorkingMemoryLockRequest & _request, 
			      const std::string & _component) 
    throw(SubarchitectureProcessException);

  /**
   * Parse the query string received by the working memory.
   */
  virtual AbstractWorkingMemoryPullQuery * 
  parseWorkingMemoryQuery(const FrameworkQuery & _query);


  

  /**
   * Empty run method. Does nothing.
   * 
   */
  virtual void runComponent(){};


  virtual 
  void 
  addComponentChangeFilter(const std::string &_src,
			   const cdl::WorkingMemoryChangeFilter &_filter);
  virtual 
  void 
  deleteComponentChangeFilter(const std::string &_src,
			      const cdl::WorkingMemoryChangeFilter &_filter);

  virtual 
  void 
  addWMChangeFilter(const std::string &_src,
		    const cdl::WorkingMemoryChangeFilter &_filter);
  virtual 
  void 
  deleteWMChangeFilter(const std::string &_src,
		       const cdl::WorkingMemoryChangeFilter &_filter);



  /**
   * Determines whether to forward change notifications to other
   * subarchitecture working memories (i.e. its peers).
   * 
   * @param sendXarchChangeNotifications
   *            The sendXarchChangeNotifications to set.
   */
  void setSendXarchChangeNotifications(bool sendXarchChangeNotifications) {
    m_sendXarchChangeNotifications = sendXarchChangeNotifications;
  }


  /**
   * Determines whether to forward change notifications to other
   * subarchitecture working memories (i.e. its peers).
   * 
   * @return Returns the sendXarchChangeNotifications.
   */
  bool isSendingXarchChangeNotifications() {
    return m_sendXarchChangeNotifications;
  }


//   /**
//    * Determines whether to forward change notifications from other
//    * working memories to locally attached components.
//    * 
//    * @param forwardXarchChangeNotifications
//    *            The forwardXarchChangeNotifications to set.
//    */
//   void setForwardXarchChangeNotifications(bool forwardXarchChangeNotifications) {
//     m_forwardXarchChangeNotifications = forwardXarchChangeNotifications;
//   }

//   /**
//    * Determines whether to forward change notifications from other
//    * working memories to locally attached components.
//    * 
//    * @return Returns the forwardXarchChangeNotifications.
//    */
//   bool isForwardingXarchChangeNotifications() {
//     return m_forwardXarchChangeNotifications;
//   }



  
  /**
   * Retrieve items from working memory by type for local use.
   **/
  template<class ItemType>
  void getItems(const std::string &_type, 
		std::vector< boost::shared_ptr< WorkingMemoryItem<ItemType> > > &_items) {
    std::vector<std::string> ids;
    m_workingMemory.getIDsByType(_type,0, ids);
    for(std::vector<std::string>::iterator i = ids.begin();
	i < ids.end(); ++i) {
      _items.push_back(getItem<ItemType>(*i));
    }
  }



  /**
   * Retrieve an item from working memory for local use.
   **/
  template<class ItemType>
  boost::shared_ptr<WorkingMemoryItem<ItemType> > getItem(const std::string &_id) {
    
    //get the based item
    boost::shared_ptr<CASTWorkingMemoryItem> pItem = m_workingMemory.get(_id);
    
    if(pItem) {
      
      std::string type = pItem->getType();

      //if the item is local
      if(pItem->isLocal()) {
	
	//cast to subclass type
	boost::shared_ptr< WorkingMemoryItem<ItemType> > pWMI = 
	  boost::dynamic_pointer_cast< WorkingMemoryItem<ItemType>, CASTWorkingMemoryItem  >(pItem);
	
	//if the cast is valid
	if(pWMI) {
	  return pWMI;
	}
	//if it's not, kick up a fuss
	else {
	  throw(CASTException(__HERE__, "Unable to cast item pointer to specific type."));
	}
      }
      else {
	
	//if it's not local then it's an any
	boost::shared_ptr< WorkingMemoryItem<CORBA::Any> >pWMI 
	  = boost::dynamic_pointer_cast< WorkingMemoryItem<CORBA::Any>, CASTWorkingMemoryItem >(pItem);

	if(pWMI) {

	  //get the translator for this data type
	  const RemoteDataTranslator<ItemType> * pTranslator = NULL;
	  pTranslator 
	    = RemoteTranslatorManager::translator<ItemType>(); 


	  if(pTranslator) {
	    //this copies the data from the working memory entry so
	    //pLocalData is new, free memory
	    ItemType * pLocalData = pTranslator->translate(*pWMI->getData());
	    //create a new wrapper object and a shared ptr to it. When
	    //this object is deleted it will clean up pLocalData
	    boost::shared_ptr< WorkingMemoryItem<ItemType> > pNewWMI(new WorkingMemoryItem<ItemType>(_id,type, pLocalData));	    
	    return pNewWMI;
	  }
	  else {
	    throw CASTException(__HERE__, "failed to get translator for type");
	  }
	}
	else {
	  throw CASTException(__HERE__, "failed to cast CASTWorkingMemoryItem to CORBA::Any WorkingMemoryItem");
	}

	//return NULL ptr
	return boost::shared_ptr<WorkingMemoryItem<ItemType> >();
      }
    }
    //return NULL ptr
    return boost::shared_ptr<WorkingMemoryItem<ItemType> >();

  }


  /**
   * Retrieve an item from a remote working memory for local use.
   **/
  template<class ItemType>
  boost::shared_ptr< const WorkingMemoryItem<ItemType> > 
  getRemoteItem(const cdl::WorkingMemoryAddress &_wma) {
    
    boost::shared_ptr< const WorkingMemoryItem<ItemType> > item;

    std::string id(_wma.m_id);
    std::string sa(_wma.m_subarchitecture);

    //create the query object.. 
    AbstractWorkingMemoryPullQuery *pQuery = 
      new WorkingMemoryPullQuery<std::string>(getProcessIdentifier(), 
					 m_subarchitectureID, 
					 sa,
					 0,
					 new std::string(id),
					 AbstractWorkingMemoryPullQuery::ID);
    
    //    log("created query");

    //create the std::vector to hold the result of the query
    std::vector<CASTWorkingMemoryEntry *> results;
    //then resuse pull forwarding mechanism to fetch the data
    
    handleXarchWorkingMemoryQuery(pQuery,results);
    //log("got something back: %i",results.size());

    if(results.size() == 1) {
      CASTWorkingMemoryEntry * pWME = results[0];      
      std::string type = pWME->getItem().getType();

      boost::shared_ptr< WorkingMemoryItem<CORBA::Any> > pWMI = 
	boost::dynamic_pointer_cast< WorkingMemoryItem<CORBA::Any>, 
	CASTWorkingMemoryItem  >(pWME->item());

      if(pWMI) {
	
	const RemoteDataTranslator<ItemType> * pTranslator = NULL;
	pTranslator 
	  = RemoteTranslatorManager::translator<ItemType>();
	    
	    
	ItemType * pLocalData = pTranslator->translate(*pWMI->getData());	      
	//create a boost::shared_ptr to the data wrapper, when out of
	//references this will delete pLocalData for us
	item = boost::shared_ptr< const WorkingMemoryItem<ItemType> >(new WorkingMemoryItem<ItemType>(id,
												      type,
												      pLocalData));
      }
      else {
	throw CASTException(__HERE__, "failed to cast CASTWorkingMemoryItem to CORBA::Any WorkingMemoryItem");
      }
    }
    else {
      //delete memory

      debug("Pull resulted in more or less than 1 item. Count == %i", results.size());
    }


    //need to delete all entries in the list too
    for(std::vector<CASTWorkingMemoryEntry *>::iterator i = results.begin();
	i < results.end(); ++i) {
      delete (*i);
      (*i) = NULL; //probably not needed?
    }


    //return null for the time being
    return item;

  }

  
  bool m_sendXarchChangeNotifications;
  //  bool m_forwardXarchChangeNotifications;

  ///change list connections for other subarchitectures
  WMChangePushConnectorMap m_xarchChangeConnections;

  ///pull connections for retrieval from other subarchitectures
  WMEntryListPullConnectorMap  m_xarchWMPullConnections;

  PermissionsPullConnectorMap m_xarchLockConnections;

  ///pull connections for retrieval from other subarchitectures
  WMEntryPushConnectorMap  m_xarchWMPushConnections;

  ///change connection for local components
  WMChangePushConnectorVector  m_wmChangeBroadcast;

  CASTWorkingMemory m_workingMemory;

  ///set for storing change filters
  WorkingMemoryChangeFilterMap<std::string> m_componentFilters;

  ///set for storing change filters for other wms
  WorkingMemoryChangeFilterMap<std::string> m_wmFilters;


private:


  void readBlock(const std::string & _id, const std::string & _component);

  /**
   * Push the input filter to all other wms.
   * 
   * @param _data
   */
  void sendFilter(const cdl::WorkingMemoryChangeFilter & _data);


  /**
   * @param _id
   * @param _type
   */
  void logMemoryAdd(const std::string & _srcID, const std::string & _srcSA, const std::string & _id,
		    const std::string & _type) {
    logEvent(cdl::ui::ADD, _srcID, _srcSA, _type, _id);
  }

  /**
   * @param _id
   * @param _type
   */
  void logMemoryDelete(const std::string & _srcID, const std::string & _srcSA,
		       const std::string & _id, const std::string & _type) {
    logEvent(cdl::ui::DELETE, _srcID, _type, "", _id);
  }

  /**
   * @param _id
   * @param _type
   */
  void logMemoryOverwrite(const std::string & _srcID, const std::string & _srcSA,
			  const std::string & _id, const std::string & _type) {
    logEvent(cdl::ui::OVERWRITE, _srcID, _srcSA, _type,
	     _id);
  }

  void logReadEvent(const std::string & _srcID, 
		    const std::string & _srcSA, 
		    const std::string & _type, 
		    const int & _count) {
    logEvent(cdl::ui::GET, _srcID, _srcSA, _type, "");
  }

  void logReadEvent(const std::string & _srcID, 
		    const std::string & _srcSA, 
		    const std::string & _id) {
    logEvent(cdl::ui::GET, _srcID, _srcSA, "", _id);
  }

  void logMemorySize(const int & _currentSize) {
    m_componentStatusMutex.lock();
    if(_currentSize > m_componentStatus.m_totalChangeEventsFiltered) {
      m_componentStatus.m_totalChangeEventsFiltered = _currentSize;
    }
    m_componentStatus.m_changeQueue = _currentSize;
    m_componentStatusMutex.unlock();
  }
  


  /**
   * Perform a working memory operation as described by _pData.
   * 
   * @param _pData
   */     
   void performOperation(CASTWorkingMemoryEntry *_pData);
  
   /**
   * Perform a working memory operation on the local wm as described
   * by _pData.
   * 
   * @param _pData
   */     
  void performLocalOperation(CASTWorkingMemoryEntry *_pData);
  
   /**
    * Forward a working memory operation to a remote wm as described by
   * _pData.
   * 
   * @param _pData
   */     
   void performXarchOperation(CASTWorkingMemoryEntry *_pData);
  



  /**
  * Signal that an operation has occurred to all connected
  * components.
  * 
  * @param _op
  *            The operation type to signal.
  * @param _src
  *            The component that caused the operation.
  * @param _id
  *            The id of the entry that was the subject of the
  *            operation.
  * @param _type
  *            The ontological type of the entry that was the subject
  *            of the operation.
  */
  void signalChange(cdl::WorkingMemoryOperation _op, const std::string & _src,
		    const std::string &  _id,  const std::string &  _type);


  
  
  /**
   * Extract a subarchitecture name from an ID for a working memory
   * component. Assumes that the id was created with the createID
   * method (in Java!).
   * 
   * @param _id
   *            The id of the working memory component.
   * @return The name of subarchitecture which contains it.
   */
  std::string subarchitectureFromID(const std::string & _id);


  

  /**
   * Get a single entry by its ID.
   * 
   * @param _pQuery
   *            The ID of the entry to get.
   * @param _pWMEL Ouput for the requested working memory entry. Must not be null.   
   */
  void getEntryByID(WorkingMemoryPullQuery<std::string> * _pQuery,
		    std::vector<CASTWorkingMemoryEntry*> & _wmel);



//   /**
//    * Get an array of working memory entries from the stored working
//    * memory. If an id is missing then there is nothing returned for
//    * that id.
//    *
//    * @param _pQuery
//    *            The IDs of the entries to get.
//    * @param _pWMEL Ouput for the requested working memory entry. Must not be null.   
//    */

//   void getEntriesByID(WorkingMemoryPullQuery< std::vector<std::string> > * _pQuery,
// 		      std::vector<CASTWorkingMemoryEntry*> & _wmel);



  /**
   * Get an array of working memory entries that have a given
   * ontological type.
   * @param _pQuery
   *            The ontological type of the entries to get.
   * @param _pWMEL Ouput for the requested working memory entry. Must not be null.   
   */

  void getEntriesByType(WorkingMemoryPullQuery<std::string> * _pQuery,
			std::vector<CASTWorkingMemoryEntry*> & _wmel);


  void buildIDLists(std::map<std::string,std::string>& _config);

  bool isWorkingMemoryID(const std::string & _id) const;

  void processWorkingMemoryFilter(const std::string & _src,
				  const cdl::WorkingMemoryChangeFilter & _filter);

  void processComponentFilter(const std::string & _src,
			      const cdl::WorkingMemoryChangeFilter & _filter);

  /**
   * Determines whether to actually peform change filtering on the wm,
   * just used for testing
   */
  bool m_wmChangeFiltering;

    /**
   * Determines whether to share wm filters
   */
  bool m_wmDistributedFiltering;
  
  /**
   * Stores subarchitectures to ignore changes from
   */ 
  StringSet m_ignoreList;

  /**
   * Stores ids of other working memories.
x   */ 
  StringSet m_wmIDs;


  /**
   * Used for locks and permissions
   */
  CASTWMPermissionsMap m_permissions;
  
  
  /**
   * Connector for sending filters
   */
  PushConnectorOut<cdl::WorkingMemoryChangeFilter> * m_xarchWMCFPushConnector;

  
};

} //namespace cast

#endif
