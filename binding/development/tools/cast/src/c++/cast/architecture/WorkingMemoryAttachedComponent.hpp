/*
 * CAST - The CoSy Architecture Schema Toolkit
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

#ifndef CAST_WORKING_MEMORY_ATTACHED_COMPONENT_H_
#define CAST_WORKING_MEMORY_ATTACHED_COMPONENT_H_

#include <cast/core/CASTCore.hpp>
#include <balt/interface/BALTInterface.hpp>

#include "cast/architecture/LocalWorkingMemoryAttachedComponent.hpp"
#include "cast/architecture/DoesNotExistOnWMException.hpp"

#include <list>
#include <vector>

namespace cast {

  class WorkingMemoryAttachedComponent :
    public LocalWorkingMemoryAttachedComponent, 
    public PullSender<cdl::WorkingMemoryEntryList>,
    public PullSender<std::vector<CASTWorkingMemoryEntry *> > {


  private:
    
    bool existsOnWorkingMemoryXarch(const std::string & _id, 
				    const std::string & _subarch);

    int getVersionNumberXarch(const std::string & _id, 
			      const std::string & _subarch);

    /**
     * Connection used to pull information from working memory.
     */
    PullConnectorOut<cdl::WorkingMemoryEntryList> * m_pOutputFromRemoteWorkingMemory;

    /**
     * Connection used to pull information from working memory.
     */
    PullConnectorOut<std::vector<CASTWorkingMemoryEntry *> > * m_pOutputFromLocalWorkingMemory;


    template <class T>
    void queryLocalWorkingMemory(const std::string &_subarch,
				 const std::string _query,
				 std::vector < boost::shared_ptr<const CASTData<T> > > & _results) {
    
      //println("local query");
    
      FrameworkQuery query(getProcessIdentifier(),_query);
      FrameworkLocalData<std::vector<CASTWorkingMemoryEntry *> > * pData = NULL;
    
      m_pOutputFromLocalWorkingMemory->pull(query,pData);
  
      //clear the vector
      _results.clear();
    
      //get list of entries
      std::vector<CASTWorkingMemoryEntry *> * pWMEL = pData->data();

      //println("pull is over: length = " + pWMEL->length());

      const RemoteDataTranslator<T> * pTranslator = NULL;
    
      //now copy results into format and container
      for(unsigned int i = 0; i < pWMEL->size(); i++) {
	//println("copying entry");
      
	//get the actual item from the entry wrapper
	//cout<<"start of for: "<<(*pWMEL)[i]->item().use_count()<<endl;
	boost::shared_ptr<CASTWorkingMemoryItem> pItem = (*pWMEL)[i]->item();
	//cout<<"after assigment: "<<(*pWMEL)[i]->item().use_count()<<endl;

	//if the item is non-null
	if(pItem) {
	
	  //if the item is a local c++ object
	  if(pItem->isLocal()) {
	  
	    //cast the shared ptr to be of the type we're interested in

	    //old version... why oh why
	    // 	  boost::shared_ptr< WorkingMemoryItem<T> > pWMI = 
	    // 	    boost::dynamic_pointer_cast< WorkingMemoryItem<T>, CASTWorkingMemoryItem  >(pItem);

	    //cout<<"count before cast: "<<pItem.use_count()<<endl;

	    //std::cout<<"type: "<<pItem->getType()<<std::endl;
	    //std::cout<<"type: "<<typeid(T).name()<<std::endl;

	    //this increases reference count
	    boost::shared_ptr< CASTData<T> > pWMI = 
	      boost::dynamic_pointer_cast< CASTData<T>, CASTWorkingMemoryItem  >(pItem);
	  

	    CASTWorkingMemoryItem * item = pItem.get();
	    CASTData<T>* data = dynamic_cast<CASTData<T>* >(item);

	    if(!data) {
	      std::cout<<"cast failed anyway"<<std::endl;
	    }

	    //cout<<"count after cast: "<<pItem.use_count()<<endl;
	    //cout<<"count after cast: "<<pWMI.use_count()<<endl;

	    //if the cast was successful
	    if(pWMI) {
	    
	      //at this point pWMI is shared pointer to the actual
	      //WorkingMemoryItem in the attached working memory

	      // //cast again to data type
	      // 	    boost::shared_ptr< CASTData<T> > pCD = 
	      // 	      boost::dynamic_pointer_cast< CASTData<T>, WorkingMemoryItem<T>  >(pWMI);

	      // 	    if(pCD){ 
	      _results.push_back(pWMI);
	      // 	    }
	      // 	    else {
	      // 	      throw CASTException(__HERE__, "no joy");
	      // 	    }
	      //std::string id((*pWMEL)[i]->getAddress().m_id);
	    
	    
	    
	      // 	    //removed constness to construct new object around
	      // 	    //it... this new object preserves constness though
	      // 	    const T * pData = pWMI->data();
	      // 	    T * pNonConstData = const_cast<T *>(pData);
	      // 	    CASTData<T> * pCTD = new CASTData<T>(id,
	      // 						 pWMI->getType(),
	      // 						 pNonConstData);
	      // 	    //make sure data cannot be deleted
	      // 	    pCTD->protectData();
	    
	      //_results->push_back(pCTD);

	    }

	    //cast to local type failed
	    else {
	      throw CASTException(__HERE__, 
				  "%s: failed to cast CASTWorkingMemoryItem to generically typed WorkingMemoryItem", 
				  getProcessIdentifier().c_str());
	    }
	  }
	  else {
	  
	    //else if it's not local it will be an Any type
	    boost::shared_ptr< WorkingMemoryItem<CORBA::Any> > pWMI = 
	      boost::dynamic_pointer_cast< WorkingMemoryItem<CORBA::Any>, CASTWorkingMemoryItem  >(pItem);

	    //START = convert from any to data using ontology
	  
	    if(pWMI) {
	      std::string id((*pWMEL)[i]->getAddress().m_id);
	      std::string type = (*pWMEL)[i]->item()->getType();
	      int version  = (*pWMEL)[i]->item()->getVersion();

	      debug("WorkingMemoryAttachedComponent::queryLocalWorkingMemory: %s %s %d", id.c_str(), type.c_str(), version);
	    

	      if(pTranslator == NULL) {
		pTranslator 
		  = RemoteTranslatorManager::translator<T>();
	      }
	    
	    
	    
	      T * pLocalData = pTranslator->translate(*(pWMI->getData()));
	      //create a shared_ptr to the data wrapper, when out of
	      //references this will delete pLocalData for us
	      boost::shared_ptr< CASTData<T> > pNewData(new CASTData<T>(id,
									type,
									version,
									pLocalData));
	      //store the result
	      _results.push_back(pNewData);
	    }
	    else {
	      throw CASTException(__HERE__, "failed to cast CASTWorkingMemoryItem to CORBA::Any WorkingMemoryItem");
	    }
	    
	  }

	} 
	else {
	  throw CASTException(__HERE__, "item in object is null");
	}

      }

      //need to delete all entries in the list too
      for(std::vector<CASTWorkingMemoryEntry *>::iterator i = pData->data()->begin();
	  i < pData->data()->end();
	  ++i) {
	delete (*i);
	(*i) = NULL; //probably not needed?
      }

      //delete data from pull
      delete pData;

    }

    template <class T>
    void queryRemoteWorkingMemory(const std::string &_subarch,
				  const std::string _query,
				  std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {

      //println("remote query");

      FrameworkQuery query(getProcessIdentifier(),_query);
      FrameworkLocalData<cdl::WorkingMemoryEntryList> * pData = NULL;
    
      m_pOutputFromRemoteWorkingMemory->pull(query,pData);
  
      //clear the vector
      _results.clear();
    
      //get list of entries
      cdl::WorkingMemoryEntryList * pWMEL = pData->data();

      //println("pull is over: length = " + pWMEL->length());
    


      //now copy results into format and container
      for(unsigned int i = 0; i < pWMEL->length(); i++) {
	//	println("copying entry");
     
	//convert from any to data using ontology
      
	boost::shared_ptr< CASTData<T> > pTypedData;
	pTypedData = convertFromWorkingMemoryEntry<T>((*pWMEL)[i]);
      
	//println("done");

	if(pTypedData) {
	  //add to results vector
	  _results.push_back(pTypedData);
	}
	else {
	  //probably get another exception before getting here anyway
	  throw(CASTException(__HERE__,
			      "WorkingMemoryReaderProcess: unable to created typed data from working memory entry."));
	}

      }
    }

  /**
   * Use the ontology to extract a particular instantiation of
   * CASTTypedData from the given working memory entry. This method
   * represents the point where C++ datatypes, CAST datatypes and
   * ontological types combine. If an error message is produced by
   * this method it is usually because the requested C++ type T does
   * not match the C++ type given as the template parameter when
   * _wme.m_type was stored using the CASTDatatypeManager.
   *
   * @param _wme a working memory entry that has a type and some data
   * in any format.
   *
   * @return Returns an object in which the data is extracted into the
   * correct c++ format.
   */
    template <class T>
    boost::shared_ptr< CASTData<T> > 
    convertFromWorkingMemoryEntry(const cdl::WorkingMemoryEntry & _wme) const {

      //get translator for this ontological type
      std::string id(_wme.m_address.m_id);
      std::string type(_wme.m_type);
    
      //cout<<"id: "<<id<<endl;
      //cout<<"ontologicalType: "<<ontologicalType<<endl;
      //cout<<"frameworkType: "<<frameworkType<<endl;
      
      const RemoteDataTranslator<T> * pTrans 
	= RemoteTranslatorManager::translator<T>();


      T * pData = pTrans->translate(_wme.m_data);
    
      if(pData) {
	boost::shared_ptr< CASTData<T> > pTypedData(new CASTData<T>(id,
								    type,
								    _wme.m_version,
								    pData));
	return pTypedData;
      }

      throw(CASTException(__HERE__,
			  "Unable to extract data in template format from working memory entry."));
    
  }
    

  public:

    /**
     * Constructor.
     *
     * @param _id The unique identifier of this process.
     */
    WorkingMemoryAttachedComponent(const std::string &_id);  

    /**
     * Destructor.
     */
    virtual ~WorkingMemoryAttachedComponent(){};

    /**
     * Set the pull connector used by the component to get entries from
     * local working memory.
     * 
     * @param _connectionID
     *            The id of the connector.
     * @param _pOut
     *            The connector itself.
     * 
     */
    virtual void 
    setPullConnector(const std::string & _connectionID, 
		     PullConnectorOut<cdl::WorkingMemoryEntryList> * _pOut);

    /**
     * Set the pull connector used by the component to get entries from
     * local working memory.
     * 
     * @param _connectionID
     *            The id of the connector.
     * @param _pOut
     *            The connector itself.
     * 
     */
    virtual void 
    setPullConnector(const std::string & _connectionID, 
		     PullConnectorOut<std::vector<CASTWorkingMemoryEntry *> > * _pOut);


    /**
     * Determines whether an entry exists on working memory at the
     * given address.
     * 
     * @param _wma The address to check.
     *
     * @return True if the entry exists, otherwise false.
     */
    bool existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma);

    /**
     * Determines whether an entry exists on working memory at the
     * given address.
     * 
     * @param _subarch
     *            The subarchitecture in which the id is located.
     * @param _id
     *            The id for the entry in working memory.
     *
     * @return True if the entry exists, otherwise false.
     */
    bool existsOnWorkingMemory(const std::string & _id) {
      return LocalWorkingMemoryAttachedComponent::existsOnWorkingMemory(_id);
    }

    /**
     * Determines whether an entry exists on working memory at the
     * given address.
     * 
     * @param _subarch
     *            The subarchitecture in which the id is located.
     * @param _id
     *            The id for the entry in working memory.
     *
     * @return True if the entry exists, otherwise false.
     */
    bool existsOnWorkingMemory(const std::string & _id, 
			       const std::string & _subarch);


    
    /**
     * Get a count of the number of times the working memory entry at
     * the given address has been overwritten.
     * 
     * @param _wma The address for the entry in working memory.
     *
     * @return The version number of the entry. This will be 0 if it
     * has never been overwritten.
     *
     * @throws DoesNotExistOnWMException if the entry has never
     * existed on working memory.
     *
     * @remark Interface change: Renamed to reflect new role, behaviour still
     *         the same: getOverwriteCount -> getVersionNumber.
     */
    int getVersionNumber(const cdl::WorkingMemoryAddress & _wma);

    
    /**
     * Get a count of the number of times the working memory entry at
     * the given address has been overwritten.
     * 
     * @param _subarch
     *            The subarchitecture in which the id is located.
     * @param _id
     *            The id for the entry in working memory.
     *
     * @return The version number of the entry. This will be 0 if it
     * has never been overwritten.
     *
     * @throws DoesNotExistOnWMException if the entry does not exist
     * on working memory.
     *
     * @remark Interface change: Renamed to reflect new role, behaviour still
     *         the same: getOverwriteCount -> getVersionNumber.
     */
    int getVersionNumber(const std::string & _id, 
			 const std::string & _subarch);


    /**
     * Try to obtain a lock on a working memory entry with the given
     * permissions. This will block until the desired lock is obtained.
     * 
     * @param _id
     * @param _subarch
     * @param _permissions
     * @throws DoesNotExistOnWMException
     */
    virtual void lockEntry(const std::string & _id, 
		   const std::string & _subarch,
		   const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);



    /**
     * Try to obtain a lock on a working memory entry with the given
     * permissions. This will block until the desired lock is obtained.
     * 
     * @param _wma
     * @param _permissions
     * @throws DoesNotExistOnWMException
     */
    virtual void lockEntry(const cdl::WorkingMemoryAddress & _wma,
			   const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);

    
    
    /**
     * Unlock the given working memory entry.
     * 
     * @param _id
     * @param _subarch
     * @throws DoesNotExistOnWMException
     */
    virtual void unlockEntry(const std::string & _id, 
			     const std::string & _subarch)
      throw(DoesNotExistOnWMException);


    /**
     * Unlock the given working memory entry.
     * 
     * @param _wma
     * @throws DoesNotExistOnWMException
     */
    virtual void unlockEntry(const cdl::WorkingMemoryAddress & _wma)
      throw(DoesNotExistOnWMException);



    /**
     * Try to obtain a lock on a working memory entry. This will return true if
     * the item is locked, or false if not. This method does not block.
     * 
     * @param _id
     * @param _subarch
     * @param _permissions
     * @return
     * @throws DoesNotExistOnWMException
     */
    virtual bool tryLockEntry(const std::string & _id, 
		      const std::string & _subarch,
		      const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);

    
    /**
     * Try to obtain a lock on a working memory entry. This will return true if
     * the item is locked, or false if not. This method does not block.
     * 
     * @param _wma
     * @param _permissions
     * @return
     * @throws SubarchitectureProcessException
     */
    virtual bool tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
			      const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);


    /**
     * Gets the permissions currently set on the given working memory item.
     * 
     * @param _id
     * @param _subarch
     * @return
     * @throws DoesNotExistOnWMException
     */
    cdl::WorkingMemoryPermissions getPermissions(const std::string &_id,
						 const std::string &_subarch) const
      throw(DoesNotExistOnWMException);


    
    /**
     * Gets the permissions currently set on the given working memory item.
     * 
     * @param _wma
     * @return
     * @throws WMException
     */
    cdl::WorkingMemoryPermissions getPermissions(const cdl::WorkingMemoryAddress &_wma) const
      throw(DoesNotExistOnWMException);


    /**
     * 
     * Checks whether a given item on working memory is currently overwritable
     * by this component.
     * 
     * @param _id
     * @param _subarch
     * @return
     * @throws WMException
     */
    bool isOverwritable(const std::string & _id, 
			const std::string & _subarch) const
      throw(DoesNotExistOnWMException);

    /**
     * Checks whether a given item on working memory is currently deletable by
     * this component.
     * 
     * @param _id
     * @param _subarch
     * @return
     * @throws WMException
     */
    bool isDeletable(const std::string & _id, 
		     const std::string & _subarch) const
      throw(DoesNotExistOnWMException);
    
    /**
     * Checks whether a given item on working memory is currently readable by
     * this component.
     * 
     * @param _id
     * @param _subarch
     * @return
     * @throws WMException
     */
    bool isReadable(const std::string & _id, const std::string & _subarch) const
      throw(DoesNotExistOnWMException);



  protected:
    /**
     * Query working memory with the given query std::string. In this
     * instance the subarch parameter is ignored as it is not
     * appropriate in subarchs. The parameter is present for later
     * though. The query std::string must be generated using one of the
     * SubarchitectureWorkingMemoryProtocol create query methods.
     * 
     * @see SubarchitectureWorkingMemoryProtocol
     * @param _subarch
     *            The subarchitecture which the query is for.

     * @param _query
     *            The query string itself, generated by SubarchitectureWorkingMemoryProtocol.
     *
     * @param A pointer to an existing vector into which the retreieved
     * data is added. If the pointer is NULL then nothing will be done
     */
    template <class T>
    void queryWorkingMemory(const std::string &_subarch,
			    const std::string _query,
			    std::vector < boost::shared_ptr< const CASTData<T> > > & _results) {
    
      if(m_pOutputFromLocalWorkingMemory) {
	queryLocalWorkingMemory(_subarch,_query,_results);      
      }
      else if (m_pOutputFromRemoteWorkingMemory) {
	queryRemoteWorkingMemory(_subarch,_query,_results);
      }
      else {
	throw(SubarchitectureProcessException(__HERE__,"WorkingMemoryReaderProcess: no pull connection to working memory is set"));	
      }

    
    }
  
  /**
   * Checks whether this component has read the more recent version of the
   * data at this working memory address.
   * 
   * @param _id
   * @param _subarch
   * @return
   * @throws ConsistencyException
   *             if the id is not versioned.
   * @throws DoesNotExistOnWMException
   *             if the _id does not exist on wm
   */
    bool 
    haveLatestVersion(const std::string &_id, const std::string &_subarch)
      throw(ConsistencyException, DoesNotExistOnWMException);


    void checkConsistency(const std::string & _id, const std::string & _subarch) throw(ConsistencyException, DoesNotExistOnWMException);
    void checkConsistency(const cdl::WorkingMemoryAddress &_wma) throw(ConsistencyException, DoesNotExistOnWMException);



  };

} //namespace cast

#endif
