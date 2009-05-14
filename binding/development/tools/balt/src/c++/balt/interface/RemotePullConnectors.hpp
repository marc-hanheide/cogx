/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef REMOTE_PULL_CONNECTORS_H_
#define REMOTE_PULL_CONNECTORS_H_

#include "RemoteDataTranslator.hpp"
#include "includes.hpp"
#include "includes.hpp"

#include <balt/core/BALTCore.hpp>


/**
 * Generic class to handle pull sending connections that go outside of
 * native C++.
 */
template <class T>
class RemotePullSenderImpl : 
  public POA_RemoteConnectors::RemotePullSender,
  public FrameworkConnector,
  public PullConnectorOut<T> {

public:
  /**
   * Default constructor. Sets connector to nil.
   */
    RemotePullSenderImpl(const std::string & _id) : FrameworkConnector(_id)  {
      m_out = RemoteConnectors::RemotePullConnector::_nil();
      m_pTranslator = NULL;

      //std::cout<<"new RemotePullSenderImpl: "<<_id<<std::endl;
    }

    /**
     * Destructor. Deletes translator.
     */
    virtual ~RemotePullSenderImpl() {
      delete m_pTranslator;
    };

    //PullConnectorOut methods...


    /**
     * Execute a pull query on the remote pull connector and put the
     * results into _data;
     * 
     * @param _query The query for this pull.  
     *
     * @param _data The data object to hold the results of the pull.
     */
    virtual void pull(const FrameworkQuery &_query, FrameworkLocalData<T> *&_data) {


      try {
	if(CORBA::is_nil(m_out)) {
	  std::cerr<<"setPullConnector not called, or pointer connector is null"<<std::endl;
	  _data = new FrameworkLocalData<T>("error!",(T*)NULL);
	} 
	else {
	  //std::cout<<"pulling from remote 2... "<<std::endl;
	
	
	  if(_data) {
	    std::cout<<"Input pointer to pull was not null... leaking memory"<<std::endl;
	  }
      
	  //Pull the remote connection to get an Any object.
	  CORBA::Any * p_a = NULL;
      
	  p_a = m_out->pull(_query.getSource().c_str(),
			    _query.getQuery().c_str());
      
	  //_data = new FrameworkLocalData<T>("error!",(T*)NULL);

      
	  //Translate the Any object into the correct data type
      
	  //This allocates memory for the data and handles any
	  //interactions with Any memory
	  T * p_data = m_pTranslator->translate(*p_a);
      
	  //	std::cout<<"got pointer"<<std::endl;
	  //std::cout.flush();
      
	  // We lose the source information, but a pull always knows
	  // what the source of the data is anyway
	  _data = new FrameworkLocalData<T>("n/a over remote pull",p_data); 
      
	  //must deallocate return value

	  delete p_a;


	  //std::cout<<"****************** OK end"<<std::endl;
	}
      }
      catch(const CORBA::Exception & e) {
	std::cerr<<"CORBA::Exception caught in RemotePullSenderImpl::pull()"<<std::endl;
	std::cerr<<e._name()<<std::endl;
	std::cerr<<"typename: "<<BALTType::typeName<T>()<<std::endl;
	std::cerr<<_query.getSource()<<" sent "<<_query.getQuery()<<std::endl;
	std::abort();//what error
      }
      catch(std::exception & e) {
	std::cerr<<"Std::Exception caught in RemotePullSenderImpl::pull()"<<std::endl;
	std::cerr<<e.what()<<std::endl;
	std::cerr<<"typename: "<<BALTType::typeName<T>()<<std::endl;
	std::cerr<<_query.getSource()<<" sent "<<_query.getQuery()<<std::endl;
	std::abort();//what error
      }
    }
    

  //RemotePullSender methods...
  virtual void setPullConnector(RemoteConnectors::RemotePullConnector_ptr _rpc)  {
    
    /// \todo THIS CLASS IS IN CHARGE OF THE MEMORY FOR m_out/_rpc!!!
    //std::cout<<"RemotePullSenderImpl::setPullConnector"<<std::endl;
    m_out = _rpc;
  }

    /**
     * Sets the translator object used by this connector. This memory
     * is then owned by this objects and could be deleted at any time!
     * This also destroys any previously stored translator.
     */
  void setTranslator(RemoteDataTranslator<T> * _pTranslator) {
    if(m_pTranslator) {
      delete m_pTranslator;
    }
    m_pTranslator = _pTranslator;
  }
  
private:
  RemoteConnectors::RemotePullConnector_ptr m_out;
    RemoteDataTranslator<T> * m_pTranslator;
    
};



/**
 * Class to receive pulls from remote source.
 */
template <class T>
class RemotePullReceiverImpl : 
  public POA_RemoteConnectors::RemotePullReceiver,
  public FrameworkConnector,
  public PullConnectorRegister<T> {
  
public:

  /**
   * Constructor.
   */
  RemotePullReceiverImpl(const std::string & _id) : FrameworkConnector(_id) {
    m_pTranslator = NULL;
    m_pPullReceiver = NULL;
  }
      
  /**
   * Destructor. Deletes translator.
   */
  virtual ~RemotePullReceiverImpl() {
    delete m_pTranslator;
  }
      
  //PullConnectorRegister methods...

  /**
   * Set the pull receiver for this connection.
   */
  virtual void registerPullReceiver(PullReceiver<T> * _pPr) {
    m_pPullReceiver = _pPr;
  }

  //RemotePullReceiver methods...
  /**
   * Receive a pull query from a remote source.
   */
  virtual CORBA::Any * receivePullQuery(const char* _src, const char* _query) {
    
    
      //std::cerr<<"RemotePullReceiverImpl::receivePullQuery "<<_src<<std::endl;

      if(!m_pPullReceiver) {
	std::cerr<<"registerPullReceiver not called, or pointer connector is null"<<std::endl;
	return new CORBA::Any(); //return null-valued any
      }
      

      CORBA::Any *p_a = new CORBA::Any();

     try {

    
      //create a new query object from the input
      FrameworkQuery fq(_src,_query);
    
      //create an empty data object
      FrameworkLocalData<T> *fd;
    
      //std::cerr<<"1"<<std::endl;
      //get the data
      m_pPullReceiver->receivePullQuery(fq,fd);
      //std::cerr<<"2"<<std::endl;
      //create a new any objecy
      
      //std::cerr<<"3"<<std::endl;

      //translate datatype into the any
      if(!m_pTranslator) {
	std::cerr<<"NOT GOOD"<<std::endl;
      }
      if(!fd) {
	std::cerr<<"NOT GOOD EITHER"<<std::endl;
      }
      if(!fd->data()) {
	std::cerr<<"NOT GOOD EITHER EITHER"<<std::endl;
      }
      
      m_pTranslator->translate(fd->data(),(*p_a));

      //std::cerr<<"4"<<std::endl;
      delete fd;
      fd = NULL;
      //std::cerr<<"5"<<std::endl;
      //std::cerr<<"RemotePullReceiverImpl::registerPullReceiver END"<<_src<<std::endl;
    } 
    catch(std::exception & e) {
      std::cerr<<"Std::Exception caught in RemotePullSenderImpl::receivePullQuery()"<<std::endl;
      std::cerr<<e.what()<<std::endl;
      std::abort();//what error
    }
    return p_a;    

  }

  /**
   * Sets the translator object used by this connector. This memory
   * is then owned by this objects and could be deleted at any time!
   * This also destroys any previously stored translator.
   */
  void setTranslator(RemoteDataTranslator<T> * _pTranslator) {

    //std::cout<<"RemoteDataTranslator::setTranslator"<<std::endl;
    //std::cout<<typeid(T).name()<<std::endl;
    
    if(m_pTranslator) {
      delete m_pTranslator;
    }

    m_pTranslator = _pTranslator;
  }


private:
    PullReceiver<T> * m_pPullReceiver;
    RemoteDataTranslator<T> * m_pTranslator;
};

#endif
