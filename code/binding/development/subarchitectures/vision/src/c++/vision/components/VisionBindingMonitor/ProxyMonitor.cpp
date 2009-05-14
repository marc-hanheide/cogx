#include <cast/architecture/ChangeFilterFactory.hpp>
#include <iostream>

#include "ProxyMonitor.hpp"

using namespace cast;
using namespace cast::cdl;
using namespace boost;
using namespace std;
using namespace BindingData;
using namespace Vision;

namespace Binding {

  ProxyMonitor::ProxyMonitor(const string & _proxyID,
      const string & _bindingSA,
      VisionBindingMonitor * _pParent) :
    m_bDeleteMe(false),
    m_relationProxyID(""),
    m_unionID(""),
    m_proxyID(_proxyID),
    m_bindingSA(_bindingSA),
    m_pParent(_pParent),
    m_unionChangeReceiver( new MemberFunctionChangeReceiver<ProxyMonitor>(this,
                               &ProxyMonitor::monitorUnion) )
  {
    m_pParent->debug(thisProxyIDStr() + ": Initializating proxy monitor...");
  }
  
  ProxyMonitor::~ProxyMonitor()
  {
    delete m_unionChangeReceiver;
  }


  void ProxyMonitor::workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc) {
    
    monitorProxy(_wmc);
/*
    string changeType(_wmc.m_type);

    if(changeType == typeName<BindingProxy>()) {
      monitorProxy(_wmc);
    }
    else if(changeType == typeName<BindingUnion>()) {
      monitorUnion(_wmc);
    }
*/
  }


  void ProxyMonitor::cleanUpMonitor() {
    m_pParent->log("ProxyMonitor::cleanUpMonitor: There are %i proxies in the union.", m_union->m_proxyIDs.length());
      
    for(unsigned int i = 0; i < m_union->m_proxyIDs.length(); ++i) {  
      m_pParent->log("Proxy <%s> is also in the union.",
		     CORBA::string_dup(m_union->m_proxyIDs[i]));  
      if(string(m_union->m_proxyIDs[i]) != m_proxyID)
	{
	  m_pParent->log("Making proxy ID <%s> unavailable.",
			 CORBA::string_dup(m_union->m_proxyIDs[i]));
	  
	  // Create a MakeProxyUnavailable object and add it to WM.
	  BindingQueries::MakeProxyUnavailable* mpu
	    = new BindingQueries::MakeProxyUnavailable();
	  
	  mpu->m_proxyID = CORBA::string_dup(m_union->m_proxyIDs[i]);
	  
	  m_pParent->addToWorkingMemory<BindingQueries::MakeProxyUnavailable>
	    (m_pParent->newDataID(), m_bindingSA, mpu);
	}
    }
    
    m_bDeleteMe = true;
    m_pParent->debug("Proxy monitor scheduled for deletion.");
    
    if(proxyIsBound()) {
      removeUnionFilter(m_unionID);
    }

  }

  void ProxyMonitor::monitorProxy(const cdl::WorkingMemoryChange & _wmc) {
    //m_pParent->println("MONITOR PROXY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    if(!m_bDeleteMe) {
      if(_wmc.m_operation == cdl::DELETE) {
	cleanUpMonitor();
	return;
      }
      m_pParent->debug("Proceed....");
      //retrieve latest version
      updateProxyPointer();
      
      //check to see if the binding has changed
      if(hasBindingChanged()) {
	m_pParent->log("ProxyMonitor::Binding has changed for proxy at: %s", m_proxyID.c_str());
	
	if(proxyWasBound()) {
	  m_pParent->log("ProxyMonitor::Ignoring old union: %s", m_lastUnionID.c_str());
	  removeUnionFilter(m_lastUnionID);
	}
	
	if(proxyIsBound()) {
	  m_pParent->log("ProxyMonitor::Monitoring new union: %s", m_unionID.c_str());
	  addUnionFilter(m_unionID);
	  
	  //trigger the union monitoring code using a fake change event
	  WorkingMemoryChange unionChange;
	  unionChange.m_type = CORBA::string_dup(typeName<BindingUnion>().c_str());
	  unionChange.m_src = CORBA::string_dup("");
	  unionChange.m_operation = cdl::ADD;
	  unionChange.m_address.m_id = CORBA::string_dup(m_unionID.c_str());
	  unionChange.m_address.m_subarchitecture = CORBA::string_dup(m_bindingSA.c_str());
	  monitorUnion(unionChange);
	}
      }
      
    }
  }
  
  
  void ProxyMonitor::addLearnableConcept(string _featureType)
  {
    m_learnableConcepts.push_back(_featureType);
    m_pParent->debug(_featureType + "added as learnable concept.");
  }


  void ProxyMonitor::addUnionFilter(const std::string & _unionID) {
    m_pParent->addChangeFilter(createAddressFilter(_unionID,
                               m_bindingSA),
                               m_unionChangeReceiver);
/*-    m_pParent->addChangeFilter(BindingOntology::BINDING_UNION_TYPE,
			       cdl::DELETE,
			       "",
			       _unionID,
			       m_bindingSA,
			       false,
			       this);
*/
  }

  void ProxyMonitor::removeUnionFilter(const std::string & _unionID) {
    m_pParent->removeChangeFilter(m_unionChangeReceiver);
/*    m_pParent->removeChangeFilter(BindingOntology::BINDING_UNION_TYPE,
				  cdl::DELETE,
				  "",
				  _unionID,
				  m_bindingSA,
				  false);
*/
  }


  void ProxyMonitor::monitorUnion(const cdl::WorkingMemoryChange & _wmc) {

    m_pParent->debug("ProxyMonitor::Union changed...");

    if(_wmc.m_operation == cdl::DELETE)
    {
      removeUnionFilter(string(_wmc.m_address.m_id));
      m_pParent->debug("ProxyMonitor::Union deleted, wait for new union.");
      return;
    }
//  m_pParent->debug("Proceed...");
    //just ignore things moving before changes
    try
    {
      updateUnionPointer();
    }
    catch (DoesNotExistOnWMException &e)
    {
      m_pParent->println("PROXY MONITOR: Union has gone, not been told to delete yet...");
    }

    /////////
    // App specific stuff???
    /////////

    lookForLearningOpportunities();
  }

  bool ProxyMonitor::hasBindingFeature(const shared_ptr<const BindingData::BindingProxy> & _proxy,
				       const string & _featureType) {

    for(unsigned int i = 0; i < _proxy->m_proxyFeatures.length(); ++i)
    {
      string featureType(_proxy->m_proxyFeatures[i].m_type);

      if(_featureType == featureType)
        return true;

    }
    return false;
  }

  BindingData::FeaturePointer ProxyMonitor::getBindingFeature(const shared_ptr<const BindingData::BindingUnion> & _union,
							      const string & _featureType) {

    for(unsigned int i = 0; i < _union->m_unionFeatures.length(); ++i)
    {
      string featureType(_union->m_unionFeatures[i].m_type);

      if(_featureType == featureType)
        return _union->m_unionFeatures[i];
    }
    //TODO make this nicer
    throw(BindingException("Missing requested feature " + _featureType));
  }


  void ProxyMonitor::lookForLearningOpportunities() {

    assert(m_union.get() != NULL);

    m_pParent->debug(thisProxyIDStr() + ": looking for learning opportunities... ");

    //if other proxies are bound in with the monitored one
    if(m_union->m_proxyIDs.length() > 1) {
      m_pParent->debug(thisProxyIDStr() + ": other proxies are bound to the union... ");
      // collect a list of features from the union that can be
      // transferred to proxy
      vector<string> learnConcepts;

      for(unsigned int i = 0; i < m_union->m_unionFeatures.length(); ++i)
      {
        string featureType(m_union->m_unionFeatures[i].m_type);
        string featureID(m_union->m_unionFeatures[i].m_address);
        
        m_pParent->debug(thisProxyIDStr() + ": union contains feature type " +  featureType);

        // if it's a learnable concept and not yet learned, store it for later
        if (contains<string>(m_learnableConcepts,featureType)
          && ! contains<string>(m_learnedConcepts,featureType)
          && ! contains<string>(m_learnedFeatures,featureID))
        {
          // Add to learning candidates' list
          learnConcepts.push_back(featureType);
          
          // Do not learn the same feature over and over
          m_learnedConcepts.push_back(featureType);         
          m_learnedFeatures.push_back(featureID);

          m_pParent->debug(thisProxyIDStr() + ": potential learning opportunity: " + string(featureType));
        }
      }


      //TODO, make more general using binding functions

      for(vector<string>::const_iterator i = learnConcepts.begin();
        i < learnConcepts.end(); ++i)
        {
//          if (!hasBindingFeature(m_proxy, *i))
          {
            m_pParent->debug(thisProxyIDStr() + ": learning opportunity: " + string(*i) +
              " not part of proxy");

            BindingFeatureTransfer transfer;
            transfer.m_feature = getBindingFeature(m_union, *i);
            transfer.m_source.m_id = CORBA::string_dup(m_unionID.c_str());
            transfer.m_source.m_subarchitecture = CORBA::string_dup(m_bindingSA.c_str());
            transfer.m_target.m_id = CORBA::string_dup(m_proxyID.c_str());
            transfer.m_target.m_subarchitecture = CORBA::string_dup(m_bindingSA.c_str());
	         transfer.m_bindingSubarchitectureID = CORBA::string_dup(m_bindingSA.c_str());

            m_featureTransfers.push_back(transfer);

            m_pParent->debug(thisProxyIDStr() + ": ADDED " +  string(*i) + " to transfer list.");
          }
        }

    }
  }

  std::vector<BindingData::BindingFeatureTransfer> ProxyMonitor::getFeatureTransfers() const {
    return m_featureTransfers;
  }

  void ProxyMonitor::clearFeatureTransfers() {
    m_featureTransfers.clear();
  }


  bool ProxyMonitor::proxyIsBound() const {
    return m_unionID != "";
  }


  bool ProxyMonitor::proxyWasBound() const {
    return m_lastUnionID != "";
  }


  bool ProxyMonitor::hasBindingChanged()
  {
    //get the unew union addr
    string newUnionID(m_proxy->m_unionID);
    //if it's the same as the old binding then no change has happened
    if(newUnionID == m_unionID)
      return false;
    else
    {
      // store last union address
      m_lastUnionID = m_unionID;
      //store new union address
      m_unionID = newUnionID;
      //and say yes
      return true;
    }
  }

  void ProxyMonitor::updateProxyPointer()
  {
    shared_ptr<const CASTData<BindingProxy> > pProxyData =
      m_pParent->getWorkingMemoryEntry<BindingProxy>(m_proxyID,m_bindingSA);

    m_proxy = pProxyData->getData();
  }


  void ProxyMonitor::updateUnionPointer()
  {
    shared_ptr<const CASTData<BindingUnion> > pUnionData =
      m_pParent->getWorkingMemoryEntry<BindingUnion>(m_unionID,m_bindingSA);

    m_union = pUnionData->getData();
  }

  std::string ProxyMonitor::thisProxyIDStr()
  {
    return "PROXY[" + string(m_proxyID) +"]";
  }

}
