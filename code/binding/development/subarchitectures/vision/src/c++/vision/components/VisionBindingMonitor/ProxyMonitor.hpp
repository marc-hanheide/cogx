#ifndef PROXY_MONITOR_H_
#define PROXY_MONITOR_H_

#include <map>
#include <vector>
#include <binding/abstr/AbstractMonitor.hpp>


#include "VisionBindingMonitor.hpp"

using namespace cast;
using namespace std;
using namespace boost;

namespace Binding {

  class ProxyMonitor : public WorkingMemoryChangeReceiver {

  public:
    ProxyMonitor(const std::string & _proxyID,
                 const std::string & _bindingSA,
		 VisionBindingMonitor * _pParent);

    ~ProxyMonitor();

    virtual void workingMemoryChanged(const cdl::WorkingMemoryChange & _wmc);
    virtual bool needsDeleting() {return m_bDeleteMe;}

    std::vector<BindingData::BindingFeatureTransfer> getFeatureTransfers() const;
    void clearFeatureTransfers();
    void addLearnableConcept(string _featureType);

    //called when the proxy is deleted
    void cleanUpMonitor();



    bool proxyIsBound() const;
    bool proxyWasBound() const;

    /**
     * Check to see whether this has been bound to anything, and store
     * result if positive.
     */
    bool hasBindingChanged();
    bool hasRelationChanged();

  protected:

    void monitorProxy(const cdl::WorkingMemoryChange & _wmc);
    void monitorUnion(const cdl::WorkingMemoryChange & _wmc);

    void removeUnionFilter(const std::string & _unionID);
    void addUnionFilter(const std::string & _unionID);

    void updateProxyPointer();
    void updateUnionPointer();

    void lookForLearningOpportunities();

    bool hasBindingFeature(const shared_ptr<const BindingData::BindingProxy> & _proxy,
			   const string & _feature);

    BindingData::FeaturePointer getBindingFeature(const shared_ptr<const BindingData::BindingUnion> & _union,
						  const string & _feature);



    bool m_bDeleteMe;
    // HACK
    std::string m_relationProxyID;
    
    std::string m_unionID;
    std::string m_lastUnionID;
    std::string m_proxyID;
    std::string m_bindingSA;

    //a shared ptr to the proxy itself... won't get updated if on a
    //remote machine
    boost::shared_ptr<const BindingData::BindingProxy> m_proxy;

    //a shared ptr to the current bound union... won't get updated if
    //on a remote machine
    boost::shared_ptr<const BindingData::BindingUnion> m_union;

    //a list of binding features that could potentially be added to
    //the monitored proxy
    std::vector<BindingData::BindingFeatureTransfer> m_featureTransfers;
    std::vector<string> m_learnedFeatures;
    std::vector<string> m_learnedConcepts;

    /////////
    // App specific stuff???
    /////////

    //a list of the things that this proxy can learn from other proxies
    vector<string> m_learnableConcepts;


    //probably a neater way of doing this, but hey...
    VisionBindingMonitor * m_pParent;

    //silly method 'cos I can't remeber teh stl way with the interweb
    template<class T>
    bool contains(const std::vector<T> & _vector, const T & _item)
    {
      for(typename std::vector<T>::const_iterator i = _vector.begin();
        i < _vector.end(); ++i)
      {
        if(_item == *i)
          return true;
      }

      return false;
    }

  private:
    std::string thisProxyIDStr();
    WorkingMemoryChangeReceiver*    m_unionChangeReceiver;
    

  };
}


#endif //PROXY_MONITOR_H_
