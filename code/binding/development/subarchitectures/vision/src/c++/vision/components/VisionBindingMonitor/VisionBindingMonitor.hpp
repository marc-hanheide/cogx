#ifndef VISUAL_SCENE_MONITOR_H_
#define VISUAL_SCENE_MONITOR_H_

#include <binding/idl/BindingData.hh>
#include <binding/abstr/AbstractMonitor.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <vision/idl/Vision.hh>

using namespace cast; //default useful namespaces, fix to reflect your own code
using namespace std;
using namespace boost;
using namespace Vision;
//using namespace BindingFeatures;

namespace Binding {


  /**
   * A class that generates binding proxies for the vision
   * subarchitecture. The general operation is to wait for the scene to
   * become static, then create or update proxies for any
   * SceneObjects in working memory.
   *
   * When the scene is static the monitor pulls all scene objects and
   * creates proxies for them. While the scene is not changing it
   * then waits for any updates to these objects and updates the
   * proxies accordingly. Once the scene starts changing again the
   * updates are ignored. If an object is no longer present in the wm
   * when the scene settles it's proxy is deleted.
   **/


  //fwd declaration
  //
  class ProxyMonitor;

  class VisionBindingMonitor : public AbstractMonitor { //,
//			       public AbstractBindingWMRepresenter {
  public:
    VisionBindingMonitor(const string &_id);
    
    /// overridden start method used to set filters
    virtual void start();
    virtual void configure(map<string,string> & _config);
  
    friend class ProxyMonitor;

  protected:
    void taskAdopted(const string &_taskID){}
    void taskRejected(const string &_taskID){}
    void runComponent();

    bool retrieveProxy(const cdl::WorkingMemoryAddress & _wma);

    bool hasShape(const Vision::SceneObject & _so);
    bool hasColour(const Vision::SceneObject & _so);
    bool hasSize(const Vision::SceneObject & _so);
    //nah: added method to add concept from label
    void addConceptToProxy(const Vision::SceneObject & _so);

    void addColourToProxy(const Vision::SceneObject & _so);
    void addShapeToProxy(const Vision::SceneObject & _so);
//    void addSizeToProxy(const Vision::SceneObject & _so);
//    void addColourMotive(const Vision::SceneObject & _so);
//    void addShapeMotive(const Vision::SceneObject & _so);

    void processScene();
    void objectUpdated(const cdl::WorkingMemoryChange& _wmc);
    void objectAdded(const cdl::WorkingMemoryChange& _wmc);
    void objectDeleted(const cdl::WorkingMemoryChange& _wmc);
    void sceneChanged(const cdl::WorkingMemoryChange& _wmc);
//    void relationAdded(const cdl::WorkingMemoryChange& _wmc);
//    void relationUpdated(const cdl::WorkingMemoryChange& _wmc);
    void handPointing(const cdl::WorkingMemoryChange& _wmc);

    void propertyUpdates(const cdl::WorkingMemoryChange& _wmc);

    void closeLastSaliencyInterval();

    /**
     * Create a new proxy for the scene object. Opens it in the
     * abstract monitor, BUT does not store it.
     */
    void newProxy(const Vision::SceneObject & _so,
		  const std::string & _soID);

    /**
     * Adds a new proxy representing the location of the object, and
     * relates it to the proxy with an "at" relation.
     */
    void addLocationToProxy(const std::string & _soProxyID,
			    const Vision::SceneObject & _so);

    void deleteProxy(const Vision::SceneObject & _so,
		     const string & _proxyAddr);

    void processFeatureTransfer(BindingData::BindingFeatureTransfer _ft);


    

    // GUI methods
    virtual void redrawGraphicsText();

    /**
     * Create a new monitor struct for this proxy.
     */
    void monitorNewProxy(const std::string _proxyID);


    /// called by \p makeProxyUnavailableAdded if the \p
    /// \p BindingQueries::MakeProxyUnavailable belongs to this monitor. It will simply
    /// call \p deleteExistingProxy() right away.
    /// \remark override this function if immediate deletion is not appropriate
    virtual void makeProxyUnavailable(const BindingQueries::MakeProxyUnavailable&) {}


    /**
     * Request a feature value via motivation. E.g. what is the colour
     * of?
     *
     * @param  _proxyID the proxy to which the query is related
     */
    template <class FeatureType> void queryFeatureValue(const std::string _proxyID) {
      const string & type(typeName<FeatureType>());
      
      BindingQueries::FeatureRequest * req = new BindingQueries::FeatureRequest();
      req->m_request.m_parameters.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
      req->m_request.m_proxyID = CORBA::string_dup(_proxyID.c_str());

      //important bit
      //type of feature
      req->m_request.m_featurePointer.m_type = CORBA::string_dup(type.c_str());
      //address blank so that it's just a type-based call, no value
      req->m_request.m_featurePointer.m_address =  CORBA::string_dup("");
      req->m_request.m_featurePointer.m_immediateProxyID  = CORBA::string_dup("");
      req->m_request.m_answer = cast::cdl::triIndeterminate;
      req->m_request.m_processed = false;
      req->m_fromSA = CORBA::string_dup(getSubarchitectureID().c_str());

      //write to motive wm... HACK handcoded motive name
      addToWorkingMemory(newDataID(), "motiv.sa", req);
    }


    /**
     * Respond to actions from the planner
     */
    void newAction(const cdl::WorkingMemoryChange& _wmc);

    map<string,string> m_sourceProxyMapping;
    map<string, ProxyMonitor *> m_proxyMonitorMap;

    // Whether to run the feature transfer code or not... default to true
    bool m_transferFeatures;

    // Whether to do a dummy clarification, default false
    bool m_dummyClarify;

    
    //    bool m_sceneChanging;

    // Is there a salient SceneObject
    bool m_anythingSalient;
    // The address of the salient object's proxy
    string m_salientAddr;
    // The feature pointer to the salience feature of the salient object's proxy
    BindingData::FeaturePointer m_sailienceFeatPtr;


    //id of the robot proxy
    string m_robotProxyID;

  private:

    //cache the scene objects
    CASTDataCache<Vision::SceneObject> m_soCache;
    CASTDataCache<Vision::HandPointingResults> m_handPointingCache;

    /**
     * A struct for storing the ids of the relation and location proxies
     * associated with each scene object proxy
     */
    struct ObjectLocationRelation {
      std::string m_objProxyID;
      std::string m_relProxyID;
      std::string m_locationProxyID;    
    };

    typedef StringMap<ObjectLocationRelation>::map OPRMap;

    OPRMap m_locations;

    /**
     * Update the location proxy associated with this so proxy
     */
    bool updateLocation(const Vision::SceneObject & _so,
			const string & _proxyAddr);

//     template<class T>
//     const boost::shared_ptr<const T> getProxyFeature(shared_ptr<const BindingData::BindingProxy> _proxy) {
//       std::string type(typeName<T>());

//       for(unsigned int i = 0; 
// 	  i < _proxy->m_proxyFeatures.length(); ++i) {

// 	if(string(_proxy->m_proxyFeatures[i].m_type) == type) {
// 	  return getWorkingMemoryEntry<T>(string(_proxy->m_proxyFeatures[i].m_address),
// 					  bindingSA())->getData();
// 	}

//       }

//       //return null if not found
//       return boost::shared_ptr<const T>();

//     }



  }; // Abstract monitor
} // namespace Binding

#endif //VISUAL_SCENE_MONITOR_H_

