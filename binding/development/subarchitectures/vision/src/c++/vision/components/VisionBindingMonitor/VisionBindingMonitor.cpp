#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <binding/feature-specialization/helpers/SalienceHelper.hpp>
#include <motivation/util/CompetenceRegistration.hpp>

#include <planning/idl/PlanningData.hh>

#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>

#include "VisionBindingMonitor.hpp"
#include "ProxyMonitor.hpp"

using namespace Vision;

namespace Binding {

  using namespace boost;
  using namespace boost::assign;
  using namespace planning::autogen;
  
  VisionBindingMonitor::VisionBindingMonitor(const string& _id) :
    WorkingMemoryAttachedComponent(_id),
    AbstractMonitor(_id),
//    AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this)),
//    m_sceneChanging(true), //start on true as we don't know what's going on
    m_anythingSalient(false),
    m_robotProxyID(""),
    m_soCache(*this),
    m_handPointingCache(*this) {

    setReceiveXarchChangeNotifications(true);

  }


  void
  VisionBindingMonitor::start() {

    AbstractMonitor::start();

    m_sourceID = m_subarchitectureID;


    addChangeFilter(createLocalTypeFilter<HandPointingResults>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
                        &VisionBindingMonitor::handPointing));

//     addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::ADD),
// 		    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
// 									   &VisionBindingMonitor::objectAdded));

//     addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
// 		    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
// 									   &VisionBindingMonitor::objectUpdated));

    
    addChangeFilter(createLocalTypeFilter<SceneObjectUpdate>(cdl::ADD),
 		    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
 									   &VisionBindingMonitor::propertyUpdates));

    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::DELETE),
		    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
									   &VisionBindingMonitor::objectDeleted));

    //listen for actions from the planner
    addChangeFilter(createLocalTypeFilter<Action>(cdl::ADD), 
		    new MemberFunctionChangeReceiver<VisionBindingMonitor>(this,
									   &VisionBindingMonitor::newAction));


                              
  }


  /**
   * Respond to actions from the planner
   */
  void VisionBindingMonitor::newAction(const cdl::WorkingMemoryChange& _wmc) {
    //we don't care, just overwrite with success
    
    Action * action = new Action(*(getWorkingMemoryEntry<Action>(_wmc.m_address)->getData()));        
    log("responding positively to an action: %s", string(action->m_action.m_type).c_str());
    action->m_status = planning::autogen::COMPLETE;
    action->m_succeeded = cdl::triTrue;
    overwriteWorkingMemory(_wmc.m_address, action);
    action = NULL;
  }


  void 
  VisionBindingMonitor::addLocationToProxy(const std::string & _soProxyID,
					   const Vision::SceneObject & _so) {

    //add the location
    startNewBasicProxy();
    BindingFeatures::Location location;
    location.m_location = _so.m_bbox.m_centroid; 
    addFeatureToCurrentProxy(location);
    addOtherSourceIDToCurrentProxy(m_sourceID, BindingFeaturesCommon::NEGATIVE);
    BindingFeatures::TemporalFrame tf;
    tf.m_temporalFrame = BindingFeaturesCommon::PERCEIVED;
    addFeatureToCurrentProxy(tf);
    string locationID = storeCurrentProxy();
    


    //then link via a relation
    //leave out relations for the time being
    startNewRelationProxy();

    BindingFeatures::TemporalFrame tfr;
    tfr.m_temporalFrame = BindingFeaturesCommon::PERCEIVED;
    addFeatureToCurrentProxy(tfr);

    BindingFeatures::RelationLabel label;
    label.m_label = CORBA::string_dup("pos");
    addFeatureToCurrentProxy(label); 
    addOutPortToCurrentProxy(locationID, "from");
    addOutPortToCurrentProxy(_soProxyID, "to");
    addOtherSourceIDToCurrentProxy(m_sourceID, BindingFeaturesCommon::NEGATIVE);
    string relationID = storeCurrentProxy();

    //store the details for the relation for later management tasks
    m_locations[_soProxyID].m_objProxyID = _soProxyID;
    m_locations[_soProxyID].m_relProxyID = relationID;
    m_locations[_soProxyID].m_locationProxyID = locationID;

  }

//   void
//   VisionBindingMonitor::objectAdded(const cdl::WorkingMemoryChange& _wmc) {

//     //get the object
//     const Vision::SceneObject & sceneObject(m_soCache[string(_wmc.m_address.m_id)]);

//     string proxyAddr = newProxy(sceneObject, _wmc);
//     addLocationToProxy(proxyAddr, sceneObject);
//     log("New proxy: " + proxyAddr);


//     if(m_transferFeatures) {
//       //also start monitoring the proxy
//       ProxyMonitor *pProxyMonitor = new ProxyMonitor(proxyAddr,
// 						     getBindingSA(),
// 						     this);

//       pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Colour>());
//       pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Shape>());
//       pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Size>());
// //      pProxyMonitor->m_learnableConcepts.push_back(BindingOntology::SIZE_TYPE);

//       //add a change filter to monitor that particular proxy
//       addChangeFilter(createAddressFilter(proxyAddr,
//                       getBindingSA()),
//                       pProxyMonitor);


//       //store for later
//       m_proxyMonitorMap[proxyAddr] = pProxyMonitor;      
//     }

//     bindNewProxies();
//   }
  
  /// 
  void VisionBindingMonitor::handPointing(const cdl::WorkingMemoryChange& _wmc) {

    //get the object
    const Vision::HandPointingResults & hpResult(m_handPointingCache[string(_wmc.m_address.m_id)]);
    string objID = string(hpResult.m_SceneObjAddress);

    
    if(!hpResult.m_pointerDetected) {
      debug("Pointer not detected");
      return;
    }
    
    //get the proxy address for the object
    map<string,string>::iterator i =
        m_sourceProxyMapping.find(objID);

    if(i == m_sourceProxyMapping.end()) {
      log("No proxy for pointed object");
      return;
    }

    string proxyAddr = i->second;
    
    if(proxyAddr == m_salientAddr) {
      log("Pointed object's proxy already salient (object ID %s)", objID.c_str());
      return;
    }


    log("Updating SALIENCE for proxy ID %s (object ID %s)",proxyAddr.c_str(), objID.c_str());

    changeExistingProxy(proxyAddr);

    // Add an open salience interval
    const FrameworkBasics::BALTTime time(BALTTimer::getBALTTime());
    BindingData::FeaturePointer salienceFeatPtr =
        addSalienceToCurrentProxy(startTime(time), infiniteFuture());
    storeCurrentProxy();

    // Deal with the saliency...
    if(m_anythingSalient)
    {
        // Close the saliency interval of the last salient SceneObject proxy
      changeExistingProxy(m_salientAddr);
      BindingFeaturesCommon::StartTime
          start(getWorkingMemoryEntry<BindingFeatures::Salience>(string(m_sailienceFeatPtr.m_address),getBindingSA())->getData()->m_start);
      addSalienceToCurrentProxy(start, endTime(time));
      deleteFeatureFromCurrentProxy(m_sailienceFeatPtr);
      storeCurrentProxy();
    }
    else
      m_anythingSalient = true;

    // Remember the new SceneObject proxy as the currently salient
    m_salientAddr = proxyAddr;
    m_sailienceFeatPtr = salienceFeatPtr;

  }


  void
  VisionBindingMonitor::objectDeleted(const cdl::WorkingMemoryChange& _wmc) {

    //get the proxy address for the object
    string soID(_wmc.m_address.m_id);

    map<string,string>::iterator i =
      m_sourceProxyMapping.find(soID);

    if(i == m_sourceProxyMapping.end()) {
      //throw(BindingException("Cannot find proxy for deleted object"));
      debug("no proxy for so <%s>, not deleting. Hopefully this was a hand", soID.c_str());
      return;
    }
  
    const string & proxyID(i->second);


    //stop monitoring
    assert(m_proxyMonitorMap.find(proxyID) != m_proxyMonitorMap.end());
    m_proxyMonitorMap[proxyID]->cleanUpMonitor();

    // If the object was salient, nothing is salient now
    // (alternatively the most recently salient object (if exists) could be made salient)
    if(proxyID == m_salientAddr)
      m_anythingSalient = false;

    assert(m_locations.find(proxyID) != m_locations.end());

    debug("deleting proxy rel proxy for so <%s>: %s", soID.c_str(), m_locations[proxyID].m_relProxyID.c_str());
    // Delete the relation proxy first (binder bug)  
    deleteExistingProxy(m_locations[proxyID].m_relProxyID);
    
    debug("deleting proxy for so <%s>: %s", soID.c_str(), proxyID.c_str());
    // Delete scene object proxy
    deleteExistingProxy(proxyID);

    debug("deleting location proxy for so <%s>: %s", soID.c_str(), m_locations[proxyID].m_locationProxyID.c_str());
    // Now delete the location proxies
    deleteExistingProxy(m_locations[proxyID].m_locationProxyID);
    
    m_locations.erase(proxyID);
    m_sourceProxyMapping.erase(i);

  }

/*
  void
  VisionBindingMonitor::sceneChanged(const cdl::WorkingMemoryChange& _wmc) {


    shared_ptr<const CASTData<Vision::SceneChanged> > sceneChanged_ptr
      = getWorkingMemoryEntry<Vision::SceneChanged>(_wmc.m_address);
    shared_ptr<const Vision::SceneChanged> sceneChanged = sceneChanged_ptr->getData();

    if(sceneChanged->m_sceneChanging) {
      log("Scene changing");
      //      m_sceneChanging = true;
    }
    else if(sceneChanged->m_sceneChanged) {
      log("Scene changed");
      //      m_sceneChanging = false;
      //    processScene();
    }
    else if(sceneChanged->m_sceneProcessed) {
      log("Scene processed");
 //     bindNewProxies();
    }
  }
*/

  void
  VisionBindingMonitor::newProxy(const Vision::SceneObject & _so,
				 const std::string & _soID)
  {
    log("Create proxy for Scene Object: <" + _soID + ">");

    startNewBasicProxy();

    BindingFeatures::TemporalFrame tf;
    tf.m_temporalFrame = BindingFeaturesCommon::PERCEIVED;
    addFeatureToCurrentProxy(tf);

    BindingFeatures::SourceData sd;
    sd.m_type = CORBA::string_dup(typeName<SceneObject>().c_str());
    sd.m_address.m_id = CORBA::string_dup(_soID.c_str());
    sd.m_address.m_subarchitecture = CORBA::string_dup(m_sourceID.c_str());
    addFeatureToCurrentProxy(sd);
     
    //add a negeated source id to stop it binding with other visual
    //objects
    addOtherSourceIDToCurrentProxy(m_sourceID, BindingFeaturesCommon::NEGATIVE);

    addConceptToProxy(_so);

//     // add this label string thingy just to see what it is: (sorry for ugliness....)
//     // BindingData::Feature::DebugString debug;
//     // debug.m_debugString = CORBA::string_dup(string(string("\\\"") +
//     // string(_so->m_label.m_string) + "\\\" (confidence: " + 
//     // lexical_cast<string>(_so->m_label.m_confidence) + ")").c_str());
//     // addFeatureToCurrentProxy(debug);


  }

  void
  VisionBindingMonitor::closeLastSaliencyInterval() {

    // Deal with the saliency...
    if(m_anythingSalient) { 
      // Close the saliency interval of the last salient SceneObject proxy
      changeExistingProxy(m_salientAddr);
      const FrameworkBasics::BALTTime time(BALTTimer::getBALTTime());
      BindingFeaturesCommon::StartTime
	start(getWorkingMemoryEntry<BindingFeatures::Salience>(string(m_sailienceFeatPtr.m_address),getBindingSA())->getData()->m_start);
      addSalienceToCurrentProxy(start, endTime(time));
      deleteFeatureFromCurrentProxy(m_sailienceFeatPtr);
      storeCurrentProxy();
    }
    else {
      m_anythingSalient = true;
    }

  }

  bool VisionBindingMonitor::updateLocation(const Vision::SceneObject & _so,
					    const string & _proxyAddr) {

    OPRMap::iterator i(m_locations.find(_proxyAddr));

    assert(i != m_locations.end());

    string locationProxyAddr(i->second.m_locationProxyID);

    //load the location proxy associated with the scene object proxy
    const ProxyPtr& locationProxy(m_proxyLocalCache.get(locationProxyAddr).getLocalPtr());
    
    //now get the location feature
      
    const BindingFeatures::Location & location(locationProxy->getFeature<BindingFeatures::Location>());

    //get the two locations to be compared
    double dist = Vision::distance(_so.m_bbox.m_centroid,
				   location.m_location);

    //compare to the bbox width of the object
    if(dist <  _so.m_bbox.m_size.m_x) {
      debug("location not updated for proxy: %s", locationProxyAddr.c_str());
      return false;
    }


    //delete old location
    set<string> del;
    del.insert(typeName<BindingFeatures::Location>());
    
    changeExistingProxy(locationProxyAddr, del);
    
    
    //then add the new one
    BindingFeatures::Location newLocation;
    newLocation.m_location = _so.m_bbox.m_centroid; 
    addFeatureToCurrentProxy(newLocation);
    //and store

    //sanity reload just in case
    try {
      storeCurrentProxy();
    }
    catch (const ConsistencyException & e) {
      println("it were here: %s",e.what());
      abort();
    }


    log("location updated for proxy: %s", locationProxyAddr.c_str());

    //return true if updated
    return true;

  }


  

  void VisionBindingMonitor::redrawGraphicsText()
  {
    //print current proxies
    printText("Current binding proxies from: %s\n\n", m_subarchitectureID.c_str());

    //now remove any proxies we've not seen again
    for(map<string,string>::iterator i = m_sourceProxyMapping.begin();
	i != m_sourceProxyMapping.end(); ++i)
      {
	printText("SceneObject: %s  -->  BindingProxy %s\n\n",
		  i->first.c_str(),i->second.c_str());
      }
  }

  void VisionBindingMonitor::addConceptToProxy(const Vision::SceneObject & _so) {

    // All SceneObjects are "things" in our world by default... this
    // should probably be overwritten by categorisation/recognition
    // when we have it
    BindingFeatures::Concept concept;
   
    if(strlen(_so.m_label.m_string) > 0) {
      concept.m_concept = CORBA::string_dup(_so.m_label.m_string);
    }
    else {
      concept.m_concept = CORBA::string_dup("thing");
    }

    addFeatureToCurrentProxy(concept);
    
  }


  bool VisionBindingMonitor::hasColour(const Vision::SceneObject & _so)
  {
    //nah: colour must be > 0 otherwise we can't tell whether it exists
    //or not
    //old: return _so.m_color.m_int >= 0 && _so.m_color.m_confidence >= 0;
    return _so.m_color.m_int > 0 && _so.m_color.m_confidence > 0;
  }


  void VisionBindingMonitor::addColourToProxy(const Vision::SceneObject & _so)
  {  

    BindingFeatures::Colour colour;

    int visualValue = _so.m_color.m_int;
    string value;

    // HACK: Temporary hardcoded colour table
    if(visualValue == Vision::RED) value = Vision::COLOR_RED;
    else if(visualValue == Vision::GREEN) value = Vision::COLOR_GREEN;
    else if(visualValue == Vision::BLUE)  value = Vision::COLOR_BLUE;
    else if(visualValue == Vision::YELLOW) value = Vision::COLOR_YELLOW;
    else
    {
      log("Cannot translate the visual colour attribute");
      return;
    }
 
    colour.m_colour = CORBA::string_dup(value.c_str());

    addFeatureToCurrentProxy(colour);

    // after data is set in your feature, then store it onto the proxy
/*    BindingData::FeaturePointer colourPointer(addFeatureToCurrentProxy(colour));
    BindingData::FeatureComparatorQuery* featureQuery = new BindingData::FeatureComparatorQuery();

    featureQuery->m_knownFeature = colourPointer;
    featureQuery->m_bindingSubarchitectureID = CORBA::string_dup(bindingSA().c_str());
    featureQuery->m_answered = false;

    addToWorkingMemory(newDataID(), getBindingSA(),
                       BindingOntology::FEATURE_COMPARATOR_QUERY_TYPE, featureQuery);
*/
  }


  bool VisionBindingMonitor::hasShape(const Vision::SceneObject & _so)
  {
    return _so.m_shape.m_int > 0 && _so.m_shape.m_confidence > 0;
  }


  void VisionBindingMonitor::addShapeToProxy(const Vision::SceneObject & _so)
  {
    
    BindingFeatures::Shape shape;

    int visualValue = _so.m_shape.m_int;
    string value;

    // HACK: Temporary hardcoded shape table
    if(visualValue == SQUARED) value = SHAPE_SQUARED;
    else if(visualValue == TRIANGULAR) value = SHAPE_TRIANGULAR;
    else if(visualValue == CIRCULAR) value = SHAPE_CIRCULAR;   
    else
    {
      log("Cannot translate the visual shape");
      return;
    }
 
    shape.m_shape = CORBA::string_dup(value.c_str());

    addFeatureToCurrentProxy(shape);
  }


  bool VisionBindingMonitor::hasSize(const Vision::SceneObject & _so)
  {
    return _so.m_size.m_int > 0 && _so.m_size.m_confidence > 0;
  }


//  void VisionBindingMonitor::addSizeToProxy(const Vision::SceneObject & _so)
//  {
//    log("Adding size feature");
//
//    BindingFeatures::Size size;
//    
//    int visualValue = _so.m_size.m_int;
//    string value;
//
//     // HACK: Temporary hardcoded shape table
//    if(visualValue == 7) value = "large";
//    else if(visualValue == 8) value = "small";
//    else
//    {
//      log("Cannot translate the visual shape");
//      return;
//    }
//
//    size.m_size = CORBA::string_dup(value.c_str());
// 
//    addFeatureToCurrentProxy(size);
//  }


/**  HACK: Temporary moved here from comparator
   * Handles learning oportunitys by translating the binding feature to visual attribute.
   * Then wraps the attribute in a learn instruction and pushes it in WM.
 */
  void VisionBindingMonitor::processFeatureTransfer(BindingData::BindingFeatureTransfer _ft)
  {
    debug("Process new feature");
    debug("Got a new feature of type <" + string(_ft.m_feature.m_type) + ">");

    shared_ptr<const CASTData<BindingData::BindingProxy> > bindingProxy_ptr
        = getWorkingMemoryEntry<BindingData::BindingProxy>(_ft.m_target);

    shared_ptr<const BindingData::BindingProxy> bindingProxy = bindingProxy_ptr->getData();

    unsigned int i;
    for(i = 0;
        i < bindingProxy->m_proxyFeatures.length() &&
        string(bindingProxy->m_proxyFeatures[i].m_type) != typeName<BindingFeatures::SourceData>();
        ++i)
    {}
  
    debug("Extracting SourceData feature.");

    shared_ptr<const CASTData<BindingFeatures::SourceData> > sourceData_ptr
        = getWorkingMemoryEntry<BindingFeatures::SourceData>
        (string(bindingProxy->m_proxyFeatures[i].m_address), getBindingSA());

    const cdl::WorkingMemoryID sceneObjAddr = CORBA::string_dup(sourceData_ptr->getData()->m_address.m_id);

    debug("SourceData addr: <" + string(sceneObjAddr) + ">");

    long visualValue;
    BindingFeaturesCommon::TruthValue truth;
    string VisualObjType;

  /// Translate binding feature to visual attribute
  // Colour
    if(string(_ft.m_feature.m_type) == typeName<BindingFeatures::Colour>())
    {
      shared_ptr<const CASTData<BindingFeatures::Colour> > feature_ptr
          = getWorkingMemoryEntry<BindingFeatures::Colour>(string(_ft.m_feature.m_address),
          getBindingSA());

      const string value(feature_ptr->getData()->m_colour);
      truth = feature_ptr->getData()->m_parent.m_truthValue;
      debug("Colour feature: " + value);

      if(value == Vision::COLOR_RED) visualValue = Vision::RED;
      else if(value == Vision::COLOR_GREEN) visualValue = Vision::GREEN;
      else if(value == Vision::COLOR_BLUE)  visualValue = Vision::BLUE;
      else if(value == Vision::COLOR_YELLOW) visualValue = Vision::YELLOW;
      else
      {
        log("cannot translate colour string to vision attribute");
        return;
      }

      VisualObjType = typeName<SceneObject>();
    }
  // Shape
    else if(string(_ft.m_feature.m_type) == typeName<BindingFeatures::Shape>())
    {
      shared_ptr<const CASTData<BindingFeatures::Shape> > feature_ptr
          = getWorkingMemoryEntry<BindingFeatures::Shape>(string(_ft.m_feature.m_address),
          getBindingSA());

      const string value(feature_ptr->getData()->m_shape);
      truth = feature_ptr->getData()->m_parent.m_truthValue;
      debug("Shape feature: " + value);

      if(value == SHAPE_SQUARED) visualValue = SQUARED;
      else if(value == SHAPE_TRIANGULAR) visualValue = TRIANGULAR;
      else if(value == SHAPE_CIRCULAR) visualValue = CIRCULAR;
      else
      {
        log("Cannot translate the shape string to vision attribute");
        return;
      }

      VisualObjType = typeName<SceneObject>();
    }
  // Size
    else if(string(_ft.m_feature.m_type) == typeName<BindingFeatures::Size>())
  {
    shared_ptr<const CASTData<BindingFeatures::Size> > feature_ptr
        = getWorkingMemoryEntry<BindingFeatures::Size>(string(_ft.m_feature.m_address),
        getBindingSA());
    
    const string value(feature_ptr->getData()->m_size);
    truth = feature_ptr->getData()->m_parent.m_truthValue;
    debug("Size feature: " + value);
    
    if(value == SIZE_SMALL) visualValue = SMALL;
    else if(value == SIZE_LARGE) visualValue = LARGE;
    else
    {
      log("Cannot translate the size string to vision attribute");
      return;
    }
    
    VisualObjType = typeName<SceneObject>();
  }
    else
    {
      log("Unknown attribute type");
      return;
    }

    IntWithConfidence* pIntConf = new IntWithConfidence();
    pIntConf->m_int = visualValue;
    if(truth == BindingFeaturesCommon::NEGATIVE)
      pIntConf->m_confidence = float(-1.0);
    else
      pIntConf->m_confidence = float(1.0);

    IntWithConfidenceSequence intConfSeq(1, 1, pIntConf);

    LearnInstruction* pLearnInst = new LearnInstruction();
    pLearnInst->m_tutorInitiated = true;
    pLearnInst->m_targetAddress = CORBA::string_dup(sceneObjAddr);
    pLearnInst->m_features = intConfSeq;
    pLearnInst->m_type = CORBA::string_dup(VisualObjType.c_str());

    addToWorkingMemory<LearnInstruction>(newDataID(), pLearnInst);

    log("Added a learning instruction for attribute #%i", visualValue);
  }



  void 
  VisionBindingMonitor::monitorNewProxy(const std::string _proxyID) {
    //also start monitoring the proxy
    ProxyMonitor *pProxyMonitor = new ProxyMonitor(_proxyID,
						   getBindingSA(),
						   this);
	
    pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Colour>());
    pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Shape>());
//  pProxyMonitor->addLearnableConcept(typeName<BindingFeatures::Size>());
    
    //add a change filter to monitor that particular proxy
    addChangeFilter(createAddressFilter(_proxyID,
					getBindingSA()),
		    pProxyMonitor);
    	
    //store for later
    m_proxyMonitorMap[_proxyID] = pProxyMonitor;      
  }


  void
  VisionBindingMonitor::propertyUpdates(const cdl::WorkingMemoryChange& _wmc) {

    try {

      shared_ptr<const CASTData<SceneObjectUpdate> > updatesData
	= getWorkingMemoryEntry<SceneObjectUpdate>(_wmc.m_address);	
      shared_ptr<const SceneObjectUpdate> updates(updatesData->getData());
      string soID(updates->m_soID);

      //get the proxy address for the object
      map<string,string>::iterator i =
	m_sourceProxyMapping.find(soID);

      //get the object
      const Vision::SceneObject & sceneObject(m_soCache[soID]);


      //HACK ignore hand SceneObjects for the time being
      //string label(sceneObject.m_label.m_string);
      //if(label.find_first_of("Hand") != string::npos) {
      //debug("ignoring hand SceneObject");
      // return;
      //}

      //have we done something that actually needs binding to be
      //triggered
      bool bindingNeeded(false);
      bool proxyCreated(false);
      string proxyID("");

      //if we've not seen this before
      if(i == m_sourceProxyMapping.end()) {
	log("new proxy for %s", soID.c_str());
	//create a new proxy for it, does not call storeCurrentProxy
	newProxy(sceneObject,soID);       
	proxyCreated = true;
	bindingNeeded = true;
      }
      else {
	debug("updating proxy for %s", soID.c_str());
	proxyID = i->second;
      }

      //determine what needs to be turned into a feature
      set<string> skip;
      if(updates->m_updates.m_colour) {
	skip.insert(typeName<BindingFeatures::Colour>());
      }

      if(updates->m_updates.m_shape) {
	skip.insert(typeName<BindingFeatures::Shape>());
      }
    
      if(updates->m_updates.m_size) {
	skip.insert(typeName<BindingFeatures::Size>());
      }

      //if class is updated then we know that this is a "flag" or not 
      if(updates->m_updates.m_class) {
	skip.insert(typeName<BindingFeatures::Concept>());
      }



      if(skip.size() > 0) {

	log("load proxy to change for %s", soID.c_str());

	//if we are updating we need to load, therefore if (!proxyCreated
	//&& skip.size() > 0) means store needed
	if(!proxyCreated) {
	  changeExistingProxy(proxyID, skip);
	}

	if(updates->m_updates.m_colour) {  
	  log("colour confidence : %f", sceneObject.m_color.m_confidence); 
	  if(sceneObject.m_color.m_confidence > 0.8) {
	    log("adding colour feature to binding proxy  %s", soID.c_str());
	    addColourToProxy(sceneObject);
	  }
	  else if(sceneObject.m_color.m_confidence > 0.45) {
	    log("adding %s colour motive", soID.c_str());
	    //    addColourMotive(sceneObject);      
	  }
	}      

      
	if(updates->m_updates.m_shape) {
	  
	  if(sceneObject.m_shape.m_confidence > 0.8) {
	    log("adding shape feature to binding proxy  %s", soID.c_str());
	    addShapeToProxy(sceneObject);
	  }
	  else if(sceneObject.m_color.m_confidence > 0.45) {
	    log("adding %s shape motive", soID.c_str());
	    //    addShapeMotive(sceneObject);
	  }
	}
	
	//	if(updates->m_updates.m_size) {
	//	  log("%s size updated", soID.c_str());
	//	  addSizeToProxy(sceneObject);
	//	}	




	//if class is updated then we know that this is a "flag" or not 
	if(updates->m_updates.m_class) {
	  addConceptToProxy(sceneObject);
	}


	
	//if skip > 0 we need to binding something
	bindingNeeded = true;
		  
      }

    BindingData::FeaturePointer salienceFeatPtr;
    const FrameworkBasics::BALTTime time(BALTTimer::getBALTTime());
      
    if(proxyCreated)
    {  
    // Add an open salience interval
    salienceFeatPtr =
        addSalienceToCurrentProxy(startTime(time), infiniteFuture());
    }
    
      //we need to store  if
   if(//it's a new proxy
	 proxyCreated
	 ||
	 //or we loaded an existing proxy and changed it
	 bindingNeeded) {
	//store proxy back to wm
	proxyID = storeCurrentProxy();      
   }

      //nothing should be current here
      assert(currentProxyID() == "");


      //object book keeping

    // Remember the new SceneObject proxy as the currently salient

   if(proxyCreated) {

    // Deal with the saliency...
    if(m_anythingSalient)
      {
        // Close the saliency interval of the last salient SceneObject proxy
        changeExistingProxy(m_salientAddr);
        BindingFeaturesCommon::StartTime
            start(getWorkingMemoryEntry<BindingFeatures::Salience>(string(m_sailienceFeatPtr.m_address),getBindingSA())->getData()->m_start);
        addSalienceToCurrentProxy(start, endTime(time));
        deleteFeatureFromCurrentProxy(m_sailienceFeatPtr);
        storeCurrentProxy();
      }
    else
      m_anythingSalient = true;

    // Remember the new SceneObject proxy as the currently salient
    m_salientAddr = proxyID;
    m_sailienceFeatPtr = salienceFeatPtr;    
	
	//store mapping frmo obj to proxy
	m_sourceProxyMapping[soID] = proxyID;

	//if learning from feature transfers
	if(m_transferFeatures) {
	  //also start monitoring the proxy
	  monitorNewProxy(proxyID);
	}
      }

      //now handle locations      

      //log("position updated: %d", updates->m_updates.m_position);
      
      if(proxyCreated) {
	//add the location for this object
	addLocationToProxy(proxyID, sceneObject); 
      }
      else if(updates->m_updates.m_position) {
	bool locationChanged = updateLocation(sceneObject, proxyID);
	
	if(locationChanged) {
	  log("%s position updated", soID.c_str());
	}
	
	bindingNeeded = bindingNeeded || locationChanged;
      }

      if(bindingNeeded) {
	//trigger binding
	bindNewProxies();
      }

      //delete update struct
      deleteFromWorkingMemory(_wmc.m_address);

    }
    catch(const BindingException & e) {
      log("BindingException in propertyUpdates: %s", e.what());
    }

  }
  

  void VisionBindingMonitor::runComponent() {

    lockProcess();
    //let the world know what we're capable of
    registerFeatureGenerationCompetence<BindingFeatures::Colour>(*this);
    //registerFeatureGenerationCompetence<BindingFeatures::Shape>(*this);
    unlockProcess();

    //wait until binder read
    sleepProcess(5000);

    lockProcess();

    //add proxy for robot!
    startNewBasicProxy();
    BindingFeatures::Concept concept;   
    concept.m_concept = CORBA::string_dup("robot");
    addFeatureToCurrentProxy(concept);
    m_robotProxyID = storeCurrentProxy();      
    bindNewProxies();

    unlockProcess();



    if(m_transferFeatures) {

      ProxyMonitor * pProxyMonitor = NULL;
      string proxyAddr;
      vector<BindingData::BindingFeatureTransfer> featureTransfers;

      //while running
      while(m_status == STATUS_RUN) {
	//just sleep while no input has been received

	waitForChanges();
	
	lockProcess();


	//iterate through proxy monitors to see if anything needs to be done
	//now remove any proxies we've not seen again
	for(map<string,ProxyMonitor *>::iterator i = m_proxyMonitorMap.begin();
	    i != m_proxyMonitorMap.end(); ++i)
	  {


	    proxyAddr = i->first;
	    pProxyMonitor = i->second;

	    if(pProxyMonitor->needsDeleting()) {
	      log("Deleting proxy monitor <" + string(proxyAddr) + ">");
	      m_proxyMonitorMap.erase(i);		
	      removeChangeFilter(pProxyMonitor, cdl::DELETE_RECEIVER);
	      break;
	    }
	    
	    //HACK FOR TESTING
	    if(m_dummyClarify) {
	      if(pProxyMonitor->proxyIsBound()) {
		queryFeatureValue<BindingFeatures::Colour>(proxyAddr);
		m_dummyClarify = false;
	      }
	    }


	    //check whether any learning opportunites have occurred!
	    featureTransfers = pProxyMonitor->getFeatureTransfers();

	    log("%i feature(s) to transfer for proxy monitor <%s>", featureTransfers.size(), proxyAddr.c_str());

	    if(featureTransfers.size() > 0)
	      {
		for(vector<BindingData::BindingFeatureTransfer>::iterator j = featureTransfers.begin();
		    j < featureTransfers.end(); ++j) {
		  log("Transfer feature <" + string(j->m_feature.m_type) + ">");

                  processFeatureTransfer(*j);
		  //write transfere to binding sa
// 		  addToWorkingMemory<BindingData::BindingFeatureTransfer>(newDataID(),
// 				     getBindingSA(),
// 				     new BindingData::BindingFeatureTransfer(*j));

		  debug("<" + string(j->m_feature.m_type) + "> transfered");
		}
		pProxyMonitor->clearFeatureTransfers();
	      }
	  }
	log("%i proxy monitor(s) currently active", m_proxyMonitorMap.size());

	unlockProcess();
      }
    }
  }

  void 
  VisionBindingMonitor::configure(map<string,string> & _config) {

    AbstractMonitor::configure(_config);

    if(_config["--no-transfers"] != "") {
      log("turned off feature transfers");
      m_transferFeatures = false;
    }
    else {
      m_transferFeatures = true;
    }

    if(_config["--dummy-clarification"] != "") {
      log("doing dummy clarification");
      m_dummyClarify = true;
    }
    else {
      m_dummyClarify = false;
    }

    setBindingSubarchID(getBindingSA());

  }


} // namespace Binding

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::VisionBindingMonitor(_id);
  }
}

