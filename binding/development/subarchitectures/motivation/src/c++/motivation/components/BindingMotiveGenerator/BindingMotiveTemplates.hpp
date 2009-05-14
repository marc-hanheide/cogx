#ifndef BINDING_MOTIVE_TEMPLATES_H_
#define BINDING_MOTIVE_TEMPLATES_H_

#include <binding/idl/BindingData.hh>
#include <binding/utils/GraphLoader.hpp>

#include <binding/utils/BasicPredicates.hpp>
#include <binding/utils/Predicates.hpp>
#include <binding/utils/Extractors.hpp>
#include <binding/utils/Misc.hpp>
#include <planning/util/TemporaryPlanningState.hpp>

#include <cast/core/CASTUtils.hpp>

#include <motivation/idl/MotivationData.hh>
#include <ext/hash_set>

using namespace Binding;
using namespace BindingData;
using namespace BindingFeatures;
using namespace std;


/**
 * Generates a planning var from a ProxyPtr.
 **/
class ProxyVariableTranslator {
protected:
  ProxyVariableTranslator() {}
  virtual ~ProxyVariableTranslator(){}
public:
  virtual std::string translate(Binding::ProxyPtr _proxy) const {
    return translate(_proxy->id());
  }

  virtual std::string translate(const std::string & _proxyID) const {
    return _proxyID;
  }
};

/**
 * Translator that just returns the ID of the input proxy.
 */
class OwnIDTranslator : public ProxyVariableTranslator {
public:
  OwnIDTranslator() : ProxyVariableTranslator() {}
  virtual ~OwnIDTranslator(){}
  virtual std::string translate(const std::string & _proxyID) const {
    return _proxyID;
  }
};

/**
 * Translator that just returns the SourceData ID of the proxy. If no SourceData, then returns id().
 */
class SourceDataTranslator : public ProxyVariableTranslator {
public:
  SourceDataTranslator() : ProxyVariableTranslator() {}
  virtual ~SourceDataTranslator(){}

  virtual std::string translate(Binding::ProxyPtr _proxy) const {
    if(_proxy->hasFeature<BindingFeatures::SourceData>()) {
      const BindingFeatures::SourceData & sd(_proxy->getFeature<BindingFeatures::SourceData>());
      //cout<<"using SourceData in SourceDataTranslator"<<sd.m_address<<" "<<_proxy->id()<<endl;
      return string(sd.m_address.m_id);
    }
    else {
      cout<<"no SourceData in SourceDataTranslator"<<endl;
      return _proxy->id();
    }
  }
};

/**
 * Translator that looks up proxy id in map.
 */
template <class MapT>
class StringMapTranslator : public ProxyVariableTranslator {

public:
  StringMapTranslator(const MapT & _map) : 
    ProxyVariableTranslator(), 
    m_map(_map) {}
  virtual ~StringMapTranslator(){}

  virtual std::string translate(Binding::ProxyPtr _proxy) const {
    if(m_map.find(_proxy->id()) == m_map.end()) {
      return _proxy->id();
    }
    else {
      return m_map.find(_proxy->id())->second;
    }    
  }

private:
  const MapT & m_map;

};


/**
 * Translator that looks up proxy id in map.
 */
template <class MapT>
class StringMapSourceDataTranslator : public ProxyVariableTranslator {

public:
  StringMapSourceDataTranslator(const MapT & _map) : 
    ProxyVariableTranslator(), 
    m_map(_map) {}
  virtual ~StringMapSourceDataTranslator(){}

  virtual std::string translate(Binding::ProxyPtr _proxy) const {
    return translate(m_sdt.translate(_proxy));
  }

  virtual std::string translate(const std::string & _proxyID) const {
    if(m_map.find(_proxyID) == m_map.end()) {
      return _proxyID;
    }
    else {
      return m_map.find(_proxyID)->second;
    }    
  }

private:
  const MapT & m_map;
  SourceDataTranslator m_sdt;
};


//fwd decl
class BindingMotiveGenerator;

/**
 * Abstract template fpr motives generated from binding structures.
 */
class AbstractMotiveTemplate {
public:

  ///uninstantiate the template so it can be used again
  virtual void reset(const cast::cdl::TriBool & _succeeded);

  ///check whether the motive has been instantiated
  virtual bool isInstantiated() const {
    return m_instantiated;
  }

  ///try to instantiate the template starting from some data with the
  ///input id and type
  virtual bool instantiateMotive(const cast::cdl::WorkingMemoryChange & _wmc) {
    //cannot be reinstantiated once instantiated without being reset
    string type(_wmc.m_type);
    if(m_instantiated || !isApplicableType(type)) {
      return false;
    }

    //try to instantiate subclass
    if(instantiate(_wmc)) {
      m_instantiated = true;
    }
    
    return m_instantiated;
  }

  ///check whether this motive should be instantiated via this type
  virtual bool isApplicableType(const std::string & _type) = 0;


  ///get the goal string for this proxy and any additional state to
  ///include
  virtual std::string goal(ProxyVariableTranslator & _translator, 
			   TemporaryPlanningState & _state) = 0;


//   ///get the name for the robot agent
  virtual std::string agentName(ProxyVariableTranslator & _translator) {
    return "MrChips";
  }

  /**
   * Get the set of proxies used to generate this motive.
   */
  const Binding::ProxySet & sourceProxies() {
    return m_sourceProxies;
  }

    /**
   * Get the source used to generate this motive.
   */
  const cast::WorkingMemoryPointerSet  & source() {
    return m_source;
  }

  ///what general type this motive is
  virtual 
  motivation::idl::MotiveType 
  motiveType() = 0;


  ///what general type this motive is
  virtual 
  const std::string & 
  creatorSA() {
    return m_creatorSA;
  }

  /**
   * As most source data will be proxies, store these for later use.
   */
  void addAllToSourceProxies(const Binding::ProxySet & _proxySet) {    
      for(ProxySet::const_iterator sp = _proxySet.begin();
	  sp != _proxySet.end(); ++sp) {
	addToSourceProxies(sp->second);
      }
  }

  /**
   * As most source data will be proxies, store these for later use.
   */
  void addToSourceProxies(const Binding::ProxyPtr & _proxy) {    

    //useful way of grabbing the creatorSA implicitly
    if(m_creatorSA.empty() &&
       _proxy->hasFeature<BindingFeatures::SourceID>()) {
      const BindingFeatures::SourceID & sid(_proxy->getFeature<BindingFeatures::SourceID>());
      m_creatorSA = string(sid.m_sourceID);      
    }

    m_sourceProxies[_proxy->id()] = _proxy;
    addToSource<BindingData::BindingProxy>(_proxy->id(), 
					   //component().getSubarchitectureID());
					   "");
  }


  /**
   * In the case where proxies are not uses, store wm pointers. This
   * is also called by addToSourceProxies to store pointers to
   * proxies.
   */
  template <class SourceType>
  void addToSource(const std::string & _id,
		   const std::string & _subarch) {
    cast::cdl::WorkingMemoryPointer wmp;
    cast::workingMemoryPointer<SourceType>(_id,
					   _subarch,
					   wmp);
    m_source.insert(wmp);
  }

  virtual ~AbstractMotiveTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

protected:
  AbstractMotiveTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler) : 
    m_handler(_handler),
    m_component(_component),
    m_instantiated(false),
    m_relationCheck(BindingData::RELATION){}


  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc) = 0;

  BindingMotiveGenerator & component() {
    return m_component;
  }

//   virtual bool variableExistsFor(const Binding::ProxyPtr & _proxy,
// 				 ProxyVariableTranslator & _translator) {
//     const string & var(_translator.translate(_proxy));
//     return var.find(":") != string::npos;     
//   }

//   virtual bool variableExistsFor(const std::string & _id,
// 				 ProxyVariableTranslator & _translator) {
//     const string & var(_translator.translate(_id));
//     return var.find(":") != string::npos;     
//   }


  ///Get all the proxies that can be used to generate a motive. This
  ///will be all proxies on motive wm, minus proxies used previously
  ///that are not related to by new proxies
  void motiveProxies(const cast::cdl::WorkingMemoryChange & _wmc,
		     Binding::ProxySet & _proxies);

  Binding::BindingGraphHandler & m_handler;

  //source proxies used previously
  Binding::ProxySet m_sourceProxies;
  //keep a set of source pointers too, for those that are not proxies
  cast::WorkingMemoryPointerSet m_source;


  typedef __gnu_cxx::hash_set<std::string> StringSet;
  //a set of local ids used previously
  StringSet m_usedPreviously;

  ///the sa that created this
  std::string m_creatorSA;
  
  const Binding::TypeCheck<Binding::ProxyPtr> m_relationCheck;  
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortExtractor;

private:
  BindingMotiveGenerator & m_component;
  bool m_instantiated;
  
};

/**
 * Template fpr motives that are based around an actor graph. Doesn't
 * add anything to m_sourceProxies.
 */
class ActorTemplate : public AbstractMotiveTemplate {

protected:
  //extractors, save recreating every time
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortToExtractor;
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortFromExtractor;
  const Binding::RelationLabelRegexMatcher m_actorRegex;


public:
  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);
   
  ActorTemplate(BindingMotiveGenerator & _component,
		Binding::BindingGraphHandler & _handler)  : 
    AbstractMotiveTemplate(_component,_handler),
    m_outPortToExtractor("to"),
    m_outPortFromExtractor("from"),
    m_actorRegex("Actor")
  {}

  virtual ~ActorTemplate(){}

  
  bool isApplicableType(const std::string & _type) {
    return _type == cast::typeName<BindingData::BindingProxy>();
  }

  ///get the name for the robot agent
  virtual std::string agentName(ProxyVariableTranslator & _translator);


protected:

  //the rel with "Actor" label
  Binding::ProxyPtr m_actorRel;
  
  //the actual actor, the "to" of m_actorRel
  Binding::ProxyPtr m_actor;

  //the action, the "from" of m_actorRel
  Binding::ProxyPtr m_action;

};

/**
 * Specialisation of ActorTemplate with a pateien rel
 */
class ActorPatientTemplate : public ActorTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

  ActorPatientTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorTemplate(_component,_handler),
    m_patientRegex("Patient")  {
  }
  
  virtual ~ActorPatientTemplate(){}

protected:
  Binding::RelationLabelRegexMatcher m_patientRegex;
  Binding::ProxyPtr m_patientRel;
  Binding::ProxyPtr m_patient;
};

/**
 * Specialisation of ActorTemplate with a destination rel
 */
class ActorDestinationTemplate : public ActorTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

  ActorDestinationTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorTemplate(_component,_handler),
    m_destinationRegex("Destination.*")  {
  }
  
  virtual ~ActorDestinationTemplate(){}

protected:
  Binding::RelationLabelRegexMatcher m_destinationRegex;
  Binding::ProxyPtr m_destinationRel;
  Binding::ProxyPtr m_destination;
};

/**
 * Specialisation of ActorTemplate with a destination rel
 */
class ActorPatientDestinationTemplate : public ActorPatientTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

  ActorPatientDestinationTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorPatientTemplate(_component,_handler),
    m_destinationRegex("Destination.*")  {
  }
  
  virtual ~ActorPatientDestinationTemplate(){}

protected:
  Binding::RelationLabelRegexMatcher m_destinationRegex;
  Binding::ProxyPtr m_destinationRel;
  Binding::ProxyPtr m_destination;
};




/**
 * Specialisation of ActorPatientTemplate for moving a target object
 * in relation to a landmark one.
 */
class TargetLandmarkTemplate : public ActorPatientDestinationTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);
  virtual std::string goal(ProxyVariableTranslator & _translator, 
			   TemporaryPlanningState & _state);




  TargetLandmarkTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorPatientDestinationTemplate(_component,_handler),
    m_actionChecker(ConceptRegexMatcher("put"))
  {
  }
  
  virtual ~TargetLandmarkTemplate(){}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::PHYSICAL_ACTION;
  }


protected:

  Binding::FeatureChecker<Binding::ProxyPtr,BindingFeatures::Concept, Binding::ConceptRegexMatcher> m_actionChecker;
  Binding::ProxyPtr m_target;
  Binding::ProxyPtr m_landmark;
  Binding::ProxyPtr m_relation;
  

};



/**
 * Specialisation of ActorDestinationTemplate which is used for moving
 * the robot.
 */
class GoToDestinationTemplate : public ActorDestinationTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);
  virtual std::string goal(ProxyVariableTranslator & _translator, 
			   TemporaryPlanningState & _state);


  GoToDestinationTemplate(BindingMotiveGenerator & _component,
			  Binding::BindingGraphHandler & _handler)  :
    ActorDestinationTemplate(_component,_handler),
    m_actionChecker(ConceptRegexMatcher("go")) {
  }
  
  virtual ~GoToDestinationTemplate(){}

  ///get the name for the robot agent
  virtual std::string agentName(ProxyVariableTranslator & _translator);

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::PHYSICAL_ACTION;
  }



protected:

  Binding::FeatureChecker<Binding::ProxyPtr,BindingFeatures::Concept, Binding::ConceptRegexMatcher> m_actionChecker;
};





class FeatureRequestTemplate: public AbstractMotiveTemplate {
public:

  ///check whether this motive should be instantiated via this type
  bool isApplicableType(const std::string & _type) {
    return _type == cast::typeName<BindingQueries::FeatureRequest>();
  }


  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);


  virtual ~FeatureRequestTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

  FeatureRequestTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler) :
    AbstractMotiveTemplate(_component,_handler),
    m_proxyID(""),
    m_reqID(""),
    m_type(""),
    m_featureID(""),
    m_subarchID("") 
      {}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::CLARIFICATION;
  }


protected:

  ///try to instantiate the template from this proxy
  bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

private:
  //the proxy (on binding wm) that the proxy is about
  std::string m_proxyID;
  //the id of the feature request on local wm
  std::string m_reqID;
  //the type of the feature request
  std::string m_type;
  //the id of the feature involved, factual if "", polar if not
  std::string m_featureID;
  //where the query came from
  std::string m_subarchID;

};


class PolarQuestionTemplate: public AbstractMotiveTemplate {
public:

  ///check whether this motive should be instantiated via this type
  bool isApplicableType(const std::string & _type) {
    return _type == cast::typeName<BindingData::BindingProxy>();
  }


  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);


  virtual ~PolarQuestionTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

  PolarQuestionTemplate(BindingMotiveGenerator & _component,
			Binding::BindingGraphHandler & _handler) :
    AbstractMotiveTemplate(_component,_handler),
    m_polarQRegex("Polar-Q"),
    m_outPortFromExtractor("from")
  {}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::POLAR_QUESTION;
  }


protected:

  ///try to instantiate the template from this proxy
  bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

private:
  const Binding::RelationLabelRegexMatcher m_polarQRegex;
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortFromExtractor;
  const Binding::InPortsExtractor<Binding::ProxyPtr> m_inPortsExtractor; 

  Binding::ProxyPtr m_askAbout;
};



class FactualQuestionTemplate: public AbstractMotiveTemplate {
public:

  ///check whether this motive should be instantiated via this type
  bool isApplicableType(const std::string & _type) {
    return _type == cast::typeName<BindingData::BindingProxy>();
  }


  virtual ~FactualQuestionTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

  FactualQuestionTemplate(BindingMotiveGenerator & _component,
			Binding::BindingGraphHandler & _handler) :
    AbstractMotiveTemplate(_component,_handler),
    m_factualQRegex("Fact-Q"),
    m_outPortToExtractor("to"),
    m_outPortFromExtractor("from")
  {}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::FACTUAL_QUESTION;
  }


protected:

  ///try to instantiate the template from this proxy
  bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

protected:
  const Binding::RelationLabelRegexMatcher m_factualQRegex;
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortToExtractor;
  const Binding::OutPortsExtractor<Binding::ProxyPtr> m_outPortFromExtractor;


  Binding::ProxyPtr m_questionedObject;
  Binding::ProxyPtr m_questionedProperty;
  Binding::ProxyPtr m_factQRel;
};

class FactualWHQuestionTemplate: public FactualQuestionTemplate {
public:



  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);


  virtual ~FactualWHQuestionTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

  FactualWHQuestionTemplate(BindingMotiveGenerator & _component,
			Binding::BindingGraphHandler & _handler) :
    FactualQuestionTemplate(_component,_handler),
    m_conceptString("")
  {}


protected:

  ///try to instantiate the template from this proxy
  bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

private:
  std::string m_conceptString;
  std::string m_askingSA;
};





/**
 * Matches motive proxies that have a "Fact-Q" rel to a proxy with a
 * "see" concept.
 **/
class FactualPerceiveQuestionTemplate: public FactualQuestionTemplate {
public:



  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);


  virtual ~FactualPerceiveQuestionTemplate(){}

  ///return the set of proxies involved in this motive
  ///  virtual const Binding::ProxySet & motiveProxies() = 0;

  FactualPerceiveQuestionTemplate(BindingMotiveGenerator & _component,
			Binding::BindingGraphHandler & _handler) :
    FactualQuestionTemplate(_component,_handler)
  {}


protected:

  ///try to instantiate the template from this proxy
  bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

private:
  std::string m_askingSA;
};








/**
 * Specialisation of ActorTemplate with a destination rel
 */
class FindTemplate : public ActorPatientTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);

  FindTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorPatientTemplate(_component,_handler),
    m_actionChecker(ConceptRegexMatcher("find")) {
  }
  
  virtual ~FindTemplate(){}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::PHYSICAL_ACTION;
  }


protected:
  Binding::FeatureChecker<Binding::ProxyPtr,BindingFeatures::Concept, Binding::ConceptRegexMatcher> m_actionChecker;
  
};

/**
 * Play the ? game template
 */
class PlayTheGameTemplate : public ActorPatientTemplate {

public:

  ///try to instantiate the template from this proxy
  virtual bool instantiate(const cast::cdl::WorkingMemoryChange & _wmc);

  ///get the goal string for this proxy
  virtual std::string goal(ProxyVariableTranslator & _translator,			   
			   TemporaryPlanningState & _state);

  PlayTheGameTemplate(BindingMotiveGenerator & _component,
			 Binding::BindingGraphHandler & _handler)  :
    ActorPatientTemplate(_component,_handler),
    m_actionChecker(ConceptRegexMatcher("play")),
    m_goal("") {
  }
  
  virtual ~PlayTheGameTemplate(){}

  virtual 
  motivation::idl::MotiveType 
  motiveType() {
    return motivation::idl::PHYSICAL_ACTION;
  }


protected:
  Binding::FeatureChecker<Binding::ProxyPtr,BindingFeatures::Concept, Binding::ConceptRegexMatcher> m_actionChecker;
  std::string m_goal;
};



#endif //BINDING_MOTIVE_GENERATOR_H
