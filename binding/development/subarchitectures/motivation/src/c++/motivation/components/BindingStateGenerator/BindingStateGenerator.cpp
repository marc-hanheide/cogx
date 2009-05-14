#include "BindingStateGenerator.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <planning/util/PlanningUtils.hpp>
#include <planning/idl/PlanningData.hh>
#include <motivation/idl/MotivationData.hh>


using namespace boost;
using namespace motivation::idl;
using namespace Binding;

BindingStateGenerator::BindingStateGenerator(const string& _id) :
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id),
  AbstractBindingWMRepresenter(dynamic_cast<cast::WorkingMemoryReaderProcess&>(*this)),
  m_handler(*this),
  m_avmReceiver(NULL),
  m_avmID("")

{
  setReceiveXarchChangeNotifications(true); 

  m_self.name = CORBA::string_dup("MrChips");
  m_self.type = CORBA::string_dup("robot");
}

void
BindingStateGenerator::start() {

  ManagedProcess::start();

  assert(m_avmReceiver == NULL);
  m_avmReceiver 
    = new MemberFunctionChangeReceiver<BindingStateGenerator>(this, 
							      &BindingStateGenerator::avmListAdded);
  addChangeFilter(createLocalTypeFilter<AddressVariableMappings>(cdl::ADD),
		  m_avmReceiver);

    addChangeFilter(createLocalTypeFilter<AddressVariableMappings>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<BindingStateGenerator>(this, 
									    &BindingStateGenerator::dummyTrigger));


  addChangeFilter(createLocalTypeFilter<planning::autogen::PlanningStateRequest>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingStateGenerator>(this, 
								      &BindingStateGenerator::stateRequested));


  addChangeFilter(createGlobalTypeFilter<motivation::idl::FeatureGenerationCompetence>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingStateGenerator>(this, 
								      &BindingStateGenerator::generationCompetenceAdded));

  registerFunctors();
}

void
BindingStateGenerator::generationCompetenceAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  shared_ptr<const FeatureGenerationCompetence> competence(getWorkingMemoryEntry<FeatureGenerationCompetence>(_wmc.m_address)->getData());	
  string subarch(competence->m_subarchitecture);
  string type(competence->m_type);
  
  log("BindingStateGenerator::generationCompetenceAdded: %s", type.c_str());

  if(!competence->m_isRelationLabel) {
    //log("BindingStateGenerator::generationCompetenceAdded: is not relation label");
    type = toPlanningFeature(type);
  }
  else {
    //log("BindingStateGenerator::generationCompetenceAdded: is relation label");
  }

  m_subarch2competences[subarch].insert(type);
  log("subarch %s can generate type %s", subarch.c_str(), type.c_str());
}

void 
BindingStateGenerator::avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  assert(m_avmID == "");
  m_avmID = string(_wmc.m_address.m_id);
  log("AddressVariableMappings list at: " + m_avmID);
  removeChangeFilter(m_avmReceiver, cdl::DELETE_RECEIVER);
  m_avmReceiver = NULL;
}

void
BindingStateGenerator::registerFunctors() {

  //TODO separate out these things


  //PLAYMATE

  //turns concepts into object declarations
  registerBasicUnionGenerator<HasFeatureApplicable<BindingFeatures::Concept>, 
    ConceptTranslator>();

  //adds their colours
  registerBasicUnionGenerator<HasFeatureApplicable<BindingFeatures::Colour>, 
    ColourTranslator>();

  //and shapes
  registerBasicUnionGenerator<HasFeatureApplicable<BindingFeatures::Shape>, 
    ShapeTranslator>();

  //turns locations into waypoint object declarations
  registerBasicUnionGenerator<HasFeatureApplicable<BindingFeatures::Location>, 
    LocationTranslator>();


  //turn position relations into pos facts
  registerBinaryRelationUnionGenerator<RelationLabelToUnaryTranslator>("pos");

  //turn spatial relations into  facts
  registerBinaryRelationUnionGenerator<RelationLabelToBinaryTranslator>("near");
  registerBinaryRelationUnionGenerator<RelationLabelToBinaryTranslator>("left");
  registerBinaryRelationUnionGenerator<RelationLabelToBinaryTranslator>("right");
  registerBinaryRelationUnionGenerator<RelationLabelToBinaryTranslator>("front");
  registerBinaryRelationUnionGenerator<RelationLabelToBinaryTranslator>("back");




  //EXPLORER

  //turns locations into waypoint object declarations
  registerBasicUnionGenerator<HasFeatureApplicable<BindingFeatures::AreaID>, 
    AreaIDTranslator>();
  //turn position relations into pos facts
  registerBinaryRelationUnionGenerator<PositionWithIDTranslator>("position");


}
  


void
BindingStateGenerator::generateState(const UnionSet & _unions, 
				     TemporaryPlanningState & _state) {
  //apply state gen functors to all unions
  for(UnionSet::const_iterator u = _unions.begin();
      u != _unions.end(); ++u) {

    const string & var(variableName(u->second));

    for(set<AbstractBasicStateGenerator*>::const_iterator g = m_basicFunctors.begin();
	g != m_basicFunctors.end(); ++g) {
      
      const AbstractBasicStateGenerator* functor(*g);

      bool applied((*functor)(u->second, var, _state));
//       if(applied) {
// 	log("applied functor");
//       }
//       else {
// 	log("not applied functor");
//       }
    }    
  }

  //relation functors.. inefficient 
  for(UnionSet::const_iterator u = _unions.begin();
      u != _unions.end(); ++u) {

    const UnionPtr & relUnion(u->second);
    const string & relVar(variableName(relUnion));

    if(relUnion->type() == BindingData::RELATION) {


#warning BindingStateGenerator can only handle relations generated as simple relations
      //get the to and from out ports, assuming this is a simple relation
      
      const PortMap& outPorts = relUnion->outPorts();

//       //now let's just debug what's in there
//       for(PortMap::const_iterator op = outPorts.begin();
// 	  op != outPorts.end(); ++op) {
// 	cout<<"outport: "<<op->first<<endl;
//       }


      //find from and to ports
      PortMap::const_iterator fromItr(outPorts.find("from"));
      PortMap::const_iterator toItr(outPorts.find("to"));


      //sanity check use of simple rel
      if(fromItr == outPorts.end()) {
	println("ignoring relation without a \"from\" port");
      }
      else if(toItr == outPorts.end()) {
	println("ignoring relation without a \"to\" port");
      }
      else {
	
	//some dumb checks just to make sure
	assert(fromItr->second.size() == 1);	
	string fromProxyID(fromItr->second.begin()->m_proxyID);
	ProxyPtr & fromProxy(m_handler.loadProxy(fromProxyID));
	UnionPtr & fromUnion(m_handler.loadUnion(fromProxy->bindingUnionID()));
	const string & fromVar(variableName(fromUnion));

	assert(toItr->second.size() == 1);
	string toProxyID(toItr->second.begin()->m_proxyID);
	ProxyPtr & toProxy(m_handler.loadProxy(toProxyID));
	UnionPtr & toUnion(m_handler.loadUnion(toProxy->bindingUnionID()));
	const string & toVar(variableName(toUnion));

      
	for(set<AbstractBinaryRelationStateGenerator*>::const_iterator g = m_simpleRelFunctors.begin();
	    g != m_simpleRelFunctors.end(); ++g) {
	  
	  const AbstractBinaryRelationStateGenerator* functor(*g);
	
	  bool applied((*functor)(relUnion, relVar, 
				  fromUnion, fromVar,
				  toUnion, toVar,
				  _state));
// 	  if(applied) {
// 	    log("applied relation functor");
// 	  }
// 	  else {
// 	    log("not applied relation functor");
// 	  }
      }    
      }
    }
  }
    

  
  //addCompetences(_state);

  //ostringstream out;
  //out<<_state;
  //log("new state: %s",out.str().c_str());
  //log("new state: %d objects, %d facts",_state.m_objectList.size(), _state.m_factList.size());
}

void 
BindingStateGenerator::addCompetences(const std::string & _agent, 
				      TemporaryPlanningState & _state) const {

  //now add the K predicates for each subarch
  for (StringSetMap::const_iterator subarch = m_subarch2competences.begin();
       subarch != m_subarch2competences.end(); ++subarch) {    

    string saVar(variableName(subarch->first));
    
    //add subarch
    ObjectDeclaration obj;
    
    //add subarch name
    obj.name = CORBA::string_dup(saVar.c_str());
    obj.type = CORBA::string_dup("subarchitecture");
    _state.m_objectList.insert(obj);    
    
    //add competences
    for (StringSet::const_iterator comps = subarch->second.begin();
	 comps != subarch->second.end(); ++comps) {    
      string feature("knows_about__" + *comps);
      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup(feature.c_str());
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(saVar.c_str());
      fact.value = CORBA::string_dup("true");
      _state.m_factList.insert(fact);

      Fact fact2;
      fact2.modality = NO_MODALITY;
      fact2.agent = CORBA::string_dup("");
      fact2.name = CORBA::string_dup("subarchitecture_of");
      fact2.arguments.length(1);
      fact2.arguments[0]  = CORBA::string_dup(saVar.c_str());
      fact2.value = CORBA::string_dup(_agent.c_str());
      _state.m_factList.insert(fact2);
    }
  }

}

void 
BindingStateGenerator::stateRequested(const cdl::WorkingMemoryChange& _wmc) {

  log("BindingStateGenerator::stateRequested");

  generateAndStoreState();

  //get the request struct
  shared_ptr< const CASTData<PlanningStateRequest> > psrPtr = 
    getWorkingMemoryEntry<PlanningStateRequest>(_wmc.m_address);

  //and the original process request
  shared_ptr< const CASTData<PlanningProcessRequest> > pprPtr = 
    getWorkingMemoryEntry<PlanningProcessRequest>(psrPtr->getData()->m_pprAddr);

  string agentName(pprPtr->getData()->m_agent);
  
  //add these things to state
  addCompetences(agentName,m_cachedState);
  
  //create a copy to overwrite
  PlanningStateRequest * psr = new PlanningStateRequest(*(psrPtr->getData()));


 
  //copy in cached state
  m_cachedState.toPlanningState(psr->m_state);
  
  //and overwrite on wm
  overwriteWorkingMemory(string(_wmc.m_address.m_id),psr);

}

void 
BindingStateGenerator::dummyTrigger(const cdl::WorkingMemoryChange& _wmc) {

  println("BindingStateGenerator::dummyTrigger");

  generateAndStoreState();

}


void 
BindingStateGenerator::generateAndStoreState() {


  log("BindingStateGenerator::generateAndStoreState");
     
  //flag to check for exceptions etc. 
  //
  //if an exception is thrown then it's probably that the
  //AddressVariableMappings is out of date, so keep trying
  bool done(false);


  while(!done) {

    try {
      //load avms
      shared_ptr<const AddressVariableMappings> avm(getWorkingMemoryEntry<AddressVariableMappings>(m_avmID)->getData());
      log("BindingStateGenerator::read avms");

      
      m_proxy2var.clear();
      set<string> proxyIDs;
      for(unsigned int i = 0; i < avm->m_mappings.length(); ++i) {    
	string proxyID(avm->m_mappings[i].m_pointer.m_address.m_id);
	  m_proxy2var[proxyID] 
	    = string(avm->m_mappings[i].m_variable);

	  log("mapped: " + string(avm->m_mappings[i].m_pointer.m_address.m_id) + " -> " + string(avm->m_mappings[i].m_variable));

	//if this is not a subarch
	if(m_subarch2competences.find(proxyID) == m_subarch2competences.end()) {
	  proxyIDs.insert(proxyID);
	}
      }
      
      
      
      //load data from fresh
      m_handler.purgeAllLoadedData();
      ProxySet proxies = m_handler.loadProxies(proxyIDs);
      assert(proxies.size() == proxies.size());
      UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
      
      log("BindingStateGenerator::generateAndStoreState: %d proxies comprise %d unions",
	  proxies.size(), unions.size());
      
      //clear previous state
      m_cachedState.clear();
      
      //HACK add self to state
      //m_cachedState.m_objectList.insert(m_self);

      //now work through unions chucking out state
      generateState(unions,m_cachedState);

     
      cout<<"state state state: "<<endl;
      cout<<m_cachedState<<endl;

      done = true;
    }
    catch (const DoesNotExistOnWMException & e ) {
      log("BindingStateGenerator::generateAndStoreState: exception caught: %s",e.what());
      log("BindingStateGenerator::generateAndStoreState: trying again for state generation");      
    }
    catch (const BindingException & e ) {
      log("BindingStateGenerator::generateAndStoreState: exception caught: %s",e.what());
      log("BindingStateGenerator::generateAndStoreState: trying again for state generation");      
    }
  }

}

void 
BindingStateGenerator::runComponent() {
  
}

void 
BindingStateGenerator::configure(map<string,string> & _config) {

  ManagedProcess::configure(_config);

  if(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY] != "") {
    setBindingSubarchID(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY]);
  }
  else {
    log("binding subarch not specified, assuming it\'s local to monitor");
    setBindingSubarchID(m_subarchitectureID);
  }
}

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new BindingStateGenerator(_id);
  }
}

