
#include "BindingMotiveGenerator.hpp"

#include <planning/util/PlanningUtils.hpp>
#include <planning/idl/PlanningData.hh>
#include <balt/core/StringMap.hpp>
#include <binding/utils/BindingUtils.hpp>

using namespace boost;
using namespace cast;
using namespace cast::cdl;
using namespace Binding;
using namespace BindingData;
using namespace BindingFeatures;
using namespace std;
using namespace motivation::idl;


BindingMotiveGenerator::BindingMotiveGenerator(const string& _id) :
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id),
  AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this)),
  m_motiveHandler(*this),
  m_avmCache(*this),
  m_avmReceiver(NULL),
  m_avmID("") {
  setReceiveXarchChangeNotifications(true); 
}

void
BindingMotiveGenerator::start() {
  ManagedProcess::start();

  assert(m_avmReceiver == NULL);
  m_avmReceiver 
    = new MemberFunctionChangeReceiver<BindingMotiveGenerator>(this, 
							       &BindingMotiveGenerator::avmListAdded);
  addChangeFilter(createLocalTypeFilter<AddressVariableMappings>(cdl::ADD),
		  m_avmReceiver);


}


void 
BindingMotiveGenerator::avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  assert(m_avmID == "");
  m_avmID = string(_wmc.m_address.m_id);
  log("AddressVariableMappings list at: " + m_avmID);
  removeChangeFilter(m_avmReceiver, cdl::DELETE_RECEIVER);
  m_avmReceiver = NULL;
}


void 
BindingMotiveGenerator::triggerTypeAdded(const cdl::WorkingMemoryChange& _wmc) {


  for(vector<AbstractMotiveTemplate *>::iterator mt = m_motiveTemplates.begin();
      mt != m_motiveTemplates.end(); ++mt) {

    try {
      
      if((*mt)->instantiateMotive(_wmc)) {
	
	//put this read in just to stall us until things have settled
	//down
	const AddressVariableMappings & avm(m_avmCache[m_avmID]);
	
	if(hasAmbiguousSource(*mt)) {
	  const ProxySet & sourceSet((*mt)->sourceProxies());
	  for(ProxySet::const_iterator sp = sourceSet.begin();
	      sp != sourceSet.end(); ++sp) {
	    if(sp->second->shouldHaveAmbiguity()) {	    
	      BindingData::Ambiguity* ambiguity = sp->second->ambiguity();
	      if(ambiguity)
		makeSthWithAnAmbiguity(*ambiguity);
	      else {
		log("failed to find Ambiguity struct for proxy %s. Probably I'm too fast!",
		    sp->first.c_str());
	      }
	    }
	  }
	}
	else {
	  //log("instantiated motive");
	  //this hands over the instantiated template to another object for control
	  generateMotive(*mt);      
	  //NB: erase invalids all other iterators, which is ok as we're
	  //returning anyway
	  m_motiveTemplates.erase(mt);
	}
	return;	
      }
    }
    catch (const DoesNotExistOnWMException & e) {
      println("BindingMotiveGenerator::triggerTypeAdded");
      println(e.what());
      ::abort();
    }
    catch (const BALTException & e) {
      println("BindingMotiveGenerator::triggerTypeAdded");
      println(e.what());
      ::abort();
    }
    
    
  }


}

bool 
BindingMotiveGenerator::hasAmbiguousSource(AbstractMotiveTemplate *_mt) {
  //shouldn't be checking otherwise
  assert(_mt->isInstantiated());


  const ProxySet & sourceSet(_mt->sourceProxies());
  for(ProxySet::const_iterator sp = sourceSet.begin();
      sp != sourceSet.end(); ++sp) {
    //only check source data'd proxies
    if(sp->second->hasFeature<SourceData>()) {
      const SourceData & sd(sp->second->getFeature<SourceData>());
      assert(strcmp(sd.m_address.m_subarchitecture,bindingSubarchID().c_str()) == 0);

      //HACK need to use handler to get proxies from here too
      setBindingSubarchID("binding.sa");
      ProxyPtr & source(m_motiveHandler.loadProxy(string(sd.m_address.m_id)));
      setBindingSubarchID(getSubarchitectureID());

      if(source->bestUnionsForProxySet().size() > 1) {
	log("ambiguous source for motive: motive=" 
	    + sp->first + " binding=" + string(sd.m_address.m_id));
	return true;
      }

    }    
  }

  return false;

}



void 
BindingMotiveGenerator::loadAVMs(const std::string _avmID, 
				 StringStringMap & _map) {

  log("reading avms");
  const AddressVariableMappings & avm(m_avmCache[m_avmID]);

  //HACK HACK HACK nah: all the code is ugly... just get something
  //working and make smarter later


  for(unsigned int i = 0; i < avm.m_mappings.length(); ++i) {    
    _map[string(avm.m_mappings[i].m_pointer.m_address.m_id)] = string(avm.m_mappings[i].m_variable);
    cout<<"mapped: "<<string(avm.m_mappings[i].m_pointer.m_address.m_id)<<" "<<string(avm.m_mappings[i].m_variable)<<endl;
  }

  log("read avms");

}


void 
BindingMotiveGenerator::generateMotive(AbstractMotiveTemplate * _mt) {

  //may need to be smarter later
  assert(m_avmID != "");
  assert(_mt != NULL);

  StringStringMap avmMap;
  loadAVMs(m_avmID,avmMap);
  StringMapSourceDataTranslator<StringStringMap> trans(avmMap);
  TemporaryPlanningState additionalState;
  println("before motive to achieve goal");
  string goal(_mt->goal(trans,additionalState));
  
  //HACK little trick to get correct avms if they're not complete for
  //the goal
  while(goal == "") {
    log("waiting on complete avm list");
    sleepProcess(500);
    StringStringMap avmMap2;
    loadAVMs(m_avmID,avmMap2);
    StringMapSourceDataTranslator<StringStringMap> trans2(avmMap2);
    goal = _mt->goal(trans2,additionalState);
  }

  println("generating motive to achieve goal: " + goal);

  ObjectDeclaration self;
  self.name = CORBA::string_dup(_mt->agentName(trans).c_str());
  self.type = CORBA::string_dup("robot");

  //the eventual motive id
  string motiveID(newDataID());
  //and the content it will point at
  string motiveContentID(newDataID());

  //create a motive to achieve a planning goal
  AchieveGoalMotive * agm = new AchieveGoalMotive();
  agm->m_ppr.m_maplGoal = CORBA::string_dup(goal.c_str());
  agm->m_ppr.m_domainName = CORBA::string_dup( domainName().c_str() );
  agm->m_ppr.m_domainFile = CORBA::string_dup( domainFile().c_str()  );
  agm->m_ppr.m_agent = CORBA::string_dup( _mt->agentName(trans).c_str() );    
  agm->m_ppr.m_contributors.length(1);
  agm->m_ppr.m_contributors[0] = CORBA::string_dup(getSubarchitectureID().c_str());

  //let it point at the eventual motive
  workingMemoryPointer<Motive>(motiveID, getSubarchitectureID(),
			       agm->m_ppr.m_cause);

  agm->m_ppr.m_execute = true;
  agm->m_ppr.m_status = PlanningStatus( PROPOSED );
  agm->m_ppr.m_succeeded = TriBool( triIndeterminate );
  //add additional state
  agm->m_ppr.m_additionalState = additionalState.toPlanningState();

  //cout<<"additional state: "<<endl;
  //cout<<additionalState<<endl;

  addToWorkingMemory(motiveContentID,
		     agm,
		     cdl::BLOCKING);
  agm = NULL;

  Motive * motive = new Motive();
  //status is proposed
  motive->m_status = MOTIVE_PROPOSED;

  //type is taken from the template
  motive->m_motiveType = _mt->motiveType();

  //store source pointers
  const WorkingMemoryPointerSet & source(_mt->source());
  motive->m_motiveCause.length(source.size());  
  unsigned int i = 0;
  for(WorkingMemoryPointerSet::const_iterator wmp = source.begin();
      wmp != source.end(); ++wmp) {
    motive->m_motiveCause[i++] = *wmp;
  }
  
  const std::string & creator(_mt->creatorSA());
  //will lead to failures that will need to be fixed
  assert(creator != "");
  motive->m_creator = CORBA::string_dup(creator.c_str());

  //set pointer back to agm
  workingMemoryPointer<AchieveGoalMotive>(motiveContentID,
					  getSubarchitectureID(),
					  motive->m_content);
  //unknown success
  motive->m_succeeded = cast::cdl::triIndeterminate;


  
  //create a receiver object that will handle subsequent changes
  addChangeFilter(createIDFilter(motiveID,cdl::OVERWRITE), 
		  new MotiveResultsReceiver(*this,_mt,motiveID));

  addToWorkingMemory(motiveID,
		     motive);
  motive = NULL;

  log("written motive");


}

void 
BindingMotiveGenerator::configure(map<string,string> & _config) {
  ManagedProcess::configure(_config);

//   if(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY] != "") {
//     setBindingSubarchID(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY]);
//   }
//   else {
//     log("binding subarch not specified, assuming it\'s local to monitor");
  //only want to access local proxies  
  setBindingSubarchID(m_subarchitectureID);
    //}

}


void 
BindingMotiveGenerator::MotiveResultsReceiver::workingMemoryChanged(const cast::cdl::WorkingMemoryChange& _wmc) {
  m_component.log("motive overwritten");
  shared_ptr<const Motive> motive(m_component.getWorkingMemoryEntry<Motive>(m_motiveID)->getData());

  //motive is complete
  if(motive->m_status == MOTIVE_COMPLETE) {
    m_component.log("motive complete");
    
    //make sure nothing is reloaded
    m_component.m_motiveHandler.purgeAllLoadedData();

    //reset motive template after completion
    //
    //this should do any finalisation etc. for the motive gen
    m_template->reset(motive->m_succeeded);

    //then, afer reset add back in to motive set so we can generate
    //from it again
    m_component.m_motiveTemplates.push_back(m_template);

    //now delete from wm -- don't delete motive, let originator do
    //that later
    //m_component.deleteFromWorkingMemory(m_motiveID);


    //and trigger dot viewer
    TriggerDotViewer * trigger = new TriggerDotViewer();
    m_component.addToWorkingMemory(m_component.newDataID(), 
				   trigger, 
				   cdl::BLOCKING);

    //and remove this receiver and get free'd
    m_component.removeChangeFilter(this, cdl::DELETE_RECEIVER);


  }

}

void 
BindingMotiveGenerator::makeSthWithAnAmbiguity(const BindingData::Ambiguity& _ambiguity)
{
  if(m_bLogOutput) {
    //cout << "storing the Ambiguity:\n " << *issue << endl;  
    analyseAmbiguity(cout,*this,_ambiguity);
    cout << endl;
  }

  //stringstream unions;
  //str << "We do have an ambiguity:\n"
  //    << _ambiguity << "\n";
  //for(unsigned int i = 0; i < _ambiguity.m_unionIDs.length() ; ++i) {
  //  unions << _ambiguity.m_unionIDs[i] 
  //   << ((i == _ambiguity.m_unionIDs.length() - 1)?" or ":", ");
  //}
  //str << "Proxy " << _proxy.id() 
  //    << " can be bound with all of Unions " 
  //    << unions.str() << "\n";
  //
  //for(unsigned int i = 0; i < _ambiguity.m_missingProxyFeatures.length() ; ++i) {
  //  const string featuretype(_ambiguity.m_missingUnionFeatures[i]);
  //  str << "I wish I knew more about the " 
  //<< featuretype
  //<< " of the proxy " << _ambiguity.m_proxyID << "\n";
  //  for(unsigned int j = 0; j < _ambiguity.m_unionIDs.length() ; ++j) {
  //    const string unionID(_ambiguity.m_unionIDs[j]);
  //    try {
  //const LBindingUnion& the_union = m_unionLocalCache[unionID];
  //if(the_union.hasFeature(featuretype)) {
  //  const OneTypeOfFeatures& 
  //    possibly_disambiguating_features
  //    (the_union.getFeatures(featuretype));
  //  foreach(shared_ptr<AbstractFeature> feat, possibly_disambiguating_features) {
  //    str << "If I knew whether the proxy's " 
  //	<< feat->name() << " is " 
  //	<< feat->toString(AbstractFeature::only_feature) 
  //	<< " or not, I could possibly disambiguate\n";
  //  }
  //}
  //    } catch(const DoesNotExistOnWMException& _e) {
  //log("Turns out union %s is deleted, maybe the ambiguity already resolved?",
  //    unionID.c_str());
  //    }
  //  }
  //}
  //for(unsigned int i = 0; i < _ambiguity.m_missingUnionFeatures.length() ; ++i) {
  //  str << "I also wish I knew more about the " 
  //<< _ambiguity.m_missingProxyFeatures[i] 
  //<< " of one of the unions " << unions.str() << "\n";
  //}
  //for(unsigned int i = 0; i < _ambiguity.m_unionIDs.length() ; ++i) {
  //  const string unionID(_ambiguity.m_unionIDs[i]);
  //  try {
  //    for(unsigned int j = 0; j < _ambiguity.m_missingUnionFeatures.length() ; ++j) {
  //const string featuretype(_ambiguity.m_missingUnionFeatures[j]);
  //const LBindingUnion& the_union = m_unionLocalCache[unionID];
  //if(!the_union.hasFeature(featuretype)) {
  //  str << "The union " << unionID << "'s " 
  //      << featuretype << " would be cool to know about\n";
  //  const OneTypeOfFeatures& 
  //    sourceIDs(the_union.getFeatures<BindingFeatures::SourceID>());
  //  foreach(shared_ptr<AbstractFeature> source, sourceIDs) {
  //    const Feature<BindingFeatures::SourceID>& 
  //      s(extractFeature<BindingFeatures::SourceID>(*source));
  //    str << "Maybe the subarchitecture " << s->m_sourceID 
  //	<< " could tell us more about the "  
  //	<< featuretype << " of its proxy " 
  //	<< s->m_parent.m_immediateProxyID << "(which is member of union"
  //	<< unionID << ")?\n";
  //  }
  //}
  //    } 
  //  }
  //  catch(const DoesNotExistOnWMException& _e) {
  //    log("Turns out union %s is deleted, maybe the ambiguity already resolved?",
  //  unionID.c_str());
  //  }
  //}
  //log(str.str());
}
