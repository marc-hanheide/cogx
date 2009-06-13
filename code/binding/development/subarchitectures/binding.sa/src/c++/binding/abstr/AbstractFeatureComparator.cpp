#include "AbstractFeatureComparator.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace boost::logic;
using namespace cast;

namespace Binding {


AbstractFeatureComparator::AbstractFeatureComparator(const std::string &_id)
  :   
      WorkingMemoryAttachedComponent(_id),
      PrivilegedManagedProcess(_id)
{
  queueBehaviour = cdl::QUEUE;
}

void
AbstractFeatureComparator::start()
{
  PrivilegedManagedProcess::start();
  
  addChangeFilter(createGlobalTypeFilter<BindingData::FeatureComparisonTask>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<AbstractFeatureComparator>(this,&AbstractFeatureComparator::_processFeatureComparisonTask));   
}

void 
AbstractFeatureComparator::configure(map<string,string> & _config) 
{
  PrivilegedManagedProcess::configure(_config);
  
//  cout << "CONFIG: \n";
//  for_all(_config,print_pair_fct<string,string>(cout," = "));

  if(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY] != "") {
    string subarch = _config[BindingData::BINDING_SUBARCH_CONFIG_KEY];
    vector<string> subarchs;
    SubarchitectureWorkingMemoryProtocol::tokenizeString(subarch,
							 subarchs,
							 ",");
    for(vector<string>::iterator i = subarchs.begin();
	i < subarchs.end();
	++i) {
      assert(*i != "");
      bindingSA.insert(*i);
      log("adding binding subarch: " + *i);
    }
  }
  else {
    throw BindingException("binding subarchs not specified! All of them must be sepcified to register the comparator competence");
  }

}


void 
AbstractFeatureComparator::addFeatureComparisonFilter(const string& _proxyFeatureType, 
						      const string& _unionFeatureType,
						      const BindingData::ComparisonTrustSpecification& _comparisonTrustSpecification)
{
  filter[_proxyFeatureType].insert(make_pair(_unionFeatureType,_comparisonTrustSpecification));
  for(set<string>::const_iterator i = bindingSA().begin() ; i != bindingSA().end() ; ++i) {
    BindingData::FeatureComparisonCompetence* competence = new BindingData::FeatureComparisonCompetence();
    competence->proxyFeatureType = CORBA::string_dup(_proxyFeatureType.c_str());
    competence->unionFeatureType = CORBA::string_dup(_unionFeatureType.c_str());
    competence->comparisonTrustSpecification = _comparisonTrustSpecification;
    assert(*i != "");
    addToWorkingMemory(newDataID(), 
		       *i,
		       //BindingLocalOntology::FEATURE_COMPARISON_COMPETENCE_TYPE, 
		       competence, 
		       cdl::BLOCKING);
  }
}

bool 
AbstractFeatureComparator::currentComparisonIsMyTask() const
{
  if(currentComparison.featureComparison.get() == NULL)
    return false;
  
  string proxy_feature_type(currentComparison().proxyFeature.type);
  FilterMap::const_iterator i = filter.find(proxy_feature_type);
  if(i == filter.end())
    return false;

  string union_feature_type(currentComparison().unionFeature.type);
  UnionToSpecMap::const_iterator j = i->second.find(union_feature_type);
  if(j == i->second.end())
    return false;

  return true;
}



void 
AbstractFeatureComparator::_processFeatureComparisonTask(const cdl::WorkingMemoryChange & _wmc)
{
  
  log("------ CALLED (" +  string(_wmc.address.id) + "at" + string(_wmc.address.subarchitecture) + ")");

  //store binding sa id for other processes
  
  shared_ptr<const BindingData::FeatureComparisonTask> task;
  shared_ptr<const BindingData::FeatureComparison> comparison;
  try{
      task = getWorkingMemoryEntry<BindingData::FeatureComparisonTask>(string(_wmc.address.id),string(_wmc.address.subarchitecture))->getData();
  }
  catch(const DoesNotExistOnWMException& _e) {
    log(string("Task missing in AbstractFeatureComparator::_processFeatureComparisonTask: ") + _e.what());
    return;
  }
  try {
    comparison = getWorkingMemoryEntry<BindingData::FeatureComparison>(string(task->comparisonID),string(task->bindingSubarchitectureID))->getData();
  } catch(const DoesNotExistOnWMException& _e) {
    throw(BindingException(string("in AbstractFeatureComparator::_processFeatureComparisonTask(...): FeatureComparisonTask referred to nonexisting FeatureComparison: ") + _e.what()));
  }

  
  currentComparison.featureComparison = comparison;
  
  log(lexical_cast<string>(currentComparison().proxyFeature.type) + " vs. " + 
      lexical_cast<string>(currentComparison().unionFeature.type));
  
  if(!currentComparisonIsMyTask()) {  
    // i.e., not our task... do nothing
    log("not my task!" + lexical_cast<string>(currentComparison().proxyFeature.type) + " vs. " + 
	lexical_cast<string>(currentComparison().unionFeature.type));
    currentComparison = Comparison();
    return;
  }
  
  // this is our task! delete the task specification right away
  deleteFromWorkingMemory(string(_wmc.address.id), string(_wmc.address.subarchitecture));  
  
  currentComparison.id = task->comparisonID;
  currentComparison.originalValue = tribool_cast(comparison->featuresEquivalent);
  currentComparison.newValue = executeComparison();
  
  log("comparison result: " + Binding::triboolToString(currentComparison.newValue) + 
      " (old value: " + Binding::triboolToString(currentComparison.originalValue) + ")");
  
  // only store if resulting value is actually different
  if(currentComparison.featureComparison->insistOnExternalComparison ||
     currentComparison.originalValue.value != currentComparison.newValue.value) {
    BindingData::FeatureComparison* result = 
      new BindingData::FeatureComparison(*currentComparison.featureComparison);
    result->featuresEquivalent = tribool_cast(currentComparison.newValue);
    
    overwriteWorkingMemory(string(currentComparison.id), 
			   string(task->bindingSubarchitectureID),
			   //BindingLocalOntology::FEATURE_COMPARISON_TYPE, 
			   result,
			   cdl::BLOCKING);
  }
  currentComparison.featureComparison = boost::shared_ptr<const BindingData::FeatureComparison>();
}


  void AbstractFeatureComparator::runComponent() {
    startComparator();
  }

} // namespace Binding
