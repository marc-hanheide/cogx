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
  m_queueBehaviour = cdl::QUEUE;
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
      m_bindingSA.insert(*i);
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
  m_filter[_proxyFeatureType].insert(make_pair(_unionFeatureType,_comparisonTrustSpecification));
  for(set<string>::const_iterator i = bindingSA().begin() ; i != bindingSA().end() ; ++i) {
    BindingData::FeatureComparisonCompetence* competence = new BindingData::FeatureComparisonCompetence();
    competence->m_proxyFeatureType = CORBA::string_dup(_proxyFeatureType.c_str());
    competence->m_unionFeatureType = CORBA::string_dup(_unionFeatureType.c_str());
    competence->m_comparisonTrustSpecification = _comparisonTrustSpecification;
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
  if(m_currentComparison.featureComparison.get() == NULL)
    return false;
  
  string proxy_feature_type(currentComparison().m_proxyFeature.m_type);
  FilterMap::const_iterator i = m_filter.find(proxy_feature_type);
  if(i == m_filter.end())
    return false;

  string union_feature_type(currentComparison().m_unionFeature.m_type);
  UnionToSpecMap::const_iterator j = i->second.find(union_feature_type);
  if(j == i->second.end())
    return false;

  return true;
}



void 
AbstractFeatureComparator::_processFeatureComparisonTask(const cdl::WorkingMemoryChange & _wmc)
{
  
  log("------ CALLED (" +  string(_wmc.m_address.m_id) + "at" + string(_wmc.m_address.m_subarchitecture) + ")");

  //store binding sa id for other processes
  
  shared_ptr<const BindingData::FeatureComparisonTask> task;
  shared_ptr<const BindingData::FeatureComparison> comparison;
  try{
      task = getWorkingMemoryEntry<BindingData::FeatureComparisonTask>(string(_wmc.m_address.m_id),string(_wmc.m_address.m_subarchitecture))->getData();
  }
  catch(const DoesNotExistOnWMException& _e) {
    log(string("Task missing in AbstractFeatureComparator::_processFeatureComparisonTask: ") + _e.what());
    return;
  }
  try {
    comparison = getWorkingMemoryEntry<BindingData::FeatureComparison>(string(task->m_comparisonID),string(task->m_bindingSubarchitectureID))->getData();
  } catch(const DoesNotExistOnWMException& _e) {
    throw(BindingException(string("in AbstractFeatureComparator::_processFeatureComparisonTask(...): FeatureComparisonTask referred to nonexisting FeatureComparison: ") + _e.what()));
  }

  
  m_currentComparison.featureComparison = comparison;
  
  log(lexical_cast<string>(currentComparison().m_proxyFeature.m_type) + " vs. " + 
      lexical_cast<string>(currentComparison().m_unionFeature.m_type));
  
  if(!currentComparisonIsMyTask()) {  
    // i.e., not our task... do nothing
    log("not my task!" + lexical_cast<string>(currentComparison().m_proxyFeature.m_type) + " vs. " + 
	lexical_cast<string>(currentComparison().m_unionFeature.m_type));
    m_currentComparison = Comparison();
    return;
  }
  
  // this is our task! delete the task specification right away
  deleteFromWorkingMemory(string(_wmc.m_address.m_id), string(_wmc.m_address.m_subarchitecture));  
  
  m_currentComparison.id = task->m_comparisonID;
  m_currentComparison.originalValue = tribool_cast(comparison->m_featuresEquivalent);
  m_currentComparison.newValue = executeComparison();
  
  log("comparison result: " + Binding::triboolToString(m_currentComparison.newValue) + 
      " (old value: " + Binding::triboolToString(m_currentComparison.originalValue) + ")");
  
  // only store if resulting value is actually different
  if(m_currentComparison.featureComparison->m_insistOnExternalComparison ||
     m_currentComparison.originalValue.value != m_currentComparison.newValue.value) {
    BindingData::FeatureComparison* result = 
      new BindingData::FeatureComparison(*m_currentComparison.featureComparison);
    result->m_featuresEquivalent = tribool_cast(m_currentComparison.newValue);
    
    overwriteWorkingMemory(string(m_currentComparison.id), 
			   string(task->m_bindingSubarchitectureID),
			   //BindingLocalOntology::FEATURE_COMPARISON_TYPE, 
			   result,
			   cdl::BLOCKING);
  }
  m_currentComparison.featureComparison = boost::shared_ptr<const BindingData::FeatureComparison>();
}


  void AbstractFeatureComparator::runComponent() {
    startComparator();
  }

} // namespace Binding
