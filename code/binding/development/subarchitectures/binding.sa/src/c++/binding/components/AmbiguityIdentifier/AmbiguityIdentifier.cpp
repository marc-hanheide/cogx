#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/abstr/AbstractMonitor.hpp"
#include "AmbiguityIdentifier.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


using namespace std;
using namespace boost;
using namespace cast;
using namespace boost::assign;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::AmbiguityIdentifier(_id);
  }
}

namespace Binding {

AmbiguityIdentifier::AmbiguityIdentifier(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id)
{ 
}

void
AmbiguityIdentifier::start() {

  AbstractBinder::start();
  
  
/*  addChangeFilter(BindingLocalOntology::BEST_UNIONS_FOR_PROXY_TYPE, 
		  cdl::OVERWRITE, 
		  cdl::LOCAL_SA, //only look at sa-local changes
		  new MemberFunctionChangeReceiver<AmbiguityIdentifier>(this,
									     &AmbiguityIdentifier::identifyDisambiguation));
*/

  addChangeFilter(createLocalTypeFilter<BindingData::BestUnionsForProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<AmbiguityIdentifier>(this,
									     &AmbiguityIdentifier::identifyDisambiguation));



/*  addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
		  cdl::DELETE, 
		  cdl::ALL_SA, 
		  new MemberFunctionChangeReceiver<AmbiguityIdentifier>(this,
									     &AmbiguityIdentifier::proxyDeleted));
*/
  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<AmbiguityIdentifier>(this,
									&AmbiguityIdentifier::proxyDeleted));
}



AmbiguityIdentifier::~AmbiguityIdentifier() {

}



ostream& 
operator<<(ostream& _out, const map<string,set<string> >& _stuff) {
  for(map<string,set<string> >::const_iterator i = _stuff.begin() ; 
      i != _stuff.end();
      ++i) {
    _out << i->first << " : {";
    set<string>::const_iterator j = i->second.begin();
    while(j != i->second.end()) {
      _out << *j;
      ++j;
      if(j != i->second.end())
	_out << ", ";
    }
    _out << "}\n";
  }
  return _out;
}



void 
AmbiguityIdentifier::identifyDisambiguation(const cdl::WorkingMemoryChange & _wmc) {
  shared_ptr<const BindingData::BestUnionsForProxy> 
    best(loadBindingDataFromWM<BindingData::BestUnionsForProxy>(_wmc));
  if(best->unionIDs.length() == 1)
    return; // i.e. do nothing...

  assert(best->unionIDs.length() != 0);
  
  BindingData::Ambiguity* issue = new BindingData::Ambiguity;
  
  
  issue->proxyID = CORBA::string_dup(best->proxyID);
  issue->unionIDs.length(best->unionIDs.length());
  for(unsigned int i = 0 ; i < issue->unionIDs.length() ; ++i) {
    issue->unionIDs[i] = CORBA::string_dup(best->unionIDs[i]);
  }
  
  try {
    
    string proxyID(issue->proxyID);
    const LBindingProxy& proxy(proxyLocalCache[proxyID]);
    const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
    const map<string,set<string> >& proxy2unionComparable(ont.proxy2unionComparable());
    const map<string,set<string> >& union2proxyComparable(ont.union2proxyComparable());
    set<string> missing_union_features;
    set<string> missing_proxy_features;
    for(unsigned int i = 0 ; i < proxy->proxyFeatures.length() ; ++i) {
      const string type(proxy->proxyFeatures[i].type);
      map<string,set<string> >::const_iterator comp(proxy2unionComparable.find(type));
      if(comp != proxy2unionComparable.end()) // comparable union features found
	missing_union_features.insert(comp->second.begin(),comp->second.end());
    }
    //    cout << "missing_union_features: " << endl;
    //  Binding::print_set(cout, missing_union_features);  
    
    
    for(unsigned int i = 0 ; i < issue->unionIDs.length() ; ++i) {
      string unionID(issue->unionIDs[i]); 
      const LBindingUnion& binding_union(unionLocalCache[unionID]);
      for(unsigned int j = 0 ; j < binding_union->unionFeatures.length() ; ++j) {
	const string type(binding_union->unionFeatures[j].type);
	const string immediate(binding_union->unionFeatures[j].immediateProxyID);
	// erase types that actually are represented on the unions...
	if(immediate != proxyID) {
	  // ... but do not erase it if it's directly from the proxy, because then it wasn't used for scoring in the first place
	  missing_union_features.erase(type);
	  // insert features based on unions here to avoid accessing
	  // the union cache again.  these are not inserted if
	  // features are only from the immediate proxy, though
	  map<string,set<string> >::const_iterator comp(union2proxyComparable.find(type));
	  if(comp != union2proxyComparable.end()) // comparable proxy features found
	    missing_proxy_features.insert(comp->second.begin(),comp->second.end());
	}
      }
    }

    /*#warning, serious optimization issue here
      for(unsigned int i = 0 ; i < issue->unionIDs.length() ; ++i) {
      string unionID(issue->unionIDs[i]); 
      const LBindingUnion& binding_union(unionLocalCache[unionID]);
      for(unsigned int j = 0 ; j < binding_union->unionFeatures.length() ; ++j) {
      string type(binding_union->unionFeatures[j].type);
      string ownerID(string(toFeature(binding_union->unionFeatures[j])->immediateProxyID())); // EXPENSIVE
      if(ownerID == proxyID) { // do not count these!
      map<string,set<string> >::const_iterator comp(union2proxyComparable.find(type));
      if(comp != union2proxyComparable.end()) {
      for(set<string>::const_iterator k = comp->second.begin() ; k != comp->second.end(); ++k)
      if(missing_proxy_features.find(*k) != missing_proxy_features.end())
      missing_proxy_features.erase(*k);
      }
      }
      }
      }
    */    

    // cout << "missing_union_features: ";
    // Binding::print_set(cout, missing_union_features);  
    {
      unsigned j = 0;
      issue->missingUnionFeatures.length(missing_union_features.size());
      for(set<string>::const_iterator i = missing_union_features.begin();
	  i != missing_union_features.end();
	  ++i,++j) {
	issue->missingUnionFeatures[j] = CORBA::string_dup(i->c_str());
      }
    }
    //  cout << "missing_proxy_features: ";
    //  Binding::print_set(cout,missing_proxy_features);
    for(unsigned int i = 0 ; i < proxy->proxyFeatures.length() ; ++i) {
      string type(proxy->proxyFeatures[i].type);
      // erase types that actually do exist in proxy
      missing_proxy_features.erase(type);
    }
    //cout << "missing_proxy_features: ";
    //Binding::print_set(cout,missing_proxy_features);
    {
      unsigned j = 0;
      issue->missingProxyFeatures.length(missing_proxy_features.size());
      for(set<string>::const_iterator i = missing_proxy_features.begin();
	  i != missing_proxy_features.end();
	  ++i,++j) {
	issue->missingProxyFeatures[j] = CORBA::string_dup(i->c_str());
      }
    }
    if(bLogOutput) {
      //cout << "storing the Ambiguity:\n " << *issue << endl;  
      analyseAmbiguity(cout,*this,*issue);
      cout << endl;
    }
    if(!existsOnWorkingMemory(proxyID+ambiguityIDPostFix())) {  
      addToWorkingMemory(proxyID+ambiguityIDPostFix(), 
			 issue,
			 cast::cdl::BLOCKING);
    } else {
      overwriteWorkingMemory(proxyID+ambiguityIDPostFix(), 
			     issue,
			     cast::cdl::BLOCKING);    
    }
    
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in AmbiguityIdentifier::identifyDisambiguation(..) " + string(_e.what()));
    delete issue;
  }
  
}

void 
AmbiguityIdentifier::proxyDeleted(const cdl::WorkingMemoryChange & _wmc)
{
  try {
    if(existsOnWorkingMemory(string(_wmc.address.id)+ambiguityIDPostFix())) {
      deleteFromWorkingMemory(string(_wmc.address.id)+ambiguityIDPostFix());
    }
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in AmbiguityIdentifier::proxyDeleted(..) " + string(_e.what()));
    
  }  
}


} // namespace Binding 

