#include <sstream>
#include "BindingUtils.hpp"
//#include "Features.hpp"
#include "binding/feature-utils/AbstractFeature.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
//#include "feature-specialization/FeatureProperties.hpp"
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/abstr/AbstractBindingWMRepresenter.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include <cast/architecture/DoesNotExistOnWMException.hpp>
#include <algorithm>
#include <functional>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach


namespace Binding {
  using namespace std;
  using namespace boost;
  using namespace cast;

void
validateFeatureType(const std::string& _feature, const string& _location) throw(BindingException) {
  const BindingFeatureOntology&  bindingOntology(BindingFeatureOntology::construct()); 
  //  cout << bindingOntology << endl;
  const set<string>& features = bindingOntology.features();
  //cout << features;
  if(features.find(_feature) == features.end())
    throw BindingException((string("Unknown primitive feature: ") + 
			    _feature).c_str() + 
			   _location==""?"\n(at "+ _location + ")":"");
}

ostream& 
operator<<(ostream& _out, const OneTypeOfFeatures& _values) {
  //  for_all(_values, print_set_fct<const set<shared_ptr<AbstractFeature> > >(_out));
  print_set(_out,_values);
  return _out;
}

ostream& 
operator<<(ostream& _out,const FeatureSet& _fset) {
  for_all(_fset,print_pair_fct<string, OneTypeOfFeatures>(_out));
  return _out;
}

/*string 
toString(const FeatureSet& _fset, AbstractFeature::FeatureDisplayMode _mode) {
  stringstream str;
  str << _fset;
  return str.str();
}*/

/*string 
toDotLabel(const OneTypeOfFeatures& s, AbstractFeature::FeatureDisplayMode _mode) {
  stringstream str;
  print_set_w_fct(str, s, to_dot_label_fct(_mode));
  return str.str();
}*/


//string 
//toDotLabel(const FeatureSet& _fset, const set<string>& _excluded_features) {
//}

string toString(const shared_ptr<AbstractFeature>& _ptr, AbstractFeature::FeatureDisplayMode _mode)
{
  return _ptr->toString(_mode);
}

/*string toDotLabel(const shared_ptr<AbstractFeature>& _ptr, AbstractFeature::FeatureDisplayMode _mode)
{  
  return _ptr->toDotLabel(_mode);
}
*/
void
validateFeatureSet(const FeatureSet& _fset, 
		   const string& _location) throw(BindingException)
{
  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  stringstream err;
  if(_fset.empty()) {
    err << "Empty Feature set\n";
  }
  for(FeatureSet::const_iterator i = _fset.begin();
      i != _fset.end();
      ++i) {
    //const FeatureProperties& prop(ont.featurePropertyMap().find(i->first)->second);
    const FeatureProperties& prop(ont.featureHelper(i->first).properties());
    if(prop.m_isSimplex && i->second.size() > 1) {
      err << "Simplexness violated for " << i->first << endl;	
    }
//    if(!prop.m_isImplemented) {
//     err << "Not implemented: " << i->first << "\n";	
//    }
  }
  for(set<string>::const_iterator i = ont.features().begin();
      i != ont.features().end();
      ++i) {
    //const FeatureProperties& prop(ont.featurePropertyMap().find(*i)->second);
    const FeatureProperties& prop(ont.featureHelper(*i).properties());
    if(prop.m_isDemanded) {
      if(_fset.find(*i) == _fset.end()) {
	err << "Required feature is missing: " << *i << "\n";
      }
    }
  }
  cout << err.str() << endl;
  if(!err.str().empty())
    throw(BindingException("FeatureSet not valid: " + 
			   err.str() + (_location!=""?"(at "+ _location + ")":"")));
}



ostream& operator<<(ostream& _out, const BindingData::Ambiguity& _issue) 
 {  
  _out << "Ambiguity {\\n";
  _out << "  m_proxyID: " << _issue.m_proxyID << "\\n";
  _out << "  m_unionIDs: {";
  for(unsigned int i = 0 ; i < _issue.m_unionIDs.length() ; ++i) {    
    _out << _issue.m_unionIDs[i] ;  
    if(i < _issue.m_unionIDs.length() - 1)
      _out << ", ";
  }
  _out<< "}\\n";
  _out << "  m_missingUnionFeatures: {";
  for(unsigned int i = 0 ; i < _issue.m_missingUnionFeatures.length() ; ++i) {    
    _out << trimFeatureName(string(_issue.m_missingUnionFeatures[i])) ;  
    if(i < _issue.m_missingUnionFeatures.length() - 1)
      _out << ", ";
  }
  _out<< "}\\n";
  _out << "  m_missingProxyFeatures: {";
  for(unsigned int i = 0 ; i < _issue.m_missingProxyFeatures.length() ; ++i) {    
    _out << trimFeatureName(string(_issue.m_missingProxyFeatures[i]));  
    if(i < _issue.m_missingProxyFeatures.length() - 1)
      _out << ", ";
  }
  _out<< "}\\n";
  _out << "  m_disambiguatingUnionFeatureInstances: {/*not yet implemented*/}\\n";
  _out<< "}\\n";

  return _out;
}

ostream& 
operator<<(ostream& _out, const BindingData::ComparisonTrust& _trust)
{
  switch(_trust) {
  case BindingData::TRUST_COMPLETELY:
    _out << "TRUST_COMPLETELY";
    return _out;
  case BindingData::TRUST_BUT_VALIDATE:
    _out << "TRUST_BUT_VALIDATE";
    return _out;
  case BindingData::DONT_USE:
    _out << "DONT_USE";
    return _out;
  default:
    throw(BindingException("Unknown BindingData::ComparisonTrust"));
  }
}

ostream& operator<<(ostream& _out, const BindingData::ComparisonTrustSpecification& _spec)
{
  _out << "{";
  _out << "m_trustInternalFalse = " << _spec.m_trustInternalFalse;
  _out << ", m_trustInternalIndeterminate = " << _spec.m_trustInternalIndeterminate;
  _out << ", m_trustInternalTrue = " << _spec.m_trustInternalTrue << "}";
  return _out;
}

ostream& operator<<(ostream& _out, const BindingData::FeatureComparisonCompetence& _competence)
{
  _out << "FeatureComparisonCompetence{\n";
  _out << "  m_proxyFeatureType = " << _competence.m_proxyFeatureType << endl; 
  _out << "  m_unionFeatureType = " << _competence.m_unionFeatureType << endl; 
  _out << "  m_comparisonTrustSpecification = " << _competence.m_comparisonTrustSpecification<<"\n}";
  return _out;
}

/// makes a combined ID out of two IDs
string 
combinedID(const std::string& _id1, 
	   const std::string _id2) {
  //    cout << "_combinedID:\n" << _id1 << " + \n" << _id2 << " = "<< endl;
  std::string ret(_id1.size()+_id2.size(),' ');
  std::string::const_iterator i1(_id1.begin());
  std::string::const_iterator i2(_id2.begin());
  int n(0);
  bool flip(true);
  while(!(i1 == _id1.end() && i2 == _id2.end())) {
    ret[n++] = flip?*i1++:*i2++;
    flip = !flip;
    if(i1 == _id1.end())
      flip = false;
    if(i2 == _id2.end())
      flip = true;
  }
  //    cout << ret;
  return ret;
}    


string
toString(const BindingData::FeaturePointer& _feat) {
  stringstream str;
  str << PRINTNAMED(_feat.m_type) << ", ";
  str << PRINTNAMED(_feat.m_address);
  return str.str();
}

string
toString(const BindingData::FeaturePointers& _feats) {
  stringstream str;
  for(unsigned int i = 0; i< _feats.length() ; ++i)
    str << i << ": " << toString(_feats[i]);
  return str.str();
}

string
toString(const BindingData::BindingProxy& _proxy) {
  stringstream str;
  str << "m_proxyFeatures: " << toString(_proxy.m_proxyFeatures) << ", ";
  str << "m_unionID: " << _proxy.m_unionID;
  return str.str();
}


const string&
to_string(BindingData::BindingProxyType _t) {
  switch(_t) {
  case BindingData::BASIC: 
    {
      static const string str("BASIC");
      return str;
    }
  case BindingData::GROUP:
    {
      static const string str("GROUP");
      return str;
    }
  case BindingData::RELATION:
    {
      static const string str("RELATION");
      return str;
    }
  }
  static const string str("UNKNOWN TYPE!");
  return str;
}

const string&
to_string(BindingData::BindingProxyState _state)
{
  switch(_state) {
  case BindingData::NEW: 
    {
      static const string str("NEW");
      return str;
    }
  case BindingData::UPDATED: 
    {
      static const string str("UPDATED");
      return str;
    }
  case BindingData::REPROCESSED: 
    {
      static const string str("REPROCESSED");
      return str;
    }
  case BindingData::BOUND: 
    {
      static const string str("BOUND");
      return str;
    }
  default:
    cerr << "unknown BindingData::BindingProxyState in proxyStateToString in BindingDotViewer.cpp" << endl;
    abort();
  }
}

string 
trimFeatureName(const string& _string) {
  string str(_string);
  string::size_type pos = str.find_last_of("::");
  str = str.substr(pos+1,_string.size());
  return str;
}
static const string& 
neator(unsigned int _i, unsigned int _size) 
{
  static const string nil;
  static const string or_(" or ");
  static const string comma(", ");
  switch(_size - _i) {
  case 0: assert(false);
  case 1: return nil;
  case 2: return or_;
  default: return comma;
  }
  return comma;
}

static unsigned int& 
strategy_count(ostream& _out, const unsigned int i = 1) {
  static unsigned int strategy(0);
  assert(i>0);
  if(i==1) {
    _out << "Disambiguation strategy " 
	 << strategy++ << ": \n";
  } else {
    _out << "Disambiguation strategies " 
	 << strategy;
    strategy += i;
    _out << " to " << strategy++ << ": \n";    
  }
  return strategy;
}

void 
analyseAmbiguity(std::ostream& _out,
		      AbstractBindingWMRepresenter& _bindingWMRepresenter,
		      const BindingData::Ambiguity& _ambiguity) 
{
  AbstractBindingWMRepresenter& repr(_bindingWMRepresenter);
  cast::WorkingMemoryReaderProcess& component(repr.component());
  
  stringstream unions;
  _out << "We do have an ambiguity:\n"
       << _ambiguity << "\n";
  const string proxyID(_ambiguity.m_proxyID);
  const LBindingProxy* ptr = repr.maybeLoadProxy(proxyID);
  if(!ptr) {
    component.log("no proxy despite it having an ambiguity");
    return;
  }
  const LBindingProxy& proxy(*ptr);

  for(unsigned int i = 0; i < _ambiguity.m_unionIDs.length() ; ++i) {
    unions << _ambiguity.m_unionIDs[i]
	   << neator(i,_ambiguity.m_unionIDs.length());
  }
  _out << "Proxy " << proxyID
       << " can in principle be bound with all of Unions " 
       << unions.str() << "\n";
  stringstream dummy;
  (strategy_count(dummy)) = 1;
  FeatureSet important_features;
  for(unsigned int i = 0; i < _ambiguity.m_missingProxyFeatures.length() ; ++i) {
    const string featuretype(_ambiguity.m_missingProxyFeatures[i]);
    strategy_count(_out);
    _out << "I wish I knew anything about the " 
	 << trimFeatureName(featuretype)
	 << " of proxy " << proxyID << "\n";
    for(unsigned int j = 0; j < _ambiguity.m_unionIDs.length() ; ++j) {
      const string unionID(_ambiguity.m_unionIDs[j]);
      try {
	const LBindingUnion& 
	  the_union(repr.m_unionLocalCache[unionID]);
	if(the_union.hasFeature(featuretype)) {
	  const FeatureSet 
	    possibly_disambiguating_features
	    (the_union.getFeatureSetComparableTo(featuretype));
	  typedef pair<string,OneTypeOfFeatures> Element;
	  foreach(const Element& e, possibly_disambiguating_features) {
	    foreach(const shared_ptr<AbstractFeature>& feat, 
		    e.second) {
	      strategy_count(_out);
	      _out << "If I knew whether proxy " << proxyID << "'s " 
		   << trimFeatureName(feat->name()) << " is " 
		   << feat->toString(AbstractFeature::only_feature) 
		   << " or not, I could possibly determine which one of unions " 
		   << unions.str() 
		   << " that should bind to the proxy " << proxyID << "\n";
	      important_features[feat->name()].insert(feat);
	    }
	  }
	}
      } catch(const cast::DoesNotExistOnWMException& _e) {
	component.log("Turns out union %s is deleted, maybe the ambiguity already resolved?",
		      unionID.c_str());
      }
    }
  }
  if(!important_features.empty()) {
    typedef pair<string,OneTypeOfFeatures> Element;
    strategy_count(_out,important_features.size());
    _out << "So, if I knew whether the ";
    unsigned int ftype = 0;
    foreach(const Element& e, important_features) {
      _out << trimFeatureName(e.first) << " is ";
      unsigned int f = 0;
      foreach(const shared_ptr<AbstractFeature>& feat, e.second) {
	_out << feat->toString(AbstractFeature::only_feature)
	     << neator(f++,e.second.size());
      }
      _out << neator(ftype++,important_features.size());
    }
    _out << " or not, I could possibly determine which one of unions " 
	 << unions.str() 
	 << " that should bind to the proxy " << proxyID << "\n";
  }
  
  
  for(unsigned int i = 0; i < _ambiguity.m_missingUnionFeatures.length() ; ++i) {
    strategy_count(_out);
    _out << "I wish to know more about the " 
	 << trimFeatureName(string(_ambiguity.m_missingUnionFeatures[i]))
	 << " of one of the unions " << unions.str() << "\n";
  }
  for(unsigned int i = 0; i < _ambiguity.m_unionIDs.length() ; ++i) {
    const string unionID(_ambiguity.m_unionIDs[i]);
    try {
      for(unsigned int j = 0; j < _ambiguity.m_missingUnionFeatures.length() ; ++j) {
	const string featuretype(_ambiguity.m_missingUnionFeatures[j]);
	const LBindingUnion& 
	  the_union(repr.m_unionLocalCache[unionID]);
	if(!the_union.hasFeature(featuretype)) {
	  strategy_count(_out);
	  _out << "I want to know more about union " << unionID << "'s " 
	       << trimFeatureName(featuretype) << "\n";
	  const OneTypeOfFeatures& 
	    sourceIDs(the_union.getFeatures<BindingFeatures::SourceID>());
	  foreach(const shared_ptr<AbstractFeature>& source, sourceIDs) {
	    const Feature<BindingFeatures::SourceID>& 
	      s(extractFeature<BindingFeatures::SourceID>(*source));
	    strategy_count(_out);
	    _out << "Maybe the subarchitecture " << s->m_sourceID 
		 << " could tell us more about the "  
		 << trimFeatureName(featuretype) << " of its proxy " 
		 << s->m_parent.m_immediateProxyID << " (which is a member of union "
		 << unionID << ")?\n";
	    if(proxy.hasFeature(featuretype)) {
	      const FeatureSet&
		pset(proxy.getFeatureSetComparableTo(featuretype));
	      strategy_count(_out);
	      typedef pair<string,OneTypeOfFeatures> Element;
	      foreach(const Element& e, pset) {
		foreach(const shared_ptr<AbstractFeature> f, e.second) {
		  _out << "More specifically, maybe the subarchitecture " << s->m_sourceID 
		       << " could even tell us if its proxy " 
		       << s->m_parent.m_immediateProxyID  << "s "  
		       << trimFeatureName(featuretype) << " is " 
		       << f->toString(AbstractFeature::only_feature) << "?\n";
		    }
	      }
	    }
	  }
	}
      } 
    }
    catch(const DoesNotExistOnWMException& _e) {
      component.log("Turns out union %s is deleted, maybe the ambiguity already resolved?",
		    unionID.c_str());
    }
  }
}

} // namespace Binding

