#include "AbstractFeature.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"
#include <iostream>

namespace Binding {
using namespace std;
/*
bool OptimizedAbstractFeaturePtrCmp::operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
						const boost::shared_ptr<AbstractFeature>& _f2) const {
  double q1 = BindingFeatureOntology::featureStats(_f1->name()).quality();
  double q2 = BindingFeatureOntology::featureStats(_f2->name()).quality();
  if(q1 != q2)
    return q1 < q2;
  return *_f1 < *_f2;
}


bool 
OptimizedAbstractFeaturePtrWithRepetitionsCmp::operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
							  const boost::shared_ptr<AbstractFeature>& _f2) const {
  double q1 = BindingFeatureOntology::featureStats(_f1->name()).quality();
  double q2 = BindingFeatureOntology::featureStats(_f2->name()).quality();
  if(q1 != q2)
  return q1 < q2;
  return _f1->featureID() <_f2->featureID();
}
*/

const FeatureProperties& 
AbstractFeature::properties() const {
  //return BindingFeatureOntology::featurePropertyMap().find(name())->second;
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  return ontology.featureHelper(this->typeInfo()).properties();
}

std::string 
AbstractFeature::toString(FeatureDisplayMode _fs) const 
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  std::stringstream str;
  ontology.featureHelper(this->typeInfo()).print(str,*this);
  switch(_fs) {
  case show_id:
    return "(" + featureID() + "):" + str.str();
  case show_proxy_id:
    return "[" + immediateProxyID() + "]:" + str.str();
  case show_all:
    return "(" + featureID() + "[" + immediateProxyID() + "]):" + str.str();
  case only_feature:
    return str.str();
  default:
    throw BindingException("Incorrect Feature<T>::FeatureDisplayMode");
  }
}

std::string 
AbstractFeature::toDotLabel(FeatureDisplayMode _fs) const {
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  std::stringstream str;
  ontology.featureHelper(this->typeInfo()).printDotLabel(str,*this);

  switch(_fs) {
  case show_id:
    return "(" + featureID() + "):" + str.str();
  case show_proxy_id:
    return "[" + immediateProxyID() + "]:" + str.str();
  case show_all:
    return "(" + featureID() + "[" + immediateProxyID() + "]):" + str.str();
  case only_feature:
    return str.str();
  default:
    throw BindingException("Incorrect Feature<T>::FeatureDisplayMode");
  }
  //return "(" + featureID() + "[" + immediateProxyID() + "]):" + Binding::toDotLabel(idlFeature);    
  //    return Binding::toDotLabel(idlFeature);    
}


const std::string& 
AbstractFeature::name() const {
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  return ontology.featureName(typeInfo());
}

const std::string&
AbstractFeature::shortName() const
{
  return name();
}

bool 
AbstractFeature::operator<(const AbstractFeature& _f) const
{
  // first, sort according to name
  if(this->name() != _f.name())
    return this->name() < _f.name();
  // then, to not mix up negated with nonnegated features
  if(this->negated() != _f.negated())
    return this->negated() > _f.negated();
  
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  return ontology.featureHelper(this->typeInfo()).operatorLess(*this, _f);
  
}

bool 
AbstractFeature::comparable(const AbstractFeature& _f) const {
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());    
  return NULL != ontology.internalComparator(this->typeInfo(), _f.typeInfo());
  
  //return this->properties().comparable(_f.name());
  //const set<std::string>& comparable(this->properties().comparableInternally);
  //if(comparable.find(_f.name()) == comparable.end()) // i.e. the feature is not among the comparable
  //return false;
  //return true;
}

boost::logic::tribool 
AbstractFeature::compare(const AbstractFeature& _f) const 
{
  //cout << "AbstractFeature::compare" << endl;
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());

  if(!comparable(_f))
    throw(BindingException("Attempting to compare the two uncomparable features: " + 
			   name() + " and " + _f.name() + " in AbsttractFeature::identical(...)"));
  // if the feature is the one and same it will not be counted as a
  // match (this is slightly counterintuitive...)
  boost::logic::tribool ret;
  //#warning equal immediate proxy ID test -> indeterminate test in scoring...
  //    if(this->featureID() == _f.featureID()) {
  if(this->immediateProxyID() == _f.immediateProxyID()) {
    ret = boost::logic::indeterminate;
  } else {
    //    ret = featuresEquivalent<IDLFeature>(idlFeature,_f);
    assert(ontology.internalComparator(this->typeInfo(), _f.typeInfo()));
    ret = ontology.internalComparator(this->typeInfo(), _f.typeInfo())->compare(*this,_f);
  }
  
  // 'not red' will match 'not red', and 'not blue' vs.  'red' will
  // be indeterminate. 'not blue' vs. 'not blue' will also result in
  // indeterminate. Optional behaviours could be made part of the
  // feature properties...
  if(this->negated() != _f.negated()) {
    if(ret.value == boost::logic::tribool::true_value) { 
      ret = false;
    } else {
      ret = boost::logic::indeterminate;
    }
  } else if(this->negated() && _f.negated()) {
    return boost::logic::indeterminate; 
  }
  
//#define VERBOSE_COMPARISON
#ifdef VERBOSE_COMPARISON
  cout << "cmp: Union Feature vs. Proxy Feature\n";
  cout << "cmp: " << this->featureID() << " vs. " << _f.featureID() <<endl;
  cout << "cmp: " << this->immediateProxyID() << " vs. " << _f.immediateProxyID()  <<endl;
  cout << "cmp: " << this->shortName() << " vs. " <<  _f.shortName() << endl;
  cout << "cmp: " << this->toString(show_all) << " vs. " <<  _f.toString(show_all) << endl;
  if(negated() != _f.negated()) {
    cout << "cmp: RESULT IS NEGATED" << endl;
  }
  cout << "cmp: result: " << Binding::triboolToString(ret) << endl;
#endif //VERBOSE_COMPARISON
  return ret;
}
  
} // namespace Binding 
