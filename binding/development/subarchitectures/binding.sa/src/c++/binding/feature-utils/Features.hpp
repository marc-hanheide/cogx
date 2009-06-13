#ifndef BINDING_FEATURES_H_
#define BINDING_FEATURES_H_

#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "BindingData.hpp"
//#include "feature-specialization/FeatureProperties.hpp"
#include "AbstractFeature.hpp"
#include "../utils/BindingScoreUtils.hpp"
//#include "binding/abstr/AbstractMonitor.hpp"

namespace Binding {

/// Implements an \p AbstractFeature and contains the actual feature (of
/// type \p IDLFeature) defined in the IDL-file.
template<class IDLFeature>
class Feature
  : public AbstractFeature
{
public:
  typedef IDLFeature LocalFeatureType;
  Feature(const IDLFeature& _idl_feature,
	  const std::string& _id)
    : AbstractFeature(_id),
      idlFeature(_idl_feature)
      //featureProperties(featureProperties<IDLFeature>())
  {};



//  const std::string name() const {return featureName<IDLFeature>();}
//  std::string shortName() const {return featureShortName<IDLFeature>();}
  /// returns typeid(IDLFeature)
  virtual const std::type_info& typeInfo() const {return typeid(IDLFeature);}

  //const FeatureProperties& properties() const;

  const IDLFeature& idlFeature() const {return idlFeature;}
  const IDLFeature* operator->() const {return &idlFeature;}

  /// returns result of address comparisons by default, this should be
  /// specialized in subclasses to take into accoun the contents of
  /// these
  /*  bool operator<(const AbstractFeature& _f) const {
  // first, sort according to name
  if(this->name() != _f.name())
  return this->name() < _f.name();
  // then, to not mix up negated with nonnegated features
  if(this->negated() != _f.negated())
  return this->negated() > _f.negated();

  // then according to the idlFeature (which now should be known to
  // be of the same kind since the names are equal)
  return featureOperatorLess(idlFeature(), dynamic_cast<const Feature<IDLFeature>&>(_f).idlFeature());
  }
  */
  /*

  /// returns true if this feature can be compared with \p _f
  bool comparable(const AbstractFeature& _f) const {
  return this->properties().comparable(_f.name());
  //const set<std::string>& comparable(this->properties().comparableInternally);
  //if(comparable.find(_f.name()) == comparable.end()) // i.e. the feature is not among the comparable
  //return false;
  //return true;
  }
  */

  // calls the specialized \p featuresEquivalent<IDLFeature>(idlFeature,_f)
  /*  boost::logic::tribool identical(const AbstractFeature& _f) const {
      if(!comparable(_f))
      throw("Attempting to compare the two uncomparable features: " +
      name() + " and " + _f.name() + " in Feature<>::identical(...)");
      // if the feature is the one and same it will not be counted as a
      // match (this is slightly counterintuitive...)
      boost::logic::tribool ret;
      //#warning equal immediate proxy ID test -> indeterminate test in scoring...
      //    if(this->featureID() == _f.featureID()) {
    if(this->immediateProxyID() == _f.immediateProxyID()) {
      ret = boost::logic::indeterminate;
    } else {
      ret = featuresEquivalent<IDLFeature>(idlFeature,_f);
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
    cout << "cmp: Binding Feature vs. Proxy Feature\n";
    cout << "cmp: " << this->featureID() << " vs. " << _f.featureID() <<endl;
    cout << "cmp: " << this->immediateProxyID() << " vs. " << _f.immediateProxyID()  <<endl;
    cout << "cmp: " << this->shortName() << " vs. " <<  _f.shortName() << endl;
    cout << "cmp: " << this->toString() << " vs. " <<  _f.toString() << endl;
    if(negated() != _f.negated()) {
      cout << "cmp: RESULT IS NEGATED" << endl;
    }
    cout << "cmp: result: " << Binding::toString(ret) << endl;
#endif //VERBOSE_COMPARISON
    return ret;
  }*/

  // true iff the feature is actually negated, which inverts the
  // match (i.e. two objects that are 'not red' will actually match)
  virtual bool negated() const {
    return Binding::negated(idlFeature);
  }
  virtual std::string immediateProxyID() const {
    return Binding::immediateProxyID(idlFeature);
  }

  virtual BindingFeaturesCommon::TruthValue truthValue() const {
    return Binding::truthValue(idlFeature());
  }

/*
  virtual BindingData::FeaturePointer addFeatureToCurrentProxy(AbstractMonitor& _monitor, BindingFeaturesCommon::TruthValue _truthValue) const
  {
    return _monitor.addFeatureToCurrentProxy(idlFeature, _truthValue);
  }
*/

private:
  IDLFeature idlFeature;
  //FeatureProperties featureProperties;
  Feature() {}
};

//forward decl
//class AbstractBinder;


} // namespace Binding

#endif // BINDING_FEATURES_H_
