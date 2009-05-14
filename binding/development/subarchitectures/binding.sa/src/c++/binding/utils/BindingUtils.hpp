#ifndef BINDING_UTILS_H_
#define BINDING_UTILS_H_

#include <iostream>
#include <map>
#include <set>
#include <cast/architecture/ManagedProcess.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/utils/LocalClasses.hpp>
#include <binding/BindingException.hpp>
#include <binding/feature-utils/AbstractFeature.hpp>
//#include "feature-specialization/FeatureProperties.hpp"
//#include "components/BinderComponent/Binder.hpp"
#include <stdexcept> 
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <binding/utils/Misc.hpp>

#include <boost/preprocessor/stringize.hpp>
#define PRINTNAMED(x) (BOOST_PP_STRINGIZE(x)) << ": (" << (x) << ")"

//#include "FeatureProperties.hpp"

namespace Binding {

/// asserts that a string is a feature type, throws if not
extern 
void
validateFeatureType(const std::string& _feature_type, 
		  const std::string& location = "") throw(BindingException);
extern 
void
validateFeatureSet(const FeatureSet& _fset, 
		   const std::string& location = "") throw(BindingException);
  
extern std::ostream& operator<<(std::ostream&, const FeatureSet&);
//extern std::string toString(const FeatureSet&);


//extern std::string toString(const boost::shared_ptr<AbstractFeature>&, AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature);
//extern std::string toDotLabel(const boost::shared_ptr<AbstractFeature>&, AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature);
//extern std::string toDotLabel(const OneTypeOfFeatures& s, AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature);
extern std::string trimFeatureName(const std::string& _string); 


template<class T>
struct testFeature {
  void operator()(const std::pair<std::string,T>& _feature) const throw(std::runtime_error) {
    assertFeatureType(_feature.first);
  }
};

/*
template<class T, class F>
inline
void for_all(const T& _container, const F& _functor) {
  if(!_container.empty())
    for_each(_container.begin(),_container.end(),_functor);
}
*/

template<class T, class F>
void for_all_nonconst(T& _container, const F& _functor) {
  if(!_container.empty())
    for_each(_container.begin(),_container.end(),_functor);
}


template<class T1, class T2>
struct print_pair_fct {
  print_pair_fct(std::ostream& _out, const std::string& _div = ": ", const std::string& _end = "\n") 
    : m_out(_out),
      m_div(_div),
      m_end(_end) { };
  
  void operator()(const std::pair<T1,T2>& _feature) const {
    m_out << _feature.first << m_div << _feature.second << m_end;
  }
private:
  mutable std::ostream& m_out;
  const std::string& m_div;
  const std::string& m_end;
};


template<class Set>
std::ostream&
print_set(std::ostream& _out, const Set _set) {
  typename Set::const_iterator i = _set.begin();
  _out << "{";
  while(i != _set.end()) {
    _out << *(i++) ;
    if(i != _set.end())
      _out << ", ";
  }
  _out << "}";
  return _out;
}

struct to_dot_label_fct {
  to_dot_label_fct(AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature) : m_mode(_mode) {}
  std::string operator()(const AbstractFeature& _f) const {return _f.toDotLabel(m_mode);}
private:
  AbstractFeature::FeatureDisplayMode m_mode;
};
  
struct to_string_fct {
  to_string_fct(AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature) : m_mode(_mode) {}
  std::string operator()(const AbstractFeature& _f) const {return _f.toString(m_mode);}
private:
  AbstractFeature::FeatureDisplayMode m_mode;
};

template<class Set, class Fct>
std::ostream&
print_set_w_fct(std::ostream& _out, const Set& _set, const Fct& _fct) {
  typename Set::const_iterator i = _set.begin();
  _out << "{";
  while(i != _set.end()) {
    _out << _fct(*(*(i++)));
    if(i != _set.end())
      _out << ", ";
  }
  _out << "}";
  return _out;
}


template<class T>
struct print_fct {
  print_fct(std::ostream& out) : m_out(out) {};
  void operator()(const T& _t) const {
    m_out << _t;
  }
private:
  mutable std::ostream& m_out;
};

template<class Set>
struct print_set_fct {
  print_set_fct(std::ostream& out) : m_out(out) {};
  void operator()(const Set& _set) const {
    print_set(m_out,_set);
  }  
private:
  mutable std::ostream& m_out;
};

template<class T>
std::ostream& 
operator<<(std::ostream& _out, const boost::shared_ptr<T>& _ptr) {
  _out << *_ptr;
  return _out;
}

std::string toString(const BindingData::FeaturePointer& _feat);
std::string toString(const BindingData::FeaturePointers& _feats);
std::string toString(const BindingData::BindingProxy& _proxy);

/*inline 
bool 
operator==(const BindingData::SimpleRelation& _rel1, 
	   const BindingData::SimpleRelation& _rel2)
{
  return std::string(_re1.m_from) == ;
}*/

/// makes a dot label out of a feature sets. exludes all features in
/// \p _exluded_features (default is that non are excluded)
template<class FeatureSetT>
std::string 
featureSetToDotLabel(const FeatureSetT& _fset, 
		     const std::set<std::string>& _excluded_features = std::set<std::string>(),
		     AbstractFeature::FeatureDisplayMode _mode = AbstractFeature::only_feature)
{
  std::string str;
  typename FeatureSetT::const_iterator i = _fset.begin();
  std::set<std::string>::const_iterator exclude = _excluded_features.begin();
  while(i!=_fset.end()) {					  
    while(exclude != _excluded_features.end() && *exclude < i->first)
      ++exclude;
    if(exclude == _excluded_features.end() || *exclude != i->first) {
      str += trimFeatureName(i->first) + ": ";
      std::stringstream stream;
      //    toDotLabel(boost::shared_ptr<AbstractFeature>());
      
      print_set_w_fct(stream, i->second, to_dot_label_fct(_mode));
      str += stream.str();
      ++i;
      if(i != _fset.end())
	str += "\\n";
    } else {
      ++i;
    }
  }
  //for_all(_fset,print_pair_fct<string,set<boost::shared_ptr<AbstractFeature> > >(str,"\\n"));
  return str;
}


/// makes a combined ID out of two IDs
std::string combinedID(const std::string& _id1, const std::string _id2);
std::ostream& operator<<(std::ostream& _out, const BindingData::Ambiguity& _issue);
std::ostream& operator<<(std::ostream& _out, const BindingData::ComparisonTrust& _trust);
std::ostream& operator<<(std::ostream& _out, const BindingData::ComparisonTrustSpecification& _spec);
std::ostream& operator<<(std::ostream& _out, const BindingData::FeatureComparisonCompetence& _competence);

const std::string&
to_string(BindingData::BindingProxyType _t); 

const std::string&
to_string(BindingData::BindingProxyState _state);

/// analyses an ambiguity in more detail
void analyseAmbiguity(std::ostream& _out,
		      AbstractBindingWMRepresenter& _bindingWMRepresenter,
		      const BindingData::Ambiguity& _ambiguity);

} // namespace Binding

#endif // BINDING_UTILS_H_
