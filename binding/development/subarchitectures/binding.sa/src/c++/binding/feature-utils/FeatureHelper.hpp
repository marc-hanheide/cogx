#ifndef BINDING_FEATURE_HELPER_H_
#define BINDING_FEATURE_HELPER_H_

#include <boost/shared_ptr.hpp>
#include <balt/core/StringMap.hpp>
#include <string>
#include <memory>
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/abstr/AbstractBindingWMRepresenter.hpp"

namespace BindingData {
class FeaturePointer;
}

namespace Binding {

class BindingFeatureOntology;
class AbstractBindingWMRepresenter;
class AbstractFeature;
class FeatureProperties;

class AbstractFeatureHelper {
public:
  AbstractFeatureHelper(){};
  /// is supposed to print the contents of a feature
  virtual std::ostream& print(std::ostream&, const AbstractFeature& _feat) const = 0;

  /// is supposed to print the contents of a feature in a format that
  /// fits into a dot label (some characters are not allowed, and be
  /// careful with escaping stuff).
  /// \remark calls \p print(...) unless overridden in subclasses
  virtual std::ostream& printDotLabel(std::ostream& _out, const AbstractFeature& _feat) const  {
    if(_feat.negated()) {
      _out << "~";
    }
    return print(_out, _feat);
  }

  /// returns \p m_properties
  const FeatureProperties& properties() const {assert(m_properties.get());return *m_properties;}
  /// sets the properties
  void setProperties(const FeatureProperties&);

  /// calls \p operatorLessImpl(_feat). Is used to order the feature sets
  /// and get rid of duplicates of a feature type with several
  /// equivalent members. Can only be used on features of the same
  /// type. 
  /// \remark This is NOT the same as comparing features to evaluate if they are equivalent
  bool operatorLess(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;

  /// returns the type_info for the feature type for which the Helper
  /// is dedicated
  virtual const std::type_info& typeInfo() const = 0;
  
  virtual boost::shared_ptr<AbstractFeature> getFeatureFromWM(AbstractBindingWMRepresenter& _a, 
							      const BindingData::FeaturePointer& _feat,
							      const std::string& _bindingSubarchID) const = 0;
  

protected:
  /// should return partial ordering of one feature type (_feat type
  /// is equivalent to this type, asserted)
  /// \remark: default implementation is that the pointers are compared, meaning that no duplicates will be removed in the feature set, makes sense for some features
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const; 
  virtual ~AbstractFeatureHelper(){};
  
private:
  /// returns a nonconst reference... used by the ontology... while refactoring...
  FeatureProperties& mutable_properties() {assert(m_properties.get()); return *m_properties;}
  /// to allow that properties are set by the ontology... makes sense during refactoring...
  friend class BindingFeatureOntology;
  std::auto_ptr<FeatureProperties> m_properties;
};

const std::type_info& getTypeInfo(const AbstractFeature&);

template<typename IDLFeatureT>
class FeatureHelper : public AbstractFeatureHelper {
  typedef IDLFeatureT IDLFeature;
public:
  /// just a shortcut for implementers of helpers
  const IDLFeature& extract(const AbstractFeature& _abstractFeature) const {
    return Binding::extractIDLFeature<IDLFeature>(_abstractFeature);
  }
  
  virtual const std::type_info& typeInfo() const {return typeid(IDLFeature);}

  virtual boost::shared_ptr<AbstractFeature> getFeatureFromWM(AbstractBindingWMRepresenter& _a, 
							      const BindingData::FeaturePointer& _feat,
							      const std::string& _bindingSubarchID) const {
    std::string address(_feat.m_address);
    
    if(_bindingSubarchID == "")
      return boost::shared_ptr<AbstractFeature>(new Feature<IDLFeatureT>(*_a.loadBindingDataFromWM<IDLFeatureT>(address), address));
    else
      return boost::shared_ptr<AbstractFeature>(new Feature<IDLFeatureT>(*_a.loadBindingDataFromWM<IDLFeatureT>(_bindingSubarchID,address), address));
  }

  
  /*const Feature<IDLFeature>& getFeature(const AbstractFeature& _abstractFeature) {
    return Binding::extractFeature<IDLFeature>(_abstractFeature);
  }*/

/*  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const {
    assert(typeid(IDLFeature) == getTypeInfo(_feat));
    _out << "print("<< typeid(IDLFeature).name() 
	 << "...) not implemented in the FeatureHelper for this feature type"; 
    return _out;
  }
*/
  
/*  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const {
    std::ostream& print(std::ostream& _out, getIDLFeature(_feat))
  }
  
  /// needs to be implemented
  virtual std::ostream& print(std::ostream& _out, const IDLFeatureT& _feat) const = 0;
*/  
   
};

} // namespace Binding

#endif // BINDING_FEATURE_HELPER_H_
