#ifndef BINDING_INTERNAL_COMPARATOR_H_
#define BINDING_INTERNAL_COMPARATOR_H_

//#include <idl/BindingFeatures.hh>
#include <boost/shared_ptr.hpp>
#include <boost/logic/tribool.hpp>
#include <iostream>
#include <memory>
//#include <stdexcept> 
//#include "feature-utils/FeaturePropertiesUtils.hpp"
//#include "feature-utils/AbstractFeature.hpp"

#include <boost/static_assert.hpp>
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/BindingException.hpp"
#include "ComparatorIndex.hpp"
#include "FeatureExtractor.hpp"

namespace Binding {
//forwards decl
class AbstractFeature;

class AbstractInternalComparator {  
protected:
  
  /// To denote if a comparator is symmetric or not
  enum Symmetric {
    /// the comparator is the same in both directions
    SYMMETRIC,
    NONSYMMETRIC,
  };
  
protected:
  
private: 
  /// keeps track of what features the comparator is for
  //std::auto_ptr<const ComparatorIndex> m_comparatorIndex;
public:
  /*  const ComparatorIndex& comparatorIndex() const {
      if(m_comparatorIndex.get() == NULL) {
      throw BindingException("Error: Comparator index not set for internal comparator");
      }
      return *m_comparatorIndex;
      }
  */
/*  template<typename ProxyIDLFeatureT, typename UnionIDLFeatureT>
  ComparatorIndex getComparatorIndex(BindingFeatureOntology& _ontology) const {    
    return ComparatorIndex(_ontology.featureNumber(typeid(ProxyIDLFeatureT)),_ontology.featureNumber(typeid(UnionIDLFeatureT)));
  }*/
protected:
  AbstractInternalComparator() {}
  /// constructor \p _sym is defaut symmetric, i.e. the comparator
  /// will be called independently of which of the features is in the
  /// proxy or union
  /*  template<typename ProxyIDLFeatureT, typename UnionIDLFeatureT>
      AbstractInternalComparator(BindingFeatureOntology& _ontology, Symmetric _sym = SYMMETRIC) {
    _registerComparator<ProxyIDLFeatureT,UnionIDLFeatureT>(_ontology);
    if(_sym == SYMMETRIC)
      _registerComparator<UnionIDLFeatureT,ProxyIDLFeatureT>(_ontology);
  }
  
  template<typename IDLFeatureT>
  AbstractInternalComparator(BindingFeatureOntology& _ontology) {
  
  }
  */

  virtual ~AbstractInternalComparator() {}
  /// the main function which performs the actual comparison
public:
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const = 0;

  virtual void registerWithOntology(BindingFeatureOntology& _ontology, 
				    boost::shared_ptr<const AbstractInternalComparator>& _comp) const = 0;

private:
  /// not copyable
  AbstractInternalComparator(const AbstractInternalComparator&);
  /// not assignable
  const AbstractInternalComparator& operator=(const AbstractInternalComparator&);

};

/*
template<typename ProxyIDLFeatureT, typename UnionIDLFeatureT>
class NonsymmetricInternalComparator : public AbstractInternalComparator {
protected:
  typedef ProxyIDLFeatureT ProxyIDLFeature;
  typedef UnionIDLFeatureT UnionIDLFeature;
  NonsymmetricInternalComparator()
  {
  }
  /// a simplified interface to cast an abstract feature into the
  /// feature for which the comparator is implemented
  const ProxyIDLFeature& getProxyIDLFeature(const AbstractFeature& _abstractFeature) const {
    return Binding::extractIDLFeature<ProxyIDLFeature>(_abstractFeature);
  }
  /// a simplified interface to cast an abstract feature into the
  /// feature for which the comparator is implemented

  const UnionIDLFeature& getUnionIDLFeature(const AbstractFeature& _abstractFeature) const {
    return Binding::extractIDLFeature<UnionIDLFeature>(_abstractFeature);
  }

  virtual void registerWithOntology(BindingFeatureOntology& _ontology,
				    boost::shared_ptr<const AbstractInternalComparator>& _comp) const {
    //_ontology.m_internalComparators.insert(std::make_pair(getComparatorIndex<ProxyIDLFeatureT,UnionIDLFeatureT>(_ontology),_comp)); 
    _ontology.registerInternalComparator(typeid(ProxyIDLFeatureT),typeid(UnionIDLFeatureT), _comp);
  }

};
*/

template<typename IDLFeature1T, typename IDLFeature2T>
class SymmetricInternalComparator : public AbstractInternalComparator {
protected:
//  BOOST_STATIC_ASSERT(std::typeid(ProxyIDLFeatureT) != std::typeid(UnionIDLFeatureT));
  typedef IDLFeature1T IDLFeature1; 
  typedef IDLFeature2T IDLFeature2;
  SymmetricInternalComparator()
  {
//    setComparatorIndex<IDLFeature1T,IDLFeature2T>(_ontology);
//    setComparatorIndex<IDLFeature2T,IDLFeature1T>(_ontology);
  }
  /// a simplified interface to cast an abstract feature into the
  /// feature for which the comparator is implemented (since we get
  /// two features, we need to translate the one of them that has the
  /// right type, the other one is ignored)
  const IDLFeature1& getIDLFeature1(const AbstractFeature& _abstractFeatureA, 
				    const AbstractFeature& _abstractFeatureB) const {
    if(_abstractFeatureA.typeInfo() == typeid(IDLFeature1))
      return Binding::extractIDLFeature<IDLFeature1>(_abstractFeatureA);
    return Binding::extractIDLFeature<IDLFeature1>(_abstractFeatureB);
  }
  /// a simplified interface to cast an abstract feature into the
  /// feature for which the comparator is implemented
  const IDLFeature2& getIDLFeature2(const AbstractFeature& _abstractFeatureA, 
				    const AbstractFeature& _abstractFeatureB) const {
    if(_abstractFeatureA.typeInfo() == typeid(IDLFeature2))
      return Binding::extractIDLFeature<IDLFeature2>(_abstractFeatureA);
    return Binding::extractIDLFeature<IDLFeature2>(_abstractFeatureB);
  }

  virtual void registerWithOntology(BindingFeatureOntology& _ontology,
				    boost::shared_ptr<const AbstractInternalComparator>& _comp) const {
    _ontology.registerInternalComparator(typeid(IDLFeature1T),typeid(IDLFeature2T),_comp); 
    _ontology.registerInternalComparator(typeid(IDLFeature2T),typeid(IDLFeature1T),_comp); 
  }

};

template<typename IDLFeatureT>
class ReflexiveInternalComparator : public AbstractInternalComparator {
protected:
  //BOOST_STATIC_ASSERT(std::typeid(ProxyIDLFeatureT) != std::typeid(UnionIDLFeatureT));
  typedef IDLFeatureT IDLFeature;
  ReflexiveInternalComparator() 
  {
   // setComparatorIndex<IDLFeature,IDLFeature>(_ontology);
  }
  /// a simplified interface to cast an abstract feature into the
  /// feature for which the comparator is implemented
  const IDLFeature& getIDLFeature(const AbstractFeature& _abstractFeature) const {
    return Binding::extractIDLFeature<IDLFeature>(_abstractFeature);
  }
  virtual void registerWithOntology(BindingFeatureOntology& _ontology,
				    boost::shared_ptr<const AbstractInternalComparator>& _comp) const {
    _ontology.registerInternalComparator(typeid(IDLFeature),typeid(IDLFeature),_comp); 
  }
};



} // namespace Binding

#endif // BINDING_INTERNAL_COMPARATOR_H_
