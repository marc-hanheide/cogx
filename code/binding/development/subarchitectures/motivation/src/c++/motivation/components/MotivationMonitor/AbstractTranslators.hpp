#ifndef MOTIVATION_ABSTRACT_TRANSLATORS_H_
#define MOTIVATION_ABSTRACT_TRANSLATORS_H_

#include <binding/utils/LocalClasses.hpp>
#include <planning/util/TemporaryPlanningState.hpp>


using namespace cast;
using namespace cast::cdl;
using namespace std;
using namespace boost;
using namespace Binding;


/**
 * Determines whether a translator is applicable or not.
 */
struct TranslatorApplicabilityFunctor :
  public std::unary_function<const UnionPtr &, bool> {
  virtual bool operator()(const UnionPtr & _union) const = 0;
protected:
  virtual ~TranslatorApplicabilityFunctor(){}
};

/**
 * Applicable if the union is of basic type.
 */
struct BasicUnionApplicable :
  public TranslatorApplicabilityFunctor {
  virtual bool operator()(const UnionPtr & _union) const {
    return _union->type() == BindingData::BASIC;
  };
  virtual ~BasicUnionApplicable(){}
};

/**
 * Applicable if the union is of relation type.
 */
struct RelationUnionApplicable :
  public TranslatorApplicabilityFunctor {
  virtual bool operator()(const UnionPtr & _union) const {
    return _union->type() == BindingData::RELATION;
  };
  virtual ~RelationUnionApplicable(){}
};

/**
 * Applicable if the union has a particular feature
 */
template <class FeatureT>
struct HasFeatureApplicable :
  public TranslatorApplicabilityFunctor {
  virtual bool operator()(const UnionPtr & _union) const {
    return _union->hasFeature<FeatureT>();
  };
  virtual ~HasFeatureApplicable(){}
};

/**
 * Applicable if the union has a feature that matches the one provided
 * in constructor.
 */
template <class FeatureT>
struct MatchesFeatureApplicable :
  public TranslatorApplicabilityFunctor {
  
  MatchesFeatureApplicable(const FeatureT & _feature) :
    m_feature(_feature,"") {}

  virtual bool operator()(const UnionPtr & _union) const {
    Feature<FeatureT> feature(_union->getFeature<FeatureT>(), "");
    return m_feature == feature;
  };
  
  virtual ~MatchesFeatureApplicable(){}
  
protected: 
  Feature<FeatureT> m_feature;
};




/**
 * Applicable if the union has a feature that matches the one provided
 * in constructor.
 */
struct RelationLabelApplicable :
  public TranslatorApplicabilityFunctor {
  
  RelationLabelApplicable(const string & _label) {
    BindingFeatures::RelationLabel rl;
    rl.m_label = CORBA::string_dup(_label.c_str());
    relApp = new MatchesFeatureApplicable<BindingFeatures::RelationLabel>(rl);
  }

  virtual bool operator()(const UnionPtr & _union) const {
    return (*relApp)(_union);
  };

  virtual ~RelationLabelApplicable(){ delete relApp;}
 
protected:
  //can't be bothered to work around constructors now...
  MatchesFeatureApplicable<BindingFeatures::RelationLabel> * relApp;
};






/**
 * Converts something into planning state
 */
struct AbstractTranslator  {
protected:
  virtual ~AbstractTranslator(){}
};

/**
 * Converts a basic union into planning state
 */
struct BasicUnionTranslator :
  public AbstractTranslator {

  /**
   * @param _union The union to convert
   * @param _var The var to be assigned to this union in the planning state.
   * @param _state The state to be extended.
   */
  virtual void translate(const UnionPtr & _union,
			 const string & _var,
			 TemporaryPlanningState & _state) const = 0;
protected:
  virtual ~BasicUnionTranslator(){}
};



/**
 * Converts a binary relation union into planning state
 */
struct BinaryRelationUnionTranslator :
  public AbstractTranslator {

  /**
   * @param _union The union to convert
   * @param _var The var to be assigned to this union in the planning state.
   * @param _state The state to be extended.
   */
  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const = 0;
protected:
  virtual ~BinaryRelationUnionTranslator(){}
};


/**
 * Converts a basic union into planning state based on a single type
 * of binding feature. NB if there are repetitions of the feature,
 * then these are ignored.
 */
template <class FeatureT>
struct BasicUnionFeatureTranslator :
  public BasicUnionTranslator {

  /**
   * Loads the feature then calls the subclass with the union + feature.
   *
   * @param _union The union to convert
   * @param _var The var to be assigned to this union in the planning state.
   * @param _state The state to be extended.
   */
  virtual void translate(const UnionPtr & _union,
			 const string & _var,
			 TemporaryPlanningState & _state)  const {
    //this should be checked before this is called
    assert(_union->hasFeature<FeatureT>());
    const FeatureT& feature(_union->getFeature<FeatureT>());
    translateFeature(_union, feature, _var, _state);
  }

protected:

  virtual void translateFeature(const UnionPtr & _union,
				const FeatureT & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const = 0;

  virtual ~BasicUnionFeatureTranslator(){}
};





/*
 * Abstract base class for generator functors for translating from a
 * single (i.e. basic) union to planning state.
 */
struct AbstractBasicStateGenerator {
  virtual bool operator()(const UnionPtr & _union, 
			  const string & _var,
			  TemporaryPlanningState & _state) const = 0;
protected:
  virtual ~AbstractBasicStateGenerator(){}
};


/*
 * Abstract base class for generator functors for translating from a
 * binary relation union structure to planning state.
 */
struct AbstractBinaryRelationStateGenerator {
  virtual bool operator()(const UnionPtr & _relUnion,
			  const string & _relVar,
			  const UnionPtr & _fromUnion,
			  const string & _fromVar,
			  const UnionPtr & _toUnion,
			  const string & _toVar,
			  TemporaryPlanningState & _state) const = 0;
  
protected:
  virtual ~AbstractBinaryRelationStateGenerator(){}
};




/**
 * Encapsulates the applicability and translation functionality into a
 * single object.
 */
template <class Applicable, class Translator>
struct BasicBindingStateGenerator :
  public AbstractBasicStateGenerator {

  virtual bool operator()(const UnionPtr & _union, 
			  const string & _var,
			  TemporaryPlanningState & _state) const {
    bool apply(m_basicCheck(_union) && m_applicable(_union));
    if(apply) {
      m_translator.translate(_union,_var,_state);
    }
    return apply;
  }
  
  virtual ~BasicBindingStateGenerator(){}

private:
  
  Applicable m_applicable;
  Translator m_translator;
  BasicUnionApplicable m_basicCheck;

};

/**
 * Encapsulates the applicability and translation functionality into a
 * single object.
 */



template <class Applicable, class Translator>
struct BasicBinaryRelationStateGenerator :
  public AbstractBinaryRelationStateGenerator {

  BasicBinaryRelationStateGenerator() :
    m_applicable(),
    m_translator(),
    m_relationCheck() {}
  
  //HACK! to allow the rel label checker to be used easily
  BasicBinaryRelationStateGenerator(const string & _label) :
    m_applicable(_label),
    m_translator(),
    m_relationCheck() {}
  

  virtual bool operator()(const UnionPtr & _relUnion,
			  const string & _relVar,
			  const UnionPtr & _fromUnion,
			  const string & _fromVar,
			  const UnionPtr & _toUnion,
			  const string & _toVar,
			  TemporaryPlanningState & _state) const {

    bool apply(m_relationCheck(_relUnion) && m_applicable(_relUnion));
    if(apply) {
      m_translator.translate(_relUnion,_relVar,
			     _fromUnion,_fromVar,
			     _toUnion,_toVar,			     
			     _state);
    }
    return apply;
  }
  
  virtual ~BasicBinaryRelationStateGenerator(){}

private:
  
  Applicable m_applicable;
  Translator m_translator;
  RelationUnionApplicable m_relationCheck;

};



#endif
