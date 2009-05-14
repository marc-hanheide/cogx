#ifndef MOTIVATION_BASIC_TRANSLATORS_H_
#define MOTIVATION_BASIC_TRANSLATORS_H_


#include "AbstractTranslators.hpp"

#include <sstream>

//HACK! some quick hacks tp rename gensyms for readability


// extern string conceptGensym(const string & _gensym,
// 		     const BindingFeature::Concept & _concept) {
//    string c(_concept.m_concept);
//    return c + "_" + _gensym;
// }

// extern string waypointGensym(const string & _gensym) {
//    return "wp_" + _gensym;
// }



template <class FeatureT>
std::string toPlanningFeature() {
  throw(cast::CASTException(__HERE__,"Unknown feature for toPlanningFeature: %s", cast::typeName<FeatureT>().c_str()));
}


template <>
std::string toPlanningFeature<BindingFeatures::Colour>() {
  return "colour";
}

template <>
std::string toPlanningFeature<BindingFeatures::Shape>() {
  return "shape";
}

template <>
std::string toPlanningFeature<BindingFeatures::AreaID>() {
  return "area-id";
}


///HORRID HACK

std::string toPlanningFeature(const std::string & _featureT) {
  
  if(_featureT == cast::typeName<BindingFeatures::Colour>()) {
    return toPlanningFeature<BindingFeatures::Colour>();
  }
  else if(_featureT == cast::typeName<BindingFeatures::Shape>()) {
    return toPlanningFeature<BindingFeatures::Shape>();
  }
  else if(_featureT == cast::typeName<BindingFeatures::AreaID>()) {
    return toPlanningFeature<BindingFeatures::AreaID>();
  }
  
  throw(cast::CASTException(__HERE__,"Unknown feature for toPlanningFeature: %s", _featureT.c_str()));
}



/**
 * Translates a union with a concept feature into an object of
 * whatever type the concept states. Adds a single object delclaration
 * to the planning state.
 */
struct ConceptTranslator : 
  public BasicUnionFeatureTranslator<BindingFeatures::Concept> {

  virtual ~ConceptTranslator(){};

  virtual void translateFeature(const UnionPtr & _union,
				const BindingFeatures::Concept & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const {
    //cout<<"blah "<<_var<<endl;
    ObjectDeclaration obj;
    

    //HACK not sure the best way to do this ..
//     if(strcmp(_feature.m_concept, "robot") == 0) {
//       obj.name = CORBA::string_dup("MrChips");	  
//       obj.type = CORBA::string_dup("robot");
//     }
    if(strcmp(_feature.m_concept, "thing") == 0
       || strcmp(_feature.m_concept, "triangle") == 0
       || strcmp(_feature.m_concept, "circle") == 0
       || strcmp(_feature.m_concept, "square") == 0
       || strcmp(_feature.m_concept, "object") == 0
       || strcmp(_feature.m_concept, "Borland_book") == 0
       || strcmp(_feature.m_concept, "book") == 0
) {
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("movable");
    }
    else if(strcmp(_feature.m_concept, "addressee") == 0) {

      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("robot");

      //skip
      //return;
    }
    else if(strcmp(_feature.m_concept, "kitchen") == 0
	    || strcmp(_feature.m_concept, "office") == 0
	    || strcmp(_feature.m_concept, "library") == 0
	    || strcmp(_feature.m_concept, "corridor") == 0
	    || strcmp(_feature.m_concept, "hall") == 0
	    ) {
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("area-name");


//       //HACK FOR TESTING
//     //cout<<"blah "<<_var<<endl;
//       ObjectDeclaration obj2;
//       //obj.type = CORBA::string_dup(_feature.m_concept); 
//       obj2.name = CORBA::string_dup("dummy");
//       obj2.type = CORBA::string_dup("area-id");
//       _state.m_objectList.insert(obj2);    


//       Fact fact;
//       fact.modality = NO_MODALITY;
//       fact.agent = CORBA::string_dup("");
//       fact.name = CORBA::string_dup(toPlanningFeature<BindingFeatures::AreaID>().c_str());
//       fact.arguments.length(1);
//       fact.arguments[0]  = CORBA::string_dup(_var.c_str());
//       fact.value = CORBA::string_dup("dummy");
//       _state.m_factList.insert(fact);
      

    }
    else if(strcmp(_feature.m_concept, "ColorGame") == 0
	    || strcmp(_feature.m_concept, "colourGame") == 0
	    || strcmp(_feature.m_concept, "colorGame") == 0
	    || strcmp(_feature.m_concept, "colour_game") == 0
	    ) {
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("game");


      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup("game");
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(_var.c_str());
      fact.value = CORBA::string_dup("colour_game");
      _state.m_factList.insert(fact);

    }
    else if(strcmp(_feature.m_concept, "ShapeGame") == 0
	    || strcmp(_feature.m_concept, "shapeGame") == 0
	    || strcmp(_feature.m_concept, "shape_game") == 0
	    ) {
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("game");

      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup("game");
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(_var.c_str());
      fact.value = CORBA::string_dup("shape_game");
      _state.m_factList.insert(fact);

    }
    else if(strcmp(_feature.m_concept, "UnknownGame") == 0
	    || strcmp(_feature.m_concept, "unknownGame") == 0
	    || strcmp(_feature.m_concept, "unknown_game") == 0
	    ) {
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("game");

      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup("game");
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(_var.c_str());
      fact.value = CORBA::string_dup("unknown_game");
      _state.m_factList.insert(fact);

    }
    else {    
      //default, assign concept
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup(_feature.m_concept); 
    }
    
    _state.m_objectList.insert(obj);    
  }
};

/**
 * Translates a union with a concept feature into an object of
 * whatever type the concept states. Adds a single object delclaration
 * to the planning state.
 */
struct AreaIDTranslator : 
  public BasicUnionFeatureTranslator<BindingFeatures::AreaID> {

  virtual ~AreaIDTranslator(){};

  virtual void translateFeature(const UnionPtr & _union,
				const BindingFeatures::AreaID & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const {

    //new approach, use area-id number directly as area-id of overall var
    
    
    std::stringstream out;
    out << "area_id_"<<_feature.m_id;
    std::string areaIDVar(out.str());
    
    //cout<<"blah "<<_var<<endl;
    ObjectDeclaration areaIDDecl;
    //areaIDDecl.type = CORBA::string_dup(_feature.m_concept); 
    areaIDDecl.name = CORBA::string_dup(areaIDVar.c_str());
    areaIDDecl.type = CORBA::string_dup(toPlanningFeature<BindingFeatures::AreaID>().c_str());
    _state.m_objectList.insert(areaIDDecl);    


    //cout<<"blah "<<_var<<endl;
    ObjectDeclaration areaDecl;
    //areaDecl.type = CORBA::string_dup(_feature.m_concept); 
    areaDecl.name = CORBA::string_dup(_var.c_str());
    areaDecl.type = CORBA::string_dup("area-name");
    _state.m_objectList.insert(areaDecl);  


    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(toPlanningFeature<BindingFeatures::AreaID>().c_str());
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_var.c_str());
    fact.value = CORBA::string_dup(areaIDVar.c_str());
    _state.m_factList.insert(fact);

    Fact fact2;
    fact2.modality = NO_MODALITY;
    fact2.agent = CORBA::string_dup("");
    fact2.name = CORBA::string_dup("area-name");
    fact2.arguments.length(1);
    fact2.arguments[0]  = CORBA::string_dup(areaIDVar.c_str());
    fact2.value = CORBA::string_dup(_var.c_str());
    _state.m_factList.insert(fact2);
  
  }
};

/**
 * Translates a union with a colour feature into a fact about that
 * union.
 */
struct ColourTranslator : 
  public BasicUnionFeatureTranslator<BindingFeatures::Colour> {

  virtual ~ColourTranslator(){};

  virtual void translateFeature(const UnionPtr & _union,
				const BindingFeatures::Colour & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const {
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(toPlanningFeature<BindingFeatures::Colour>().c_str());
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_var.c_str());
    fact.value = CORBA::string_dup(_feature.m_colour);
    _state.m_factList.insert(fact);
  }
};

/**
 * Translates a union with a shape feature into a fact about that
 * union.
 */
struct ShapeTranslator : 
  public BasicUnionFeatureTranslator<BindingFeatures::Shape> {

  virtual ~ShapeTranslator(){};

  virtual void translateFeature(const UnionPtr & _union,
				const BindingFeatures::Shape & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const {
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(toPlanningFeature<BindingFeatures::Shape>().c_str());
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_var.c_str());
    fact.value = CORBA::string_dup(_feature.m_shape);
    _state.m_factList.insert(fact);
  }
};


/**
 * Translates a union with a location feature into a waypoint
 * object. Adds a single object delclaration to the planning state.
 */
struct LocationTranslator : 
  public BasicUnionFeatureTranslator<BindingFeatures::Location> {

  virtual ~LocationTranslator(){};

  virtual void translateFeature(const UnionPtr & _union,
				const BindingFeatures::Location & _feature,
				const string & _var,
				TemporaryPlanningState & _state) const {

    //only do this if there is no concept in the union... concepts
    //override locations
    if(!_union->hasFeature<BindingFeatures::Concept>()) {
      ObjectDeclaration obj;
      obj.name = CORBA::string_dup(_var.c_str());
      obj.type = CORBA::string_dup("waypoint");
      _state.m_objectList.insert(obj);    
    }
  }
};

/**
 * converts a binary relation with label "rl" into the fact of a form,
 (rl _toVar : _fromVar)
 */
struct RelationLabelToUnaryTranslator : 
  public BinaryRelationUnionTranslator {

  virtual ~RelationLabelToUnaryTranslator(){};

  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const {

    const BindingFeatures::RelationLabel & rl(_relUnion->getFeature<BindingFeatures::RelationLabel>());
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(rl.m_label);
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_toVar.c_str());
    fact.value = CORBA::string_dup(_fromVar.c_str());
    _state.m_factList.insert(fact);
  }


};

/**
 * converts a binary relation with label "rl" into the fact of a form,
 (rl _fromVar : _toVar)
 */
struct ReversedRelationLabelToUnaryTranslatorWithTemporalFrame : 
  public BinaryRelationUnionTranslator {

  virtual ~ReversedRelationLabelToUnaryTranslatorWithTemporalFrame(){};

  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const {

    const BindingFeatures::RelationLabel & rl(_relUnion->getFeature<BindingFeatures::RelationLabel>());
    string label(rl.m_label);
    
    if(_relUnion->hasFeature<BindingFeatures::TemporalFrame>()) {

      const BindingFeatures::TemporalFrame & tf(_relUnion->getFeature<BindingFeatures::TemporalFrame>());
      
      if(tf.m_temporalFrame == BindingFeaturesCommon::PERCEIVED) {
	label = "perceived-" + label;
      }
      else if(tf.m_temporalFrame == BindingFeaturesCommon::ASSERTED) {
	label = "asserted-" + label;
      }
      else if(tf.m_temporalFrame == BindingFeaturesCommon::TYPICAL) {
	// this reflects conceptual default relations, as e.g. provided by coma.sa	
	label = "asserted-" + label;
      }
      else {
	//HACK defaulting to perceived
      label = "perceived-" + label;
      }
    }
    else {
      //HACK defaulting to perceived
      label = "perceived-" + label;
    }

    
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(label.c_str());
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_fromVar.c_str());
    fact.value = CORBA::string_dup(_toVar.c_str());
    _state.m_factList.insert(fact);
  }


};



/**
 * converts a binary relation with label "rl" into the fact of a form,
 (rl _fromVar : _toVar)
 */
struct ReversedRelationLabelToUnaryTranslator : 
  public BinaryRelationUnionTranslator {

  virtual ~ReversedRelationLabelToUnaryTranslator(){};

  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const {

    const BindingFeatures::RelationLabel & rl(_relUnion->getFeature<BindingFeatures::RelationLabel>());
    string label(rl.m_label);
    
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(label.c_str());
    fact.arguments.length(1);
    fact.arguments[0]  = CORBA::string_dup(_fromVar.c_str());
    fact.value = CORBA::string_dup(_toVar.c_str());
    _state.m_factList.insert(fact);
  }


};


///HACK way of renaming relations for planning domains
extern string translateRelationLabel(const CORBA::String_member & _label) {
  if(strcmp(_label,"left") == 0) {
    return "wp_left_of";
  }
  if(strcmp(_label,"right") == 0) {
    return "wp_right_of";
  }
  if(strcmp(_label,"front") == 0) {
    return "wp_front_of";
  }
  if(strcmp(_label,"back") == 0) {
    return "wp_back_of";
  }
  if(strcmp(_label,"near") == 0) {
    return "wp_near";
  }

  return string(_label);

}


/**
 * converts a binary relation with label "rl" into the fact of a form,
 (rl _toVar  _fromVar : true)
 */
struct RelationLabelToBinaryTranslator : 
  public BinaryRelationUnionTranslator {

  virtual ~RelationLabelToBinaryTranslator(){};

  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const {
    const BindingFeatures::RelationLabel & rl(_relUnion->getFeature<BindingFeatures::RelationLabel>());
    Fact fact;
    fact.modality = NO_MODALITY;
    fact.agent = CORBA::string_dup("");
    fact.name = CORBA::string_dup(translateRelationLabel(rl.m_label).c_str());
    fact.arguments.length(2);
    fact.arguments[0]  = CORBA::string_dup(_fromVar.c_str());
    fact.arguments[1]  = CORBA::string_dup(_toVar.c_str());
    fact.value = CORBA::string_dup("true");
    _state.m_factList.insert(fact);
  }


};



/**
 * Ugly hack translator for explorer position... everything should be more general
 */
struct PositionWithIDTranslator : 
  public BinaryRelationUnionTranslator {

  virtual ~PositionWithIDTranslator(){};

  virtual void translate(const UnionPtr & _relUnion,
			 const string & _relVar,
			 const UnionPtr & _fromUnion,
			 const string & _fromVar,
			 const UnionPtr & _toUnion,
			 const string & _toVar,
			 TemporaryPlanningState & _state) const {

    const BindingFeatures::RelationLabel & rl(_relUnion->getFeature<BindingFeatures::RelationLabel>());
    string label(rl.m_label);
  
    //enforce temp frame
    assert(_relUnion->hasFeature<BindingFeatures::TemporalFrame>());

    const BindingFeatures::TemporalFrame & tf(_relUnion->getFeature<BindingFeatures::TemporalFrame>());
    
    if(tf.m_temporalFrame == BindingFeaturesCommon::PERCEIVED) {
      label = "perceived-" + label;

      //perceived rels need to to to teh area_id from the to union


      assert(_toUnion->hasFeature<BindingFeatures::AreaID>());
      const BindingFeatures::AreaID & areaID(_toUnion->getFeature<BindingFeatures::AreaID>());

      std::stringstream out;
      out << "area_id_"<<areaID.m_id;
      std::string areaIDVar(out.str());
    
      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup(label.c_str());
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(_fromVar.c_str());
      fact.value = CORBA::string_dup(areaIDVar.c_str());
      _state.m_factList.insert(fact);
    }
    else {
      label = "asserted-" + label;
      
      //asserted positions can just add the position rel between the two unions
      Fact fact;
      fact.modality = NO_MODALITY;
      fact.agent = CORBA::string_dup("");
      fact.name = CORBA::string_dup(label.c_str());
      fact.arguments.length(1);
      fact.arguments[0]  = CORBA::string_dup(_fromVar.c_str());
      fact.value = CORBA::string_dup(_toVar.c_str());
      _state.m_factList.insert(fact);
 
    }
  }
};


#endif
