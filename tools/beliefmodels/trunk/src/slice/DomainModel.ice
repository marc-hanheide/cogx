#ifndef DOMAINMODEL_ICE 
#define DOMAINMODEL_ICE
  
// ===================================================================
// MODULE: beliefmodels.domainmodel.sample
// 
// Defines sample domain models for belief models
// 
// Authors:		Geert-Jan M. Kruijff	<gj@dfki.de>
// 
// For an edit log, see the bottom of this file. 
//
// ===================================================================

#include <BeliefModels.ice>

module beliefmodels { 
module domainmodel { 
module cogx { 
	 
			
	// ===================================================================
	// GROUNDED DOMAIN MODEL 
	// A domain model is a collection of formula types, to define the 
	// interfaces Formula and GoalFormula used in the belief models ADL 
	// definition. We adopt a grounded domain model here: the "truth" value of 
	// a belief is based in how it is connected with other structures. 
	
	// The class SuperFormula implements the Formula interface, acting as supertype class. 
	// As an EpistemicObject, each SuperFormula has an identifier. 
	
	class SuperFormula extends beliefmodels::adl::Formula {
	};
	 
	// The class UncertainSuperFormula extends the supertype with an uncertainty value
				
	class UncertainSuperFormula extends SuperFormula { 
		float prob;
	}; 
		
	// The class LogicalSuperFormula extends the supertype with a boolean value
	
	class LogicalSuperFormula extends SuperFormula { 
		bool truth;
	};  
		
	// A complex formula is a super formula of the form (LHS OP RHS). 
	// Because ComplexFormula implements the SuperFormula class, and LHS and RHS are specified to be 
	// of this class, we can create arbitrarily complex formulas. Furthermore, LHS and RHS can be 
	// logical formulas, or uncertain formulas (this is not determined a priori).  
				
	enum LogicalOp { and, or , xor, none};				
	 	
					
	sequence<SuperFormula> SuperFormulaSeq;
	 	
	class ComplexFormula extends UncertainSuperFormula { 
		LogicalOp op;
		SuperFormulaSeq formulae;
	};
	 

	// GROUNDED BELIEFS
	// We extend the ADL notion of Belief with structure to indicate how
	// the belief is grounded in other structures. Grounding indicates several 
	// aspects: What other structures the belief is related to (by reference; Id), 
	// what the status of the belief is (by value; different "truth" values for
	// assertions, propositions), and a formula explaining the reason for the grounding 
	// status (empty, to indicate truth; or a complex formula, explaining a negative value). 

	sequence<string>Ids; 

	enum GroundStatus { assertionVerified, assertionFalsified, propositionTrue, propositionFalse, propositionAmbiguous}; 

	class Ground { 
		Ids indexSet;
		GroundStatus gstatus;
		string modality;  // source modality 
		beliefmodels::adl::Formula reason;
	}; 

	class GroundedBelief extends beliefmodels::adl::Belief { 
		Ground grounding;
	}; 
	
	// A continual formula is a formula that is possibly uncertain, and which can be 
	// either asserted, or be a proposition. 
	
	enum ContinualStatus { proposition, assertion };
	
	class ContinualFormula extends UncertainSuperFormula {
		ContinualStatus cstatus;
		bool polarity;
	}; 
					
	// ===================================================================
	// DOMAINMODEL: VISUAL OBJECTS 
	
	enum Shape { cylindrical, spherical, cubic, compact, elongated, unknownShape };
	
	enum ObjectType { box, ball, cube, mug, thing, unknownObjectType }; 
	
	enum Color { red, blue, yellow, green, white, black, orange, unknownColor };
	
	enum Graspable { grasp, nograsp , unknownGrasp } ;
	
	// A property is always a ContinualFormula
	class ObjectTypeProperty extends ContinualFormula { 
		ObjectType typeValue;
	}; 
	
	class LocationProperty extends ContinualFormula {
		string location;
	};
	
	 
	class GraspableProperty extends ContinualFormula {
		Graspable graspableValue;
	};
	
	// A property is always a ContinualFormula
	class ColorProperty extends ContinualFormula { 
		Color colorValue;
	};  
	
	sequence<ColorProperty> Colors;
	
	class ShapeProperty extends ContinualFormula { 
		Shape shapeValue;
	};  
	
	enum Saliency {low, high, unknownSaliency};
	
	class SaliencyProperty extends ContinualFormula {
		Saliency sal;
	}; 
	
	class LinguisticLabelProperty extends ContinualFormula { 
		string label;
	};  
	 
	class LinguisticAttributeProperty extends ContinualFormula {
		string attribute;
	};
	
	class UnionRefProperty extends ContinualFormula {
		string unionRef;
	};
	
	enum Proximity {proximal, distal, unknownProximity};
	
	class ProximityProperty extends ContinualFormula {
		Proximity prox;
	};
	
	// ==========================
	// FOREGROUNDING STUFF
	
	dictionary<string,double> SaliencyAssignment;
	
	
	
}; // end cogx 
}; // end domainmodel
}; // end beliefmodels

#endif

// ===================================================================
// EDIT LOG
// 
// GJ	090915	extended with grounded beliefs, continual formulas, complex formulas
// GJ	090710	started module
// 
