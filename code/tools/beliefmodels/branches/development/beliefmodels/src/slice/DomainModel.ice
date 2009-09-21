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
	
	class SuperFormula extends beliefmodels::adl::EpistemicObject implements beliefmodels::adl::Formula  {
	};
	
	// The class UncertainSuperFormula extends the supertype with an uncertainty value
				
	class UncertainSuperFormula extends SuperFormula { 
		long unc;
	}; 
		
	// The class LogicalSuperFormula extends the supertype with a boolean value
	
	class LogicalSuperFormula extends SuperFormula { 
		bool truth;
	};  
		
	// A complex formula is a super formula of the form (LHS OP RHS). 
	// Because ComplexFormula implements the SuperFormula class, and LHS and RHS are specified to be 
	// of this class, we can create arbitrarily complex formulas. Furthermore, LHS and RHS can be 
	// logical formulas, or uncertain formulas (this is not determined a priori).  
				
	enum LogicalOp { and, or };				
					
	class ComplexFormula extends SuperFormula { 
		LogicalOp op;
		SuperFormula lhs;
		SuperFormula rhs;
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
	}; 
					
	// ===================================================================
	// DOMAINMODEL: VISUAL OBJECTS 
	
	enum Shape { cylindrical, square };
	
	enum ObjectType { box, ball, cube }; 
	
	enum Color { red, blue, yellow, green };
	
	// A property is always a ContinualFormula
	
	class ColorProperty extends ContinualFormula { 
		Color colorValue;
	};  
	
	sequence<ColorProperty> Colors;
	
	class ShapeProperty extends ContinualFormula { 
		Shape shapedValue;
	}; 
	
	// A material object is a formula with one or more colors, a shape, and an object type; 
	// the entirety has again an uncertain value associated with it. 
	
	class MaterialObject extends ContinualFormula { 
		ObjectType type;
		ColorProperty clr; 
		ShapeProperty shp; 
	}; 
	
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