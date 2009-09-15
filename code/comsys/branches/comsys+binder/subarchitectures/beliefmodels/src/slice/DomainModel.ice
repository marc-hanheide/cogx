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
	// DOMAIN MODEL 
	// A domain model is a collection of formula types, to define the 
	// interfaces Formula and GoalFormula used in the belief models ADL 
	// definition. 
	
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




	// ===================================================================
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
		GroundStatus status;
		beliefmodels::adl::Formula reason;
	}; 

	class GroundedBelief extends beliefmodels::adl::Belief { 
		Ground grounding;
	}; 
					
					
	// ===================================================================
	// EXAMPLE: VISUAL OBJECTS 
	
	enum Shape { cylindrical, square };
	
	enum ObjectType { box, ball, cube }; 
	
	enum Color { red, blue, yellow, green };
	
	// A property is a formula with a value, and a degree of (un)certainty associated with that value
	
	class ColorProperty extends UncertainSuperFormula { 
		Color colorValue;
	};  
	
	sequence<ColorProperty> Colors;
	
	class ShapeProperty extends UncertainSuperFormula { 
		Shape shapedValue;
	}; 
	
	// A material object is a formula with one or more colors, a shape, and an object type; 
	// the entirety has again an uncertain value associated with it. 
	
	class MaterialObject extends UncertainSuperFormula { 
		ObjectType type;
		ColorProperty clr; 
		ShapeProperty shp; 
	}; 



	
}; // end cogx 
}; // end domainmodel
}; // end beliefmodels

#endif

