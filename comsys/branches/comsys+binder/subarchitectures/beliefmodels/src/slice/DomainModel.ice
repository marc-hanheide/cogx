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
	// As an EpistemicObject, each SuperFormula has an identifier. A SuperFormula provides a ContinualStatus field,  
	// and a Ground, which is a list of one or more union ids. 	
		
	enum ContinualStatus { assertionVerified, assertionFalsified, propositionTrue, propositionFalse, propositionAmbiguous };	
		
	sequence<string> Ground;					
		
	class SuperFormula extends beliefmodels::adl::EpistemicObject implements beliefmodels::adl::Formula  {
		ContinualStatus status;
		Ground grnd;
	};
	
	// The class UncertainSuperFormula extends the supertype with an uncertainty value
				
	class UncertainSuperFormula extends SuperFormula { 
		long unc;
	}; 
		
	// The class LogicalSuperFormula extends the supertype with a boolean value
	
	class LogicalSuperFormula extends SuperFormula { 
		bool truth;
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

