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
module sample { 
	
			
	// ===================================================================
	// DOMAIN MODEL 
	// A domain model is a collection of formula types, to define the 
	// interfaces Formula and GoalFormula used in the belief models ADL 
	// definition. 
	
	// The class SuperFormula implements the Formula interface, acting as supertype class. 
	// As an EpistemicObject, each SuperFormula has an identifier
		
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
		
					
	// ===================================================================
	// EXAMPLE: OBJECTS 
	
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
		Colors clrs; 
		ShapeProperty shp; 
	}; 


	// ===================================================================
	// EXAMPLE: LOCATIONS	

	enum LocationType { kitchen, office, printerroom, toilet };

	// A location is an epistemic object with a location type; the identifier(s)
	// inherited through EpistemicObject can be used to identify the location 
	// as such across different models. 

	class Location extends LogicalSuperFormula { 
		LocationType type;
	}; 

	// ===================================================================
	// EXAMPLE: ACTIONS

	// The ActionArgument class just defines a placeholder for optional 
	// arguments for an action

	class ActionArgument { 
		string argrole;
		beliefmodels::adl::Formula arg;
	}; 

	sequence<ActionArgument> ActionArguments;

	// The Action class provides the basic structure to include optional 
	// arguments. Any action extends this class, meaning that it only 
	// needs to specify the obligatory arguments in its formulation. 

	class Action extends LogicalSuperFormula { 
		ActionArguments optargs;	
	}; 

	// GoToLocation is an action which specifies a go-action towards a named location

	class GoToLocation extends Action { 
		beliefmodels::adl::Agents		actor; 
		Location	destination;
	}; 

	// DirectionType defines different directions

	enum DirectionType { left, right, forward, backward }; 
	
	// MoveDirection is an action which specifies a move-action towards a specific direction
	
	class MoveDirection extends Action { 
		beliefmodels::adl::Agents	actor;
		DirectionType direction;
	}; 
	
	// QualitativeSpeedType defines different qualitative speed characterizations
	
	enum QualitativeSpeedType { slow, normal, fast };
	
	// Speed is a class that can be used as and optional argument for a movement-like action
	
	class Speed extends LogicalSuperFormula { 
		QualitativeSpeedType	speedMode;
	}; 
	
	
	
	
}; // end sample 
}; // end domainmodel
}; // end beliefmodels

#endif

