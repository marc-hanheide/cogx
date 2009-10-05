#ifndef BELIEF_ICE
#define BELIEF_ICE

// ===================================================================
// MODULE: beliefmodels.adl
// 
// Defines the Abstract Definition Layer for belief models
// 
// Authors:		Geert-Jan M. Kruijff	<gj@dfki.de>
// 
// For an edit log, see the bottom of this file. 
//
// ===================================================================

module beliefmodels { 
	module adl { 
		
		// ===================================================================		
		// EPISTEMIC OBJECTS AND RELATIONS
		// An epistemic object is an object with an identifier, which can 
		// be explicitly referenced. Relations of arbitrary arity can be 
		// defined over epistemic objects. 
		
		class EpistemicObject { 
			string id;
		}; // end EpistemicObject
		 

		class RelationArgument { 
			int argPos;
			EpistemicObject arg; 
		}; 
		
		sequence<RelationArgument> RelationArguments;

		class EpistemicRelation extends EpistemicObject { 
			string type;
			RelationArguments args;
		}; 
		
		sequence<EpistemicRelation> EpistemicRelations;
		
		// ===================================================================
		// AGENTS
		// Formally, an agent just acts as an identifier
		
		// An Agent is a simple epistemic object with an (inherited) identifier
		
		class Agent extends EpistemicObject { 

		}; // end struct Agent
		
		// A set of agents is a sequence of Agent structs
		
		sequence<Agent> Agents; 
		
		// ===================================================================
		// SPATIO-TEMPORAL FRAMES
		// Formally, a spatiotemporal frame is a tuple over a spatial interval, 
		// a temporal interval, an agent-relative perspective, and a point-index. 
		// We define a class SpatioTemporalFrame, to serve as a placeholder
		// in the belief model structure. 
		
		
		// The class SpatialInterval defines a minimal structure, namely just 
		// an (inherent) identifier used for identifying an area. 
		
		class SpatialInterval extends EpistemicObject{ 

		}; // end SpatialInterval
		
		// The class TemporalInterval defines a minimal temporal structure, 
		// consisting of a start, an end, and a point identifier for the 
		// interval (inherited identifier). Start and end are modeled as 
		// strings, to provide a qualitative level of representations in which 
		// we can also deal with futures. 
		
		class TemporalInterval extends EpistemicObject { 
			string start;
			string end; 
		}; // end TemporalInterval

		// The class Perspective defines a minimal structure, including just
		// a sequence of agents for which the perspective holds.

		class Perspective extends EpistemicObject { 
			Agents	ags; 
		}; // end Perspective

		// The class SpatioTemporalFrame is the basic class specifying a spatio-temporal frame. 
				
		class SpatioTemporalFrame extends EpistemicObject { 
			SpatialInterval		spatialint;
			TemporalInterval	tempint;
			Perspective			persp; 
		}; // end SpatioTemporalFrame
		
		sequence<SpatioTemporalFrame> SpatioTemporalFrames; 
		
		// A SpatioTemporal model is a collection of spatio-temporal frames, and a collection of 
		// relations defined between subsets of spatio-temporal frames. 
		
		class SpatioTemporalModel { 
			SpatioTemporalFrames	frames;
			EpistemicRelations		connex;
		}; // end SpatioTemporalModel
		
		// ===================================================================
		// BELIEFS
		// A belief is a structure consisting of a spatio-temporal frame, a 
		// set of agents, and a formula. 

		// A formula is an empty interface. A domain model needs to define a 
		// collection of classes, which directly or through inheritance implement
		// the interface. 

		class Formula extends EpistemicObject { 		
		}; // 

		class Belief extends EpistemicObject { 
			SpatioTemporalFrame sigma; 
			Agents ags;
			Formula phi; 
		}; // end Belief
		
		sequence<Belief> Beliefs; 
		
		
		
		// ===================================================================
		// TASKS 
		// A task is a structure consisting of a spatio-temporal frame, a
		// set of agents, and a goal-formula. 
		
		// A goal formula is an empty interface. A domain model needs to define a 
		// collection of classes, which directly or through inheritance implement
		// the interface. 

		interface GoalFormula { 		
		}; //  

		class Task extends EpistemicObject { 
			SpatioTemporalFrame sigma;
			Agents ags;
			GoalFormula goal;
		}; // end Task
		
		sequence<Task> Tasks;
		
		// ===================================================================
		// FOREGROUND
		// The foreground collection is an interface, which needs to define access
		// to foregrounded beliefs and tasks. The exact nature of the 
		// foreground collection (e.g. a stack, etc.) is to be defined at the 
		// implementation level. 
		
		interface Foreground { 
			Beliefs	getForegroundedBeliefs();
			Tasks	getForegroundedTasks();
			void	addForegroundedBelief(Belief K);
			void	addForegroundedTask(Task T);
			void	removeForegroundedBelief (Belief K);
			void	removeForegroundedTask (Task T);
		}; // end Foregrounding

		// ===================================================================
		// BELIEF MODEL
		// A belief model is a tuple consisting of a set of agents, a spatio- 
		// temporal model, a set of beliefs, a set of tasks, and the foregrounding

		class BeliefModel { 
			Agents   a;
			SpatioTemporalModel s;
			Beliefs	 k;
			Tasks	 t;
			Foreground f;
		}; 

	}; // end module adl
}; // end beliefmodels

#endif

// ===================================================================
// TO DO
//
// 090709	need to extend EpistemicObject to provide co-indexation mechanisms
// 090709	there should be a "check" or an exception for out-of-bounds/undefined arguments in an epistemic relation; no checks are made currently on contiguity
//
// EDIT LOG
//
// GJ	090710	started module