#ifndef MOTIVATION_ICE
#define MOTIVATION_ICE

#include <cast/slice/CDL.ice>
#include <Planner.ice>

module motivation {
    module slice {
    
      interface RemoteFilterServer {
    	void setPriority(string motiveType, string priority);    	
      };
      
    

    	class TestSource {
    		string name;
    		cast::cdl::CASTTime time;
    	};
    
    
    	enum MotiveStatus {
    		UNSURFACED,
    		SURFACED,
    		POSSIBLE,
    		IMPOSSIBLE,
    		ACTIVE,
    		COMPLETED,
    		WILDCARD
    	};

      interface ExternalGoalServer {
      	MotiveStatus submitGoal(string goalString, float importance);
      };
    	
    	enum MotivePriority {
    		UNSURFACE,
    		LOW,
    		NORMAL,
    		HIGH
    	};
    	
		["java:type:java.util.LinkedList<cast.cdl.WorkingMemoryAddress>:java.util.List<cast.cdl.WorkingMemoryAddress>"] 
		sequence<cast::cdl::WorkingMemoryAddress> GoalSeq;

    	class PlannedGoals {
			GoalSeq goals;
			cast::cdl::WorkingMemoryAddress planningTask;
    	};
    		
    	class Motive {
			cast::cdl::CASTTime created;
			cast::cdl::CASTTime updated;
			cast::cdl::WorkingMemoryAddress referenceEntry;
			cast::cdl::WorkingMemoryAddress thisEntry;

    		MotiveStatus status;

    		/** a counter for the number of tries this motive has been planned */
    		long tries; 	

    		/** [0-1] encoding for a priority */
    		MotivePriority priority;

    		/** [0-inf] encoding for costs, -1 encodes infinity AKA hard goal */
    		double costs;


    		/** [0-1] encoding for information gain */
    		double informationGain;

    		/** rank of activated motives */
    		int rank;

    		/** the derived single goal for the planner */
    		autogen::Planner::Goal goal;

    		/** maxplanningTime in sec **/
    		int maxPlanningTime;
    		/** max execution time in sec **/
    		int maxExecutionTime;
    		
    	};
    	
    	class TestMotive extends Motive {
    		string value;
    		
    	};

    	class CannedTextMotive extends Motive {
    		string text;
    		
    	};

    	class HomingMotive extends Motive {
    		long homePlaceID;
    	};

    	class GeneralGoalMotive extends Motive {
    	};

    	class RobotInitiativeMotive extends Motive {
    	};

    	class TutorInitiativeMotive extends Motive {
    	};

    	class HypothesisVerificationMotive extends Motive {
			
    	};

    	class TutorInitiativeLearningMotive extends TutorInitiativeMotive {
			string assertedFeature;
			string assertedValue;	
			bool assertedLearn;		
    	};

    	class TutorInitiativeQuestionMotive extends TutorInitiativeMotive {
    	};


		class RobotNonSituatedMotive extends Motive {
    	};

    	class ExploreMotive extends Motive {
    		/**
	 		* The ID of the place.
	 		*/
			long placeID;
    		
    	};

    	class PatrolMotive extends Motive {
    		/**
	 		* The ID of the place.
	 		*/
			long placeID;
			
			/**
			* last time we have been here
			*/
			cast::cdl::CASTTime lastVisisted;
    		
    	};
    	
    	class CategorizePlaceMotive extends Motive {
    		/**
	 		* The ID of the place.
	 		*/
			long placeID;
    		
    	};

    	class CategorizeRoomMotive extends Motive {
      		/**
	 		* The ID of the place.
	 		*/
			long roomId;
    	};

		class AnalyzeProtoObjectMotive extends Motive {};

		class LearnObjectFeatureMotive extends Motive {
			/**
			* The feature to learn.
			*/
			string feature;
			
		};
		
			class LookAtViewConeMotive extends Motive {
	    	    cast::cdl::WorkingMemoryAddress coneAddr;				
			};

    	
    	class PlanProxy {
    	    cast::cdl::WorkingMemoryAddress planAddress;
    	};


    };
};

#endif
