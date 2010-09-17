#ifndef COMADATA_ICE
#define COMADATA_ICE

#include <SpatialProbabilities.ice>

module comadata {
    
    sequence<string> InstanceSet;
    sequence<string> ConceptSet;
    sequence<long> PlaceIdSet;
    
    interface ComaReasonerInterface {
        string testReverseString(string s);
		InstanceSet getAllInstances(string concept);
		InstanceSet getRelatedInstances(string instance);
		InstanceSet getRelatedInstancesByRelation(string instance, string relation);
		InstanceSet getAllConcepts(string instance);
		ConceptSet getAllSubconcepts(string concept);
		bool addInstance(string instance, string concept);
		bool addRelation(string instance1, string relation, string instance2);
		bool deleteInstance(string instance);
		bool deleteRelation(string instance1, string relation, string instance2);
		bool isInstanceOf(string instance, string concept);
		string executeSPARQL(string sparqlQuery);
    };
    
    sequence<string> BindingRow;
    sequence<BindingRow> BindingTable;
    dictionary<string, int> StringToIntMap;
    
    struct QueryResults {
    	string query;
    	StringToIntMap varPosMap;
    	BindingTable bt;   
    };
    
    interface HFCInterface {
		QueryResults querySelect(string q);
		string ping();    	
    };
    
    
    class ComaRoom {
    	int roomId;
    	string seedPlaceInstance;
    	PlaceIdSet containedPlaceIds;
    	SpatialProbabilities::ProbabilityDistribution categories;
    };
    
};

#endif
