#ifndef COMADATA_ICE
#define COMADATA_ICE

module comadata {
    
    sequence<string> instanceSet;
    sequence<string> conceptSet;
    sequence<long> placeIdSet;
    
    interface ComaReasonerInterface {
        string testReverseString(string s);
		instanceSet getAllInstances(string concept);
		instanceSet getRelatedInstances(string instance);
		instanceSet getRelatedInstancesByRelation(string instance, string relation);
		instanceSet getAllConcepts(string instance);
		conceptSet getAllSubconcepts(string concept);
		bool addInstance(string instance, string concept);
		bool addRelation(string instance1, string relation, string instance2);
		bool deleteInstance(string instance);
		bool deleteRelation(string instance1, string relation, string instance2);
		bool isInstanceOf(string instance, string concept);
		string executeSPARQL(string sparqlQuery);
    };
    
    class ComaRoom {
    	int roomId;
    	string seedPlaceInstance;
    	placeIdSet containedPlaceIds;
    	conceptSet concepts;
    };
    
};

#endif
