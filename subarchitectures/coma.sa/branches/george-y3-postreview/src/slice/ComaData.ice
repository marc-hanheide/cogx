#ifndef COMADATA_ICE
#define COMADATA_ICE

#include <SpatialProbabilities.ice>

module comadata {
    
    sequence<string> InstanceSet;
    sequence<string> ConceptSet;
    sequence<string> ValueSet;
    sequence<string> PropertySet;
    sequence<long> PlaceIdSet;
    
    interface ComaReasonerInterface {
        string testReverseString(string s);
	InstanceSet getAllInstances(string concept);
	InstanceSet getRelatedInstances(string instance);
	InstanceSet getRelatedInstancesByRelation(string instance, string relation);
	InstanceSet getInverseRelatedInstancesByRelation(string instance, string relation);
	InstanceSet getInstancesByPropertyValue(string property, string value);
	InstanceSet getImmediateRelatedInstancesByRelation(string instance, string relation);
	ConceptSet getAllConcepts(string instance);
	ConceptSet getAllSubconcepts(string concept);
	ConceptSet getBasicLevelConcepts(string instance);
	ConceptSet getMostSpecificConcepts(string instance);
	ValueSet getPropertyValues(string instance, string property);
	PropertySet getRelationsBetweenInstances(string instance1, string instance2);
	bool addInstance(string instance, string concept);
	bool addRelation(string instance1, string relation, string instance2);
	bool deleteInstance(string instance);
	bool deleteRelation(string instance1, string relation, string instance2);
	bool isInstanceOf(string instance, string concept);
	bool isSubRelation(string relation1, string relation2);
	bool areInstancesRelated(string instance1, string relation, string instance2);
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
    
    sequence<string> labelList;
    
    class ComaRoom {
    	int roomId;
    	string seedPlaceInstance;
    	PlaceIdSet containedPlaceIds;
    	SpatialProbabilities::ProbabilityDistribution categories;
    };
    
};

#endif
