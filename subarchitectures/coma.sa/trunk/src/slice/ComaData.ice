#ifndef COMADATA_ICE
#define COMADATA_ICE

module comadata {
    
    sequence<string> instanceSet;
    sequence<string> conceptSet;
    
    interface ComaReasonerInterface {
        string testReverseString(string s);
	instanceSet getAllInstances(string concept);
	instanceSet getRelatedInstances(string instance);
	instanceSet getRelatedInstancesByRelation(string instance, string relation);
	instanceSet getAllConcepts(string instance);
	bool addInstance(string instance, string concept);
	bool addRelation(string instance1, string relation, string instance2);
    };
};

#endif
