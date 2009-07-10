

module binder {

module autogen {

module binderEssentials {

class FeatureValue { };

class StringValue extends FeatureValue {
	string val;
};

class FeatureValuePair {
	string featlabel;
	FeatureValue featvalue;
};

sequence<FeatureValuePair> FeatureValuePairs;


// dictionary<FeatureValuePairs,float> MultivariateProbabilityDistribution;

class FeatureValueAssignmentsWithProbValue {
	FeatureValuePairs featurepairs;
	float probValue;
};

sequence<FeatureValueAssignmentsWithProbValue> DiscreteProbabilityDistribution;

class ProbabilityDistribution { };


sequence<FeatureValue> FeatureValues;

class Feature {
	string featlabel;
	FeatureValues alternativeValues;
};

sequence<Feature> FeaturesList;

class PerceivedEntity {
	string entityID;
	float probExists;
	FeaturesList features;
	ProbabilityDistribution distribution;
};


class Proxy extends PerceivedEntity {
	string subarchId;
};

sequence<Proxy> ProxySeq;

class Union extends PerceivedEntity {
	ProxySeq includedProxies;
};

};


module bayesianNetworkEssentials {


	class BayesianNetworkNode {
		binderEssentials::Feature feat;
	};
	
	
	class FeatureValueCorrelation {
		binderEssentials::FeatureValue value1;
		binderEssentials::FeatureValue value2;
		float condProb;
	};
		
	sequence<FeatureValueCorrelation> FeatureValueCorrelations;
	
	class BayesianNetworkEdge {
		string incomingFeature;
		string outgoingFeature;
		FeatureValueCorrelations correlations;
	};
	
	sequence<BayesianNetworkNode> BayesianNetworkNodes;
	sequence<BayesianNetworkEdge> BayesianNetworkEdges;
		
	class BayesianNetwork {
		BayesianNetworkNodes nodes;
		BayesianNetworkEdges edges;
	};
	
};

};
};
