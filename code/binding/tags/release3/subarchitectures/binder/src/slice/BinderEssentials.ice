

module binder {

module autogen {

/***** CORE CLASSES *****/
module core {

/**
 * General abstract class for feature value 
 */
class FeatureValue { 
	float independentProb ;
};

/**
 * Sequence of feature values
 */
sequence<FeatureValue> FeatureValues;


/**
 * A feature = feature label and a list of possible feature values
 */
class Feature {
	string featlabel;
	FeatureValues alternativeValues;
};

 /** 
  * General abstract class for probability distributions
  */ 
class ProbabilityDistribution { };


/**
 * List of features
 */
sequence<Feature> FeaturesList;

/** 
 * A perceived entity
 */ 
class PerceivedEntity {
	string entityID;
	float probExists;
	FeaturesList features;
	ProbabilityDistribution distribution;
};


/** 
 * A proxy
 */ 
class Proxy extends PerceivedEntity {
	string subarchId;
};

sequence<Proxy> ProxySeq;


/** 
 * A binding union
 */ 
class Union extends PerceivedEntity {
	ProxySeq includedProxies;
};


sequence<Union> UnionSequence;

class UnionConfiguration {
	UnionSequence includedUnions;
	float configProb;
};
	
sequence<UnionConfiguration> UnionConfigurationSeq ;

class AlternativeUnionConfigurations {
	UnionConfigurationSeq alterconfigs;
};

};


/** POSSIBLE FEATURE VALUES */
module featvalues {

/** 
 * A feature value instantiated as a string object
 */
class StringValue extends core::FeatureValue {
	string val;
};

};




/** PROBABILITY DISTRIBUTIONS */
module distributions {

/**
 * A feature-value pair is a feature label associated with a specific feature value 
 */
class FeatureValuePair {
	string featlabel;
	core::FeatureValue featvalue;
};

/**
 * Set of probability distributions
 */ 
sequence<core::ProbabilityDistribution> ProbabilityDistributions;


/** GENERAL DISCRETE PROBABILITY DISTRIBUTIONS */
module discrete {

/**
 * Sequence of feature-value pairs
 */
sequence<FeatureValuePair> FeatureValuePairs;


class DiscreteProbabilityAssignment {
	FeatureValuePairs featurepairs;
	float prob;
};

sequence<DiscreteProbabilityAssignment> DiscreteProbabilityAssignments;


class DiscreteProbabilityDistribution extends core::ProbabilityDistribution {
	DiscreteProbabilityAssignments assignments;
};

};


/** MULTIPLIED PROBABILITY DISTRIBUTIONS */
module combined {

enum OperationType {
	MULTIPLIED,
	DIVIDED
	};

class CombinedProbabilityDistribution extends core::ProbabilityDistribution {
	ProbabilityDistributions distributions;
	OperationType opType;
};

};




};


module bayesiannetworks {


	class BayesianNetworkNode {
		core::Feature feat;
	};
	
	
	class FeatureValueCorrelation {
		core::FeatureValue value1;
		core::FeatureValue value2;
		float condProb;
	};
		
	sequence<FeatureValueCorrelation> FeatureValueCorrelations;
	
	class BayesianNetworkEdge {
		BayesianNetworkNode incomingNode;
		BayesianNetworkNode outgoingNode;
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
