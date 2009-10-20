 
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        
 

/**
 * Here should go a general introduction to the data structures specified in this file
 */ 
 
  
#include <cast/slice/CDL.ice>


module binder {

module autogen {

/***** CORE CLASSES *****/
module core {

/**
 * General abstract class for feature value 
 */
class FeatureValue { 
	float independentProb ;
	cast::cdl::CASTTime timeStamp;
};

/**
 * Sequence of feature values
 */
sequence<FeatureValue> FeatureValues;
 
 
/**
 * A feature = feature label and a list of possible feature values
 *
 * TODO: extend this specification to handle continuous feature values
 *
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
	cast::cdl::CASTTime timeStamp;
	FeaturesList features;
	ProbabilityDistribution distribution;
};



 
/** 
 * A proxy
 */ 
class Proxy extends PerceivedEntity {
	 cast::cdl::WorkingMemoryPointer origin;
};


dictionary<string, string> StringMap;
dictionary<string, cast::cdl::WorkingMemoryPointer> WMPointerMap;



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
	ProxySeq orphanProxies;
	double configProb;	
};
	 
sequence<UnionConfiguration> UnionConfigurationSeq ;

class AlternativeUnionConfigurations {
	UnionConfigurationSeq alterconfigs;
};


/***
 * A wrapper for a map between source data ids and the proxies they
 * are turned into. This is maintained by the abstract writer
 * classes. Currently only a 1-to-1 mapping is supported, but this may
 * have to change in the future.
 */

  class OriginMap {
    /**
     * The component which created the map.
     */
    string componentID;

    /**
     * The map itself
     */
    StringMap sourceID2ProxyID;
    WMPointerMap proxyID2WMPointer;
  };



};


module specialentities {

class RelationProxy extends core::Proxy { 
	core::Feature source;
	core::Feature target;
};
 
class GroupProxy extends core::Proxy {
	core::FeaturesList connections;
};
 
class RelationUnion extends core::Union {
	core::Feature psource;
	core::Feature ptarget;
	core::Feature usource;
	core::Feature utarget;
};

class GroupUnion extends core::Union {
	core::FeaturesList connections;
};

class PhantomProxy extends core::Proxy { };

class PhantomUnion extends core::Union { };


};


/** POSSIBLE FEATURE VALUES */
module featvalues {

/** 
 * A feature value instantiated as a string object
 */
class StringValue extends core::FeatureValue {
	string val;
};

/** 
 * A feature value instantiated as a (CAST) address
 */
class AddressValue extends core::FeatureValue {
	string val;
};

/** 
 * A feature value instantiated as an integer
 */
class IntegerValue extends core::FeatureValue {
	int val;
};

/** 
 * A feature value instantiated as a float
 */
class FloatValue extends core::FeatureValue {
	float val;
};

/** 
 * A feature value instantiated as a boolean
 */
class BooleanValue extends core::FeatureValue {
	bool val;
};

class UnknownValue extends core::FeatureValue { } ;

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
