#ifndef DEFAULTDATA_ICE
#define DEFAULTDATA_ICE

#include <cast/slice/CDL.ice>

/**
 * Data structures representing the default knowledge maintained by default.sa
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module DefaultData 
{
	sequence<string> BindingRow;
   	sequence<BindingRow> BindingTable;
	dictionary<string, int> StringToIntMap;

	/** Results returned by the forward chainer 
	    in response to a QDL query. */
	struct QdlQueryResults 
	{
		string query;
		StringToIntMap varPosMap;
		BindingTable bt;
	};

	/** Interface of the forward chainer. */
	interface HFCInterface 
	{
		QdlQueryResults querySelect(string q);
	};

	/** Abstract class for all discrete variable values. */
	class RandomVariableValue
	{
	};
	
	/** Value of a random variable represented as a string. */
	class StringRandomVariableValue extends RandomVariableValue
	{
		string value;
	};

	/** Value of a random variable represented as an int. */
	class IntRandomVariableValue extends RandomVariableValue
	{
		int value;
	};

	/** Value of a random variable represented as a bool. */
	class BoolRandomVariableValue extends RandomVariableValue
	{
		bool value;
	};

	/** A sequence of random variable values. */
	sequence <RandomVariableValue> RandomVariableValues;
	
	/** Defines probability value of a set of values of 
	    a joint probability distribution. */
	struct JointProbabilityValue
	{
		RandomVariableValues variableValues;
		float probability;
	};

	/** The table defining the probability mass function for
	    discrete variables. */
	sequence <JointProbabilityValue> ProbabilityMassFunction;

	/** Discrete probability distribution returned for
	    various queries. */
	struct DiscreteProbabilityDistribution
	{
		// Human readable description of the distribution. 
		string description;

		// Mapping between variable names and positions in the sequence.
		StringToIntMap variableNameToPositionMap;

		// The probability mass function for this distribution
		ProbabilityMassFunction massFunction;		
	};

	/** Interface to the def::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		DiscreteProbabilityDistribution query(string queryStr);
	};
};

#endif // DEFAULTDATA_ICE
