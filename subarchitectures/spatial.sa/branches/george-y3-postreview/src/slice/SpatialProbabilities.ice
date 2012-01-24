#ifndef SPATIAL_PROBABILITIES_ICE
#define SPATIAL_PROBABILITIES_ICE

#include <cast/slice/CDL.ice>

/**
 * Data structures defining probability distributions used
 * in Spatial.SA.
 * @author Andrzej Pronobis
 */

module SpatialProbabilities 
{
	/** Just a string to int map. */
	dictionary<string, int> StringToIntMap;

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
	struct ProbabilityDistribution
	{
		// Human readable description of the distribution. 
		string description;

		// Mapping between variable names and positions in the sequence.
		StringToIntMap variableNameToPositionMap;

		// The probability mass function for this distribution
		ProbabilityMassFunction massFunction;
	};

};

#endif // SPATIAL_PROBABILITIES_ICE

