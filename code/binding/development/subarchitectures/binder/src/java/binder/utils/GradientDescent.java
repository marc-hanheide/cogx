package binder.utils;


import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.utils.ProbDistribUtils;

public class GradientDescent {

	public static float getMaximum (ProbabilityDistribution distrib) {
		
		float result = 0.0f;
		if (distrib == null) {
			log("ERROR: distribution == null, aborting");
		}
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getMaximum((DiscreteProbabilityDistribution) distrib);
		}
		
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getMaximum((CombinedProbabilityDistribution) distrib);
		}
		
		else {
			log("Sorry, only discrete or combined feature distributions are handled right now");
			log("Used class: " + distrib.getClass());
		}
		
		return result;
	}
	
	
	public static float getMaximum (DiscreteProbabilityDistribution distrib) {
		
		float maxProb = 0.0f;
		if (distrib.assignments != null) {
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
			DiscreteProbabilityAssignment assignment = distrib.assignments[i];
			if (assignment.prob > maxProb) {
				maxProb = assignment.prob;
			}
		}
		}
		return maxProb;
	}
	
	
	
	public static DiscreteProbabilityAssignment getBestAssignment  (DiscreteProbabilityDistribution distrib) {
		
		float maxProb = 0.0f;
		DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;
		if (distrib.assignments != null) {
			for (int i = 0 ; i <distrib.assignments.length ; i++) {
			DiscreteProbabilityAssignment assignment = distrib.assignments[i];
			if (assignment.prob > maxProb) {
				maxProb = assignment.prob;
				bestAssign = assignment;
			}
		}
		}
		return bestAssign;
	}
	
	
public static float getMaximum (CombinedProbabilityDistribution distrib) {
	
	
//	log("Searching maximum value for a combined probability distribution...");
	float maxProb = 0.0f;
	
	DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;
	
	DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
	float total = 0.0f;
	for (int i = 0; i < firstDistrib.assignments.length; i++) {
		
		DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
		float probValue = ProbDistribUtils.getProbabilityValue(distrib, assignment);
	//	log("ass: " + ProbDistribUtils.getDiscreteProbabilityAssignmentPrettyPrint(assignment) + " --> " + probValue);

		total += probValue;
		if (probValue > maxProb) {
			maxProb = probValue;			
		}
		
	}
		
	return maxProb;
		
	}


public static DiscreteProbabilityAssignment getBestAssignment (CombinedProbabilityDistribution distrib) {
	
	
	log("Searching maximum value for a combined probability distribution...");
	float maxProb = 0.0f;
	
	DiscreteProbabilityAssignment bestAssign = new DiscreteProbabilityAssignment() ;
	
	DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
	float total = 0.0f;
	for (int i = 0; i < firstDistrib.assignments.length; i++) {
		
		DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
		float probValue = ProbDistribUtils.getProbabilityValue(distrib, assignment);
	//	log("ass: " + ProbDistribUtils.getDiscreteProbabilityAssignmentPrettyPrint(assignment) + " --> " + probValue);

		total += probValue;
		if (probValue > maxProb) {
			maxProb = probValue;
			bestAssign = assignment;	
		}
	}
	
	log("Best assignment: " + ProbDistribUtils.getDiscreteProbabilityAssignmentPrettyPrint(bestAssign));
	
	return bestAssign;
		
	}

	

private static boolean isFeatValuePairInAssignment (FeatureValuePair pair, DiscreteProbabilityAssignment assign) {
	for (int i = 0; i < assign.featurepairs.length ; i++) {
		FeatureValuePair pair2 = assign.featurepairs[i];
		if (pair.featlabel.equals(pair2.featlabel) && pair.featvalue.equals(pair2.featvalue)) {
			return true;
		}
	}
	return false;
}

/**
public static Union getBestUnion(UnionDistribution distribution) {
	
	float maxValue = 0.0f;
	Union bestUnion = null;
	
	for (int i = 0; i < distribution.alternativeUnions.length; i++) {
		
		Union curUnion = distribution.alternativeUnions[i];
		float val = getMaximum(curUnion.distribution);
		if (val > maxValue) {
			maxValue = val;
			bestUnion = curUnion;
		}	
	}
	
	return bestUnion;
}
*/



public static UnionConfiguration getBestUnionConfiguration(Vector<UnionConfiguration> configs) {
	
	float maxAverage = 0.0f;
	UnionConfiguration bestConfig = null;
	
	for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
		UnionConfiguration config = e.nextElement();
		
		float multiplication = 1.0f;
		
		for (int i = 0; i < config.includedUnions.length ; i++) {		
			Union union = config.includedUnions[i];
			multiplication = multiplication * getMaximum(union.distribution);
		}
		float average = multiplication / (config.includedUnions.length + 0.0f);
	
		if (average > maxAverage) {
			maxAverage = average;
			bestConfig = config;
		}
	}
	
	return bestConfig;
}



public static UnionConfiguration getBestUnionConfiguration(AlternativeUnionConfigurations configs) {
	Vector<UnionConfiguration> unionconfigsV = new Vector<UnionConfiguration>();
	for (int i = 0 ; i < configs.alterconfigs.length ; i++) {
		unionconfigsV.add(configs.alterconfigs[i]);
	}
	return getBestUnionConfiguration (unionconfigsV);
}


public static Union getUnionWithMaximumProbability (Union union) {
	
	Union newUnion = new Union();
	newUnion.entityID = union.entityID;
	newUnion.features = new Feature[union.features.length];
	
	DiscreteProbabilityAssignment bestAssign = null;
	
	if (union.distribution == null) {
		log("ERROR: distribution == null, aborting");
	}
	if (union.distribution.getClass().equals(DiscreteProbabilityDistribution.class)) {
		bestAssign = getBestAssignment((DiscreteProbabilityDistribution)union.distribution);
		
	}
	
	else if (union.distribution.getClass().equals(CombinedProbabilityDistribution.class)) {
		bestAssign = getBestAssignment((CombinedProbabilityDistribution)union.distribution);
	}
	
	else {
		log("Sorry, only discrete or combined feature distributions are handled right now");
		log("Used class: " + union.distribution.getClass());
	}
	
	for (int i = 0; i < union.features.length ; i++) {
		newUnion.features[i] = new Feature();
		newUnion.features[i].featlabel = union.features[i].featlabel;
		for (int j = 0 ; j < union.features[i].alternativeValues.length ; j++) {
			FeatureValuePair pair =  new FeatureValuePair (union.features[i].featlabel, union.features[i].alternativeValues[j]);
			if (isFeatValuePairInAssignment (pair, bestAssign)) {
				newUnion.features[i].alternativeValues = new FeatureValue[1];
				newUnion.features[i].alternativeValues[0] = union.features[i].alternativeValues[j];
			}
		}
	}
	
	newUnion.includedProxies = union.includedProxies;
	newUnion.probExists = union.probExists;
	newUnion.distribution = union.distribution;
	
	log("OK, extracted a new union with maximum probability");
	return newUnion;
}
	
	public static void log(String s) {
		System.out.println("[GradientDescent] " + s);
	}
}
