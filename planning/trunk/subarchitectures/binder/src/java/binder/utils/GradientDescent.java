package binder.utils;


import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.utils.ProbabilityDistributionUtils;

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
	
	
public static float getMaximum (CombinedProbabilityDistribution distrib) {
	
	
//	log("Searching maximum value for a combined probability distribution...");
	float maxProb = 0.0f;
	
	DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
	for (int i = 0; i < firstDistrib.assignments.length; i++) {
		DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
		
		float probValue = ProbabilityDistributionUtils.getProbabilityValue(distrib, assignment);
		
		if (probValue > maxProb) {
			maxProb = probValue;
		}
		
	}
	
	return maxProb;
		
	}
	
	
	public static void log(String s) {
		System.out.println("[GradientDescent] " + s);
	}
}
