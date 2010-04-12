package beliefmodels.utils;


import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.DistributionWithExistDep;

public class DistributionUtils {

	
	public static float getExistenceProbability (Belief b) {
		
		if (b.content instanceof DistributionWithExistDep) {
			
			return ((DistributionWithExistDep)b.content).existProb;
		}

		return 1.0f;
	}
	
	
	

	public static void setExistenceProbability (Belief b, float newExistProb) {
		
		if (b.content instanceof DistributionWithExistDep) {
			
			((DistributionWithExistDep)b.content).existProb = newExistProb;
		}

	}
}
