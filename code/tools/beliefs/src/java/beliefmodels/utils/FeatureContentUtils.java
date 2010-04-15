package beliefmodels.utils;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionValues;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;


public class FeatureContentUtils {

	
	public List<FeatureValueProbPair> getValuesInBelief (Belief b, String featlabel) throws BeliefException {
		
		if (b != null && b.content != null) {
			return getValuesInDistribution(b.content, featlabel);
		}
		else {
			throw new BeliefException("ERROR, null value");
		}
	}
	
	public List<FeatureValueProbPair> getValuesInDistribution (ProbDistribution distrib, String featlabel) throws BeliefException {
		
		if (distrib == null) {
			throw new BeliefException("ERROR, null value");
		}
		
		if (distrib instanceof DistributionWithExistDep) {
			return getValuesInDistribution (((DistributionWithExistDep)distrib).Pc, featlabel);
		}
		
		if (distrib instanceof CondIndependentDistribs) {
			
			for (String feat : ((CondIndependentDistribs)distrib).distribs.keySet()) {
				if (feat.equals(featlabel)) {
					return getValuesInDistribution(((CondIndependentDistribs)distrib).distribs.get(feat), featlabel);
				}
			}
		}
		
		if (distrib instanceof BasicProbDistribution) {
			DistributionValues vals = ((BasicProbDistribution)distrib).values;
			if (vals instanceof FeatureValues) {
				return ((FeatureValues)vals).values;
			}
		}
		
		return new LinkedList<FeatureValueProbPair>();
	}
}
