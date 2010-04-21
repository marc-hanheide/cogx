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
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.PointerValue;


public class FeatureContentUtils {

	
	public static List<FeatureValueProbPair> getValuesInBelief (Belief b, String featlabel) throws BeliefException {
		
		if (b != null && b.content != null) {
			return getValuesInDistribution(b.content, featlabel);
		}
		else {
			throw new BeliefException("ERROR, null value");
		}
	}
	
	public static List<FeatureValueProbPair> getValuesInDistribution (ProbDistribution distrib, String featlabel) throws BeliefException {
		
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
		
		if (distrib instanceof BasicProbDistribution && ((BasicProbDistribution)distrib).key.equals(featlabel)) {
			DistributionValues vals = ((BasicProbDistribution)distrib).values;
			if (vals instanceof FeatureValues) {
				return ((FeatureValues)vals).values;
			}
		}
		
		return new LinkedList<FeatureValueProbPair>();
	}
	
	
	
	
	public static BasicProbDistribution getFeatureForFeatureValueInBelief (Belief b, FeatureValue featval) throws BeliefException {
		
		if (b == null || b.content == null) {
			throw new BeliefException("ERROR, null value");
		}
		else {
			return getFeatureForFeatureValueInDistribution (b.content, featval);
		}
	}
	
	
	public static BasicProbDistribution getFeatureForFeatureValueInDistribution (ProbDistribution distrib, 
			FeatureValue featval) throws BeliefException {
		
	if (distrib instanceof DistributionWithExistDep) {
		return getFeatureForFeatureValueInDistribution (((DistributionWithExistDep)distrib).Pc, featval);
	}
	
	if (distrib instanceof CondIndependentDistribs) {
		for (String feat : ((CondIndependentDistribs)distrib).distribs.keySet()) {
			BasicProbDistribution result = getFeatureForFeatureValueInDistribution(((CondIndependentDistribs)distrib).distribs.get(feat),featval);
			if (result!= null) {
				return result;
			}
		}
	}
	
	if (distrib instanceof BasicProbDistribution) {
		if (((BasicProbDistribution)distrib).values.getClass().equals(FeatureValues.class)) {
			for (FeatureValueProbPair pair : ((FeatureValues)((BasicProbDistribution)distrib).values).values) {
				if (pair.val.equals(featval)) {
					return ((BasicProbDistribution)distrib);
				}
			}
		}
	}
	
		return null;
	}
	
	
	
	
	
	
	public static List<FeatureValueProbPair> getAllPointerValuesInBelief (Belief b) throws BeliefException {
		if (b != null && b.content != null) {
			return getAllPointerValuesInDistribution(b.content);
		}
		else {
			throw new BeliefException("ERROR, null value");
		}
	}
	
	
	
	public static List<FeatureValueProbPair> getAllPointerValuesInDistribution (ProbDistribution distrib) 
		throws BeliefException {
		
		List<FeatureValueProbPair> all = new LinkedList<FeatureValueProbPair>();

		if (distrib == null) {
			throw new BeliefException("ERROR, null value");
		}
		
		if (distrib instanceof DistributionWithExistDep) {
			return getAllPointerValuesInDistribution (((DistributionWithExistDep)distrib).Pc);
		}
		
		if (distrib instanceof CondIndependentDistribs) {
			
			for (String feat : ((CondIndependentDistribs)distrib).distribs.keySet()) {
				all.addAll(getAllPointerValuesInDistribution(((CondIndependentDistribs)distrib).distribs.get(feat)));
			}
		}
		
		if (distrib instanceof BasicProbDistribution) {
			DistributionValues vals = ((BasicProbDistribution)distrib).values;
			if (vals instanceof FeatureValues) {
				for (FeatureValueProbPair pair : ((FeatureValues)vals).values) {
					if (pair.val instanceof PointerValue) {
						all.add(pair);
					}
				}
			}
		}
		
		return all;
	}
	
	
	public static void addAnotherValueInBelief (Belief b, String featlabel, 
			FeatureValueProbPair newPair) throws BeliefException {
		
		System.out.println("adding another value for featlabel: " + featlabel);
		
		if (b == null || b.content == null) {
			throw new BeliefException("ERROR, null value");
		}
		
		else {
			addAnotherValueInDistribution (b.content, featlabel, newPair);
		}
	}
	
	
	public static void addAnotherValueInBasicProbDistribution(BasicProbDistribution distrib, FeatureValueProbPair newPair) {
		if (distrib.values.getClass().equals(FeatureValues.class)) {
			FeatureValues featvals = (FeatureValues)distrib.values;
			
			// here, we should verify that the prob distribution is still coherent!!
			featvals.values.add(newPair);
		}
	}
	
	
	public static void addAnotherValueInDistribution (ProbDistribution distrib, String featlabel, 
			FeatureValueProbPair newPair) throws BeliefException {
		
		if (distrib instanceof DistributionWithExistDep) {
			addAnotherValueInDistribution (((DistributionWithExistDep)distrib).Pc, featlabel, newPair);
		}
		
		if (distrib instanceof CondIndependentDistribs) {
						
			if (((CondIndependentDistribs)distrib).distribs.containsKey(featlabel)) {
							
				ProbDistribution featDistrib = ((CondIndependentDistribs)distrib).distribs.get(featlabel);
				
				addAnotherValueInBasicProbDistribution((BasicProbDistribution)featDistrib, newPair);
					
			}
		}
	}
}

