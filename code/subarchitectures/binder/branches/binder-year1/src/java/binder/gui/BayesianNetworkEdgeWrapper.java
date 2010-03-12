package binder.gui;

import java.util.LinkedList;
import java.util.List;
import java.util.TreeMap;
import java.util.TreeSet;
import binder.gui.BayesianNetworkNodeWrapper;
import binder.gui.ConditionedAlternative;

class BayesianNetworkEdgeWrapper implements Comparable<Object> {
	
	private BayesianNetworkNodeWrapper source;
	private BayesianNetworkNodeWrapper target;
	
	private TreeSet<ConditionedAlternative> conditioned_alternatives;
	
	
	public BayesianNetworkEdgeWrapper(BayesianNetworkNodeWrapper source,
			BayesianNetworkNodeWrapper target,
			TreeSet<ConditionedAlternative> conditioned_alternatives) {
		this.source = source;
		this.target = target;
		this.conditioned_alternatives = new TreeSet<ConditionedAlternative>(conditioned_alternatives);
	}
	
	public void removeFeatureConditioned(
			ConditionedAlternative conditioned_alternative) {
		this.conditioned_alternatives.remove(conditioned_alternative);
	}
	
	public void modifyFeatureConditioned(
			ConditionedAlternative conditioned_alternative) throws Exception {
		if(!this.conditioned_alternatives.contains(conditioned_alternative)) {
			throw new Exception("[ADDFEATURECONDITIONED] alternative does not exists!]");
		}
		
		String dependent_alternative = conditioned_alternative.getAlternative();
		
		float sum = 0f;
		for(ConditionedAlternative current : this.conditioned_alternatives) {
			if(dependent_alternative.equals(current.getAlternative())) {
				sum += current.getConditionalProbability();
			}
		}
		if(sum+ conditioned_alternative.getConditionalProbability() > 1f) {
			throw new Exception("[ADDFEATURECONDITIONED] probability exceeds one!]");
		}
		
		this.conditioned_alternatives.add(conditioned_alternative);
	}

	public void addFeatureConditioned(
			ConditionedAlternative conditioned_alternative) throws Exception {
		if(this.conditioned_alternatives.contains(conditioned_alternative)) {
			throw new Exception("[ADDFEATURECONDITIONED] alternative already exists!]");
		}
		
		String dependent_alternative = conditioned_alternative.getAlternative();
		
		float sum = 0f;
		for(ConditionedAlternative current : this.conditioned_alternatives) {
			if(dependent_alternative.equals(current.getAlternative())) {
				sum += current.getConditionalProbability();
			}
		}
		if(sum+ conditioned_alternative.getConditionalProbability() > 1.) {
			throw new Exception("[ADDFEATURECONDITIONED] probability exceeds one!]");
		}
		this.conditioned_alternatives.remove(conditioned_alternative);
		boolean success =  this.conditioned_alternatives.add(conditioned_alternative);
		assert success;
	}

	private Float getTotalProbability() {
		float sum = 0.0f;
		for(ConditionedAlternative alternative : this.conditioned_alternatives) {
			sum += alternative.getConditionalProbability();
		}
		return sum;
	}

	public BayesianNetworkNodeWrapper getNodeSource() {
		return this.source;
	}

	public BayesianNetworkNodeWrapper getNodeTarget() {
		return this.target;
	}
	
	/**
	 * 
	 */
	
	public int compareTo(Object compare_to) {
		int result = -1;
		if(compare_to == null) {
			return -1;
		}
		if(compare_to instanceof BayesianNetworkEdgeWrapper) {
			BayesianNetworkEdgeWrapper that = (BayesianNetworkEdgeWrapper) compare_to;
			
			result = this.source.getFeatureLabelName().compareTo(that.source.getFeatureLabelName());
			
			if(result == 0) {
				return this.target.getFeatureLabelName().compareTo(that.target.getFeatureLabelName());
			}
		}
		
		return result;
	}

	public int getNumberOfAlternatives() {
		return this.conditioned_alternatives.size();
	}

	public List<ConditionedAlternative> getConditionedAlternatives() {
		return new LinkedList<ConditionedAlternative>(this.conditioned_alternatives);
	}
	
	public boolean hasFeatureConditioned(String alternative,
			String alternative_conditioned) {
		return this.conditioned_alternatives.contains(
			new ConditionedAlternative(alternative_conditioned, alternative, 0f));
	}
}
