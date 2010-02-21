package binder.gui;

import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.utils.FeatureValueUtils;

/*
 * --------------------------------------------------------------------------------------------------
 */

class ConditionedAlternative implements Comparable<Object> {
	
	private String alternative_conditioned;
	private String alternative;
	private Float conditional_probability;
	
	public ConditionedAlternative(FeatureValueCorrelation feature) {
		alternative_conditioned = FeatureValueUtils.toString(feature.value1);
		alternative = FeatureValueUtils.toString(feature.value2);
		conditional_probability = feature.condProb;
	}
	
	public ConditionedAlternative(String alternative_conditioned, String alternative, Float conditional_probability) {
		this.alternative_conditioned = alternative_conditioned;
		this.alternative = alternative;
		this.conditional_probability = conditional_probability;
	}
	
	public String getAlternativeConditioned() {
		return alternative_conditioned;
	}
	
	public String getAlternative() {
		return alternative;
	}
	
	public Float getConditionalProbability() {
		return conditional_probability;
	}
	
	public void setAlternativeConditioned(String alternative_conditioned) {
		this.alternative_conditioned = alternative_conditioned;
	}
	
	public void setAlternative(String alternative) {
		this.alternative = alternative;
	}
	
	public void setConditionalProbability(Float conditional_probability) {
		this.conditional_probability = conditional_probability;
	}

	public int compareTo(Object compare_to) {
		if(compare_to == null) {
			return -1;
		}
		
		if(compare_to instanceof ConditionedAlternative) {
			ConditionedAlternative that = (ConditionedAlternative) compare_to;
			
			int result = this.alternative.compareToIgnoreCase(that.alternative);
			
			if(result == 0) {
				return this.alternative_conditioned.compareToIgnoreCase(that.alternative_conditioned);
			}
			return result;
		}
		return -1;
	}
}

