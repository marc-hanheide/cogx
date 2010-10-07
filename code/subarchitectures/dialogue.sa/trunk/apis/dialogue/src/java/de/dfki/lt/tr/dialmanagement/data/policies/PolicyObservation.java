package de.dfki.lt.tr.dialmanagement.data.policies;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

public class PolicyObservation extends FormulaWrapper {

	// the unique identifier for the node
	private String id;
		
	private int type;
	
	// minimum probability for the observation
	float minProb = 0.0f;
	
	// maximum probability for the observation
	float maxProb = 1.0f;
	

	public PolicyObservation (dFormula content, float minProb, float maxProb) {
		super(content);	
		this.minProb = minProb;
		this.maxProb = maxProb;
	}
	

	public PolicyObservation (dFormula content) {
		super(content);
	}
	
	public PolicyObservation (String content) {
		super(content);
	}
	
	public PolicyObservation (String content, float minProb, float maxProb) {
		super(content);
		this.minProb = minProb;
		this.maxProb = maxProb;
	}

	public boolean matchesWithObservation (Observation obs) {
		
		for (FormulaWrapper alternative : obs.getAlternatives()) {
			if (alternative.equals(this) && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {				
				return true;
			}
		}
		return false;
	}
	

	/**
	 * Returns the node identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}
	

	public String toString () {
		return FormulaUtils.getString(content) + " (" + minProb + ", " + maxProb + ")";
	}
}
