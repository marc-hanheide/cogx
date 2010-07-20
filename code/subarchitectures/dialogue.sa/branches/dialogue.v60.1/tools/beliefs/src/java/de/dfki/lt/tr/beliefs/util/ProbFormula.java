/**
 * 
 */
package de.dfki.lt.tr.beliefs.util;

import de.dfki.lt.tr.beliefs.data.formulas.Formula;

/**
 * @author marc
 * 
 */
public final class ProbFormula {
	/**
	 * @param formula
	 * @param probability
	 */
	public ProbFormula(Formula formula, float probability) {
		this.formula = formula;
		this.probability = probability;
	}

	private final Formula formula;
	private final float probability;

	/**
	 * @return the formula
	 */
	public Formula getFormula() {
		return formula;
	}

	/**
	 * @return the probability
	 */
	public float getProbability() {
		return probability;
	}
}
