/**
 * 
 */
package de.dfki.lt.tr.beliefs.util;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;

/**
 * @author marc
 * 
 */
public final class ProbFormula<T2 extends Proxy<? extends dFormula>> {
	/**
	 * @param formula
	 * @param probability
	 */
	public ProbFormula(T2 formula, float probability) {
		this.formula = formula;
		this.probability = probability;
	}

	private final T2 formula;
	private final float probability;

	/**
	 * @return the formula
	 */
	public T2 getFormula() {
		return formula;
	}

	/**
	 * @return the probability
	 */
	public float getProbability() {
		return probability;
	}
}
