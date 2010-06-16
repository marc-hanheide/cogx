/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.FormulasFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;

/**
 * @author marc
 * 
 */
public class FormulaDistribution extends GenericBasicDistribution<Formulas> {

	public static FormulaDistribution create(Ice.Object o) {
		return new FormulaDistribution(o);
	}

	public static FormulaDistribution create() {
		return new FormulaDistribution(new BasicProbDistribution("", new FormulaValues(new LinkedList<FormulaProbPair>())));
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected FormulaDistribution(Ice.Object content) {
		super(new FormulasFactory(), content);
	}

}
