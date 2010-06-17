/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.FormulasFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class FormulaDistribution extends GenericBasicDistribution<Formulas> {

	public static FormulaDistribution create(ProbDistribution o) {
		return new FormulaDistribution(o);
	}

	public static FormulaDistribution create() {
		return new FormulaDistribution(new BasicProbDistribution("", new FormulaValues(new LinkedList<FormulaProbPair>())));
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected FormulaDistribution(ProbDistribution content) {
		super(new FormulasFactory(), content);
	}

}
