/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import java.util.Iterator;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.FormulasFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.util.ProbFormula;

/**
 * @author marc
 * 
 */
public class FormulaDistribution extends GenericBasicDistribution<Formulas> implements Iterable<ProbFormula>{

	public static FormulaDistribution create(ProbDistribution o) {
		return new FormulaDistribution(o);
	}

	public static FormulaDistribution create() {
		return new FormulaDistribution(new BasicProbDistribution("",
				new FormulaValues(new LinkedList<FormulaProbPair>())));
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected FormulaDistribution(ProbDistribution content) {
		super(FormulasFactory.get(), content);
	}

	public static FormulaDistribution create(Distribution<ProbDistribution> i) {
		return create(i.get());
	}

	/**
	 * @param b
	 * @param prob
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#add(boolean, double)
	 */
	public void add(boolean b, double prob) {
		getDistribution().add(b, prob);
	}

	/**
	 * @param formula
	 * @param d
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#add(de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula,
	 *      double)
	 */
	public void add(dFormula formula, double d) {
		getDistribution().add(formula, d);
	}

	/**
	 * @param f
	 * @param prob
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#add(float, double)
	 */
	public void add(float f, double prob) {
		getDistribution().add(f, prob);
	}

	/**
	 * @param f
	 * @param prob
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#add(int, double)
	 */
	public void add(int f, double prob) {
		getDistribution().add(f, prob);
	}

	/**
	 * @param f
	 * @param d
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#add(java.lang.String, double)
	 */
	public void add(String f, double d) {
		getDistribution().add(f, d);
	}

	/**
	 * @param query
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#getProb(int)
	 */
	public float getProb(int query) {
		return getDistribution().getProb(query);
	}

	/**
	 * @param query
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#getProb(java.lang.String)
	 */
	public float getProb(String query) {
		return getDistribution().getProb(query);
	}

//	/**
//	 * @param query
//	 * @return
//	 * @see de.dfki.lt.tr.beliefs.data.Formulas#getProb(cast.cdl.WorkingMemoryAddress)
//	 */
//	public float getProb(WorkingMemoryAddress query) {
//		return getDistribution().getProb(query);
//	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#iterator()
	 */
	public Iterator<ProbFormula> iterator() {
		return getDistribution().iterator();
	}

	/**
	 * @param init
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#addAll(java.lang.Object[][])
	 */
	public void addAll(Object[][] init) {
		getDistribution().addAll(init);
	}

	/**
	 * @param query
	 * @param prob
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#setProb(int, float)
	 */
	public void setProb(int query, double prob) {
		getDistribution().setProb(query, prob);
	}

	/**
	 * @param query
	 * @param prob
	 * @see de.dfki.lt.tr.beliefs.data.Formulas#setProb(java.lang.String,
	 *      double)
	 */
	public void setProb(String query, double prob) {
		getDistribution().setProb(query, prob);
	}

//	/**
//	 * @param query
//	 * @param prob
//	 * @see de.dfki.lt.tr.beliefs.data.Formulas#setProb(cast.cdl.WorkingMemoryAddress,
//	 *      double)
//	 */
//	public void setProb(WorkingMemoryAddress query, double prob) {
//		getDistribution().setProb(query, prob);
//	}
//	
	public int size() {
		return getDistribution().size();
	}

}
