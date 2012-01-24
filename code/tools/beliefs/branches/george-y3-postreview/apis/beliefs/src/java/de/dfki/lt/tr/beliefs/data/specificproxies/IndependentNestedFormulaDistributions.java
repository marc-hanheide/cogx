/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import java.util.HashMap;
import java.util.LinkedList;

import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistributionList;
import de.dfki.lt.tr.beliefs.factories.specific.IndependentFormulaDistributionsFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribList;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/** 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentNestedFormulaDistributions extends
		GenericIndependentDistributionList<IndependentFormulaDistributions> {

	public static IndependentNestedFormulaDistributions create(
			ProbDistribution o) {
		return new IndependentNestedFormulaDistributions(o);
	}

	protected IndependentNestedFormulaDistributions(ProbDistribution content) {
		super(IndependentFormulaDistributionsFactory.get(), content);
	}

	static public void main(String[] argv) {
		ProbDistribution pd = new CondIndependentDistribList(
				new LinkedList<ProbDistribution>());
		IndependentNestedFormulaDistributions nested = IndependentNestedFormulaDistributions
				.create(pd);
		IndependentFormulaDistributions inner = IndependentFormulaDistributions
				.create(new CondIndependentDistribs(
						new HashMap<String, ProbDistribution>()));
		
		FormulaDistribution df = FormulaDistribution.create();
		df.add(15, 1.0);
		inner.put("cost", df);
		inner.put("val0", df);
		inner.put("val1", df);
		
		
		nested.add(inner);
		inner = IndependentFormulaDistributions
				.create(new CondIndependentDistribs(
						new HashMap<String, ProbDistribution>()));
		df = FormulaDistribution.create();
		df.add("test2", 1.5);
		inner.put("hurga", df);
		nested.add(inner);
		System.out.println(IceXMLSerializer.toXMLString(nested.get()));
	}

}
