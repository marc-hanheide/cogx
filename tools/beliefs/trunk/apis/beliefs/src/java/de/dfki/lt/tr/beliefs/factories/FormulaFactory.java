package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.Formula;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;


public class FormulaFactory extends
		AbstractProxyFactory<Formula<? extends dFormula>> {

	public Formula<BooleanFormula> create(boolean b) {
		return Formula.create(BooleanFormula.class, new BooleanFormula(-1, b));
	}

	public Formula<dFormula> create(dFormula formula) {
		return Formula.create(dFormula.class, formula);
	}

	public Formula<FloatFormula> create(double b) {
		return Formula.create(FloatFormula.class, new FloatFormula(-1, (float) b));
	}

	@Override
	public Formula<? extends dFormula> create(Ice.Object pd) {
		return Formula.create(dFormula.class, pd);
	}

	public Formula<IntegerFormula> create(int b) {
		return Formula.create(IntegerFormula.class, new IntegerFormula(-1, b));
	}

	public Formula<ElementaryFormula> create(String b) {
		return Formula.create(ElementaryFormula.class, new ElementaryFormula(-1, b));
	}
}
