package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.Formula;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;

public class FormulaFactory extends
		AbstractProxyFactory<Formula> {

	@Override
	public Formula create(Ice.Object pd) {
		return Formula.create(pd);
	}

}
