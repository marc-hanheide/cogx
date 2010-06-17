/**
 * 
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class FormulasFactory extends
		AbstractProxyFactory<DistributionValues, Formulas> {

	private static FormulasFactory singleton = null;

	public static FormulasFactory get() {
		if (singleton == null)
			singleton = new FormulasFactory();
		return singleton;
	}

	@Override
	public Formulas create(DistributionValues pd) {
		return Formulas.create(pd);
	}

}
