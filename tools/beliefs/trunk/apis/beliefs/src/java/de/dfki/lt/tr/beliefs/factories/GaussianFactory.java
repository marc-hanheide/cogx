/**
 * 
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.Gaussian;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.NormalValues;

/**
 * @author marc
 * 
 */
public class GaussianFactory extends AbstractProxyFactory<Gaussian> {

	public Gaussian create(double m, double v) {
		NormalValues nv = new NormalValues(m, v);
		return create(nv);
	}

	@Override
	public Gaussian create(Ice.Object pd) {
		return Gaussian.create(pd);
	}

}
