/**
 * 
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.Gaussian;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class GaussianFactory extends AbstractProxyFactory<DistributionValues, Gaussian> {

	private static GaussianFactory singleton = null;

	public static GaussianFactory get() {
		if (singleton == null)
			singleton = new GaussianFactory();
		return singleton;
	}


	@Override
	public Gaussian create(DistributionValues pd) {
		return Gaussian.create(pd);
	}

}
