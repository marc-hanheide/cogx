/**
 * 
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.Gaussian;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.NormalValues;

/**
 * @author marc
 * 
 */
public class GaussianFactory extends AbstractProxyFactory<DistributionValues, Gaussian> {


	@Override
	public Gaussian create(DistributionValues pd) {
		return Gaussian.create(pd);
	}

}
